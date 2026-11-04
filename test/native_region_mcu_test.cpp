// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_shi_registers.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <span>
#include <string>

namespace {
using namespace blofeld;
constexpr unsigned extal = 12000000, timebase = 540000000, boot = 0xff0000;
constexpr unsigned port_b = 1, all_pins = 0xff, datum = 0x42;
constexpr unsigned mcu_entry = 0x8000, after_load = mcu_entry + 2, after_store = mcu_entry + 4;
constexpr unsigned timer_control = 0xffff8f, shi_control = 0xffff91, shi_config = 0xa9;
// Independent HC12 slots: reset5, LDAB immediate1, STAB direct2.
constexpr std::uint64_t period = 45, mcu_period = 2 * period;
constexpr std::uint64_t mcu_reset_ticks = 5 * mcu_period;
constexpr std::uint64_t held_ticks = mcu_reset_ticks + mcu_period;
constexpr std::uint64_t shared_tick = held_ticks + (2 * mcu_period);
// Authored FM BSET #0,X:$ffff8f and RESET. RESET has seven base cycles.
constexpr unsigned set_timer_enable = 0x010f20, reset_peripherals = 0x000084;
constexpr unsigned bit_pc = boot + 8, reset_pc = bit_pc + 1, reset_base_cycles = 7;
std::uint64_t assertions = 0;

void require(const bool accepted, const std::string &message) {
  ++assertions;
  if (!accepted) { throw Fault(message); }
}

McuFlashImage image() {
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  constexpr unsigned nop = 0xa7;
  constexpr unsigned protection = 0x7f0d;
  constexpr unsigned security = 0x7f0f;
  constexpr unsigned unsecured = 0xfe;
  // Authored HCS12 LDAB #$42; STAB PORTB; NOP padding and a real reset vector.
  constexpr std::array<std::uint8_t, 4> program{0xc6, datum, 0x5b, port_b};
  bytes.fill(nop);
  std::ranges::copy(program, bytes.begin());
  bytes[protection] = all_pins;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = 0x80;
  bytes.back() = 0;
  return McuFlashImage::from_known(bytes);
}

McuDspRuntimeInputs runtime_inputs() {
  return {.timing = OwnDspAutomaticTiming::functional_native_regions,
          .maximum_cycles_per_period = 32,
          .writes = McuWriteOrder::before_mcu,
          .external_writes = McuWriteOrder::before_mcu,
          .wait_dsp_first = true,
          .reset_dsp_first = true,
          .initial_reset = OwnDspResetInputs{
              .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}};
}

struct Device {
  McuMachine mcu{
      image(),
      BLOFELD_DEFAULT_SLA,
      McuRunContract{.clock = {.oscillator_hz = extal,
                               .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
                     .memory_timing = McuRunMemory::unstretched_internal}};
  OwnDspMachine dsp{{.extal_hz = extal,
                     .timebase_hz = timebase,
                     .switch_timing = DspClockSwitch::functional_region_boundary}};
  std::shared_ptr<const bool> identity = std::make_shared<const bool>(true);
  std::unique_ptr<McuDspRuntime> runtime;
  bool peer = false, peer_done = false;
  unsigned bit_effects = 0, reset_effects = 0;
  std::uint64_t input_ticks = 0, bit_ticks = 0, reset_ticks = 0;
  std::string events;

  explicit Device(const bool external_peer, const bool peripheral_program) : peer(external_peer) {
    std::array<std::uint32_t, 128> rom{};
    if (peripheral_program) {
      rom[bit_pc - boot] = set_timer_enable;
      rom[reset_pc - boot] = reset_peripherals;
    }
    dsp.bus().install_rom({.start = boot, .words = rom});
    mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
    mcu.bus().write(hc12::portb, 0);
    mcu.bus().write(hc12::ddrb, all_pins);
    runtime = std::make_unique<McuDspRuntime>(runtime_inputs(), mcu, dsp, external());
    require(runtime->end_ticks() == mcu_reset_ticks &&
                dsp.clock().elapsed_ticks() == mcu_reset_ticks,
            "The genuine MCU reset-vector sequence supplies five initial IPBus periods");
    attach();
  }

  DspRuntimeEvents external() {
    if (!peer) { return {}; }
    return {.identity = identity,
            .next_ticks = [this]() -> std::optional<std::uint64_t> {
              return peer_done ? std::nullopt : std::optional{shared_tick};
            },
            .observe =
                [this](const std::uint64_t ticks) {
                  require(ticks == shared_tick && dsp.cpu().progress().instructions == 1,
                          "The real peer runs before the second native completion at the tie");
                  events += 'E';
                  peer_done = true;
                },
            .order = DspPeerOrder::peer_first,
            .maximum_events_per_period = 16};
  }

  McuDspRuntime::CycleInputs inputs() {
    return [this] {
      input_ticks = dsp.clock().elapsed_ticks();
      return DspDmaInputs{.known_requests = 15,
                          .timing = DspDmaTiming::reference_sampled_cycles,
                          .grant = DspDmaGrant::available};
    };
  }

  void effect(const DspRuntimeCycle &event) {
    if (peer && event.phase == DspRuntimePhase::region && event.completion_ticks == shared_tick) {
      require(event.region && event.region->dispatches == 1,
              "The tied receipt contains the actual last native instruction");
      events += 'D';
    }
    if (event.phase != DspRuntimePhase::peripheral) { return; }
    require(input_ticks == event.completion_ticks && event.start_ticks == event.completion_ticks,
            "Actual peripheral inputs are prepared at completion, after elapsed prefix time");
    if (event.result.native_pc == bit_pc) {
      ++bit_effects;
      bit_ticks = event.completion_ticks;
      require(dsp.bus().timers().read(timer_control, 1) == 1 && event.cycle_known,
              "The real bit callback enables the actual timer exactly once");
    } else if (event.result.native_pc == reset_pc) {
      ++reset_effects;
      reset_ticks = event.completion_ticks;
      require(dsp.bus().timers().read(timer_control, 1) == 0 &&
                  dsp.bus().read({.space = DspSpace::x, .address = shi_control}) ==
                      (shi::idle | shi::transmit_empty) &&
                  !event.cycle_known && !dsp.bus().last_dma_cycle(),
              "Guest RESET really clears timer/SHI/DMA and reports its unknown fresh DMA cursor");
    } else {
      throw Fault("Unexpected actual native peripheral PC");
    }
  }

  McuDspRuntime::Observer observer() {
    return [this](const DspRuntimeCycle &event) { effect(event); };
  }

  void attach() {
    mcu.set_cycle_observer([this](const Hc12BusCycle &cycle) {
      const bool shared = (cycle.kind == 'w' || cycle.kind == 'W') && cycle.address == port_b;
      runtime->begin_cycle(cycle, shared, inputs(), observer());
    });
    mcu.bus().set_observer([this](const BusEvent &event) {
      if (!event.write || event.address != port_b) { return; }
      runtime->observe_write(
          event,
          [this, event] {
            require(event.value == datum &&
                        mcu.bus().observe_pads(McuPort::b, all_pins).levels == datum,
                    "The actual MCU store and output pads contain the authored byte");
            events += 'W';
          },
          inputs(),
          observer());
    });
  }

  void step() {
    require(mcu.cpu().step().instruction_executed, "The actual HCS12 instruction retires");
    runtime->end_instruction();
  }

  void release() {
    step();
    require(mcu.cpu().state().pc == after_load && mcu.cpu().state().d == datum &&
                runtime->end_ticks() == held_ticks && dsp.clock().elapsed_ticks() == held_ticks,
            "The real first MCU instruction supplies the complete DSP reset hold: pc=" +
                hex(mcu.cpu().state().pc) + " d=" + hex(mcu.cpu().state().d) +
                " runtime=" + std::to_string(runtime->end_ticks()) +
                " dsp=" + std::to_string(dsp.clock().elapsed_ticks()) +
                " mcu-local=" + std::to_string(mcu.run_clock().elapsed_ticks()) +
                " mcu-timebase=" + std::to_string(mcu.run_clock().timebase_hz()));
    runtime->release_dsp_reset();
  }
};

void test_real_mcu_store_and_peer_tie() {
  Device device(true, false);
  device.release();
  device.step();
  const auto native = device.dsp.cpu().jit_statistics();
  require(device.events == "EDW" && device.runtime->end_ticks() == shared_tick &&
              device.dsp.clock().elapsed_ticks() == shared_tick &&
              device.dsp.cpu().registers().pc == boot + 2 &&
              device.dsp.cpu().progress().instructions == 2 &&
              device.dsp.clock().native_cycle_units() == 2 &&
              device.mcu.cpu().state().pc == after_store && native.native_alu_instructions == 2 &&
              native.helper_instructions == 0 && !device.runtime->faulted(),
          "Real MCU/peer/native event ordering retains two actual DSP instructions and both CPUs");
  std::cout << "ACTUAL_COUPLED order=" << device.events << " tick=" << shared_tick
            << " native=" << native.native_alu_instructions << " helpers=0\n";
}

void test_real_bit_reset_and_pending_paired_restore() {
  Device original(false, true);
  original.release();
  original.dsp.bus().write({.space = DspSpace::x, .address = shi_control, .value = shi_config});
  // TL/TC are not reset-defined. Supply explicit authored component inputs
  // before the real BSET enables the timer; the strict unknown-load guard stays.
  original.dsp.bus().timers().write({.address = timer_control - 1, .value = 0});
  original.dsp.bus().timers().write({.address = timer_control - 2, .value = all_pins});
  for (unsigned attempt = 0; attempt < 12 && original.bit_effects == 0; ++attempt) {
    original.step();
  }
  original.step(); // One actual MCU NOP consumes two of RESET's seven DSP periods.
  require(original.bit_effects == 1 && original.reset_effects == 0 &&
              original.dsp.clock().native_cycle_units() ==
                  original.dsp.cpu().progress().estimated_cycles + 2 &&
              original.dsp.cpu().registers().pc == reset_pc,
          "A real MCU boundary splits the seven-cycle guest RESET after the genuine BSET");
  const auto saved = original.runtime->checkpoint();
  Device clone(false, false);
  clone.runtime.reset();
  clone.runtime = std::make_unique<McuDspRuntime>(saved, clone.mcu, clone.dsp);
  clone.attach();
  for (unsigned attempt = 0; attempt < 12 && original.reset_effects == 0; ++attempt) {
    original.step();
    clone.step();
  }
  require(original.bit_effects == 1 && clone.bit_effects == 0 && original.reset_effects == 1 &&
              clone.reset_effects == 1 && original.reset_ticks == clone.reset_ticks &&
              original.bit_ticks == held_ticks + ((2 + 8 + 2) * period) &&
              original.reset_ticks - original.bit_ticks == reset_base_cycles * period &&
              original.dsp.cpu().registers() == clone.dsp.cpu().registers() &&
              original.dsp.clock().elapsed_ticks() == clone.dsp.clock().elapsed_ticks() &&
              original.mcu.cpu().state().pc == clone.mcu.cpu().state().pc &&
              !original.runtime->faulted() && !clone.runtime->faulted(),
          "Paired MCU/DSP restore executes the pending real RESET once at its independent cost");
  std::cout << "ACTUAL_GUEST_RESET bit-tick=" << original.bit_ticks
            << " reset-tick=" << original.reset_ticks << " paired-restore=1\n";
}
}

int main() {
  try {
    test_real_mcu_store_and_peer_tie();
    test_real_bit_reset_and_pending_paired_restore();
    std::cout << "Native region MCU integration: " << assertions << " assertions passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
