// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <vector>

namespace {
using namespace blofeld;
constexpr unsigned mcu_hz = 18000000, dsp_hz = 12000000, common = 540000000, entry = 0x8000;
constexpr unsigned byte_mask = 0xff, u_data = 0x0278, u_direction = 0x027a;
constexpr std::uint64_t initial_ticks = 300, release = 480, recovery = 570, end = 660;
constexpr std::uint64_t dsp_period = 45;
constexpr std::uint64_t recovery_duration = 90, reset_before_edge = 569, total_reset = 570;
constexpr std::uint64_t trailing_end = 540, trailing_remaining = 30, total_elapsed = 660;

constexpr unsigned boot = 0xff0000;

void require(const bool condition, const char *message) {
  if (!condition) { throw Fault(message); }
}

DspDmaInputs cycle_inputs() {
  return {.known_requests = 15,
          .timing = DspDmaTiming::reference_sampled_cycles,
          .grant = DspDmaGrant::available};
}

dsp56300::Registers nop_registers(const std::uint64_t cycles) {
  constexpr unsigned reset_status = 0xc00300;
  constexpr unsigned reset_omr = 0x305;
  dsp56300::Registers expected{};
  expected.pc = boot;
  expected.sr = reset_status;
  expected.omr = reset_omr;
  expected.m.fill(dsp56300::word_mask);
  if (cycles >= 3) { expected.pc += static_cast<unsigned>((cycles - 1) / 2); }
  return expected;
}

struct Event {
  char actor;
  std::uint64_t ticks;
  bool operator==(const Event &) const = default;
};

McuFlashImage image() {
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  constexpr unsigned nop = 0xa7;
  constexpr unsigned protection = 0x7f0d;
  constexpr unsigned security = 0x7f0f;
  constexpr unsigned unsecured = 0xfe;
  // Own LDAB #1;STAB extended PORTU. Literal1+3 native periods.
  constexpr std::array<std::uint8_t, 5> code{0xc6, 1, 0x7b, 2, 0x78};
  bytes.fill(nop);
  for (unsigned i = 0; i < code.size(); ++i) { bytes.at(i) = code[i]; }
  bytes[protection] = byte_mask;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  return McuFlashImage::from_known(bytes);
}

McuRunContract mcu_inputs() {
  return {.clock = {.oscillator_hz = mcu_hz,
                    .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = McuRunMemory::unstretched_internal};
}

McuDspRuntimeInputs inputs(const McuWriteOrder order) {
  return {.timing = OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts,
          .maximum_cycles_per_period = 16,
          .writes = McuWriteOrder::before_mcu,
          .external_writes = order,
          .wait_dsp_first = true,
          .reset_dsp_first = true,
          .initial_reset = OwnDspResetInputs{
              .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}};
}

void prepare(McuMachine &mcu, OwnDspMachine &dsp) {
  mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
  mcu.bus().write(u_data, 0);
  mcu.bus().write(u_direction, 1);
  constexpr unsigned nops = 128;
  const std::array<std::uint32_t, nops> program{};
  dsp.bus().install_rom({.space = DspSpace::p, .start = boot, .words = program});
}

struct Fixture {
  Fixture(McuWriteOrder writes, DspPeerOrder clocks) : write_order(writes), clock_order(clocks) {
    prepare(mcu, dsp);
    runtime = std::make_unique<McuDspRuntime>(inputs(writes), mcu, dsp, events());
    require(runtime->end_ticks() == initial_ticks &&
                dsp.clock().hardware_reset_ticks() == initial_ticks,
            "Five actual MCU reset-vector periods retain native DSP hold from zero");
    attach();
  }

  ~Fixture() {
    mcu.set_cycle_observer({});
    mcu.bus().set_observer({});
  }

  Fixture(const Fixture &) = delete;
  Fixture &operator=(const Fixture &) = delete;

  DspRuntimeEvents events() {
    return {
        .identity = identity,
        .next_ticks = [this]() -> std::optional<std::uint64_t> {
          if (cursor == 0) { return release; }
          if (cursor == 1) { return recovery; }
          return std::nullopt;
        },
        .observe =
            [this](const std::uint64_t at) {
              require(runtime->completed_cycles() == 0 && at == (cursor == 0 ? release : recovery),
                      "Reset pin events never substitute native DSP work");
              const auto remaining = dsp.clock().hardware_reset_remaining_ticks();
              if (cursor == 0) {
                require(dsp.clock().hardware_reset_active() &&
                            remaining.has_value() == (write_order == McuWriteOrder::after_mcu),
                        "Exact release-time input observes asserted or recovering reset in its "
                        "declared MCU order");
                if (remaining) {
                  require(*remaining == recovery_duration,
                          "Release starts literal two-EXTAL recovery");
                }
              } else {
                const bool before = clock_order == DspPeerOrder::peer_first;
                require(dsp.clock().hardware_reset_active() == before &&
                            dsp.clock().hardware_reset_ticks() ==
                                (before ? reset_before_edge : total_reset),
                        "An exact recovery-time input observes the excluded or completed real "
                        "reset edge");
                if (before) {
                  require(remaining == 1,
                          "The excluded recovery edge retains precisely one EXTAL timebase tick");
                }
              }
              dsp.bus().timers().set_tio_drive(0, cursor == 0);
              trace.push_back({.actor = 'E', .ticks = at});
              ++cursor;
            },
        .order = clock_order,
        .maximum_events_per_period = 2};
  }

  void restore(const McuDspRuntimeCheckpoint &saved) {
    runtime.reset();
    cursor = 0;
    trace.clear();
    runtime = std::make_unique<McuDspRuntime>(saved, mcu, dsp, events());
    attach();
  }

  void attach() {
    const auto observed = [this](const DspRuntimeCycle &cycle) {
      require(cycle.completion_ticks == recovery + ((cycle.cycle + 1) * dsp_period) &&
                  dsp.cpu().registers() == nop_registers(cycle.cycle + 1) &&
                  dsp.cpu().progress() == dsp56300::Progress{},
              "First native F1/F2 periods after real recovery cannot execute an instruction");
      trace.push_back({.actor = 'D', .ticks = cycle.completion_ticks});
    };
    mcu.set_cycle_observer([this, observed](const auto cycle) {
      runtime->begin_cycle(
          cycle, cycle.kind == 'w' && cycle.address == u_data, cycle_inputs, observed);
    });
    mcu.bus().set_observer([this, observed](const auto &event) {
      if (!event.write || event.address != u_data) { return; }
      runtime->observe_write(
          event,
          [this] {
            const auto pads = mcu.bus().gpio_outputs(McuPort::u);
            require((pads.mask & pads.known & pads.levels & 1U) != 0,
                    "Actual native PORTU output supplies the declared laboratory reset release");
            trace.push_back({.actor = 'M', .ticks = runtime->end_ticks()});
            runtime->release_dsp_reset();
          },
          cycle_inputs,
          observed);
    });
  }

  void step(unsigned count) {
    for (unsigned i = 0; i < count; ++i) {
      require(mcu.cpu().step().instruction_executed,
              "Actual native MCU instructions advance reset control");
      runtime->end_instruction();
    }
  }

  McuMachine mcu{image(), BLOFELD_DEFAULT_SLA, mcu_inputs()};
  OwnDspMachine dsp{{.extal_hz = dsp_hz,
                     .timebase_hz = common,
                     .switch_timing = DspClockSwitch::reference_cycle_boundary}};
  std::shared_ptr<const bool> identity = std::make_shared<const bool>(true);
  unsigned cursor = 0;
  const McuWriteOrder write_order;
  const DspPeerOrder clock_order;
  std::vector<Event> trace;
  std::unique_ptr<McuDspRuntime> runtime;
};

void run(const McuWriteOrder writes, const DspPeerOrder clocks) {
  Fixture f(writes, clocks);
  const auto native = f.dsp.cpu().registers();
  const auto saved = f.runtime->checkpoint();
  std::vector<Event> suffix;
  for (unsigned replay = 0; replay < 2; ++replay) {
    f.restore(saved);

    f.trace.clear();
    f.step(2);
    require(f.runtime->end_ticks() == trailing_end && f.dsp.clock().hardware_reset_active() &&
                f.dsp.clock().hardware_reset_remaining_ticks() == trailing_remaining &&
                f.cursor == 1 && f.runtime->completed_cycles() == 0,
            "Real trailing MCU slot consumes recovery time without running native DSP periods");
    f.step(2);
    require(
        f.runtime->end_ticks() == end && f.mcu.run_clock().bus_cycles() == 11 &&
            f.mcu.cpu().state().pc == entry + 7 && f.mcu.cpu().state().d == 1 && f.cursor == 2 &&
            !f.dsp.clock().hardware_reset_active() &&
            f.dsp.clock().hardware_reset_ticks() == total_reset &&
            f.dsp.clock().elapsed_ticks() == total_elapsed && f.runtime->completed_cycles() == 2 &&
            f.dsp.clock().timer_clk2_edges() == 1 && f.dsp.cpu().registers() == native,
        "Independent60tick MCU,45tick DSP and90tick recovery ledgers agree after native restart");
    std::vector<char> tied;
    for (const auto event : f.trace) {
      if (event.ticks == release) { tied.push_back(event.actor); }
    }
    require(tied == (writes == McuWriteOrder::before_mcu ? std::vector<char>{'E', 'M'}
                                                         : std::vector<char>{'M', 'E'}),
            "Reset release preserves its explicit MCU/external order exactly once");
    if (replay == 0) { suffix = f.trace; }
    require(f.trace == suffix,
            "Trusted held snapshot and external cursor retain the complete reset suffix");
  }
}
}

int main() {
  try {
    for (const auto writes : {McuWriteOrder::before_mcu, McuWriteOrder::after_mcu}) {
      for (const auto clocks : {DspPeerOrder::peer_first, DspPeerOrder::dsp_first}) {
        run(writes, clocks);
      }
    }
    std::cout << "Native MCU/dsp56300 reset events:4 profiles,8 restored hold/recovery suffixes "
                 "passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
