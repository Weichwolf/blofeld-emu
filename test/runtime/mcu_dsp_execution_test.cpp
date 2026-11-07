// SPDX-License-Identifier: GPL-3.0-only
#include "cpu12/memory.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp56300/core.hpp"
#include "isa.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "mcu_checkpoint_fixture.hpp"
#include "runtime/mcu_dsp_runtime.hpp"
#include "support/isa.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace {
using namespace blofeld;
using blofeld::test::require;
namespace isa = cpu12::isa;
using cpu12::Byte;
using isa::Opcode;
constexpr unsigned oscillator = 12000000;
constexpr std::uint64_t timebase = 540000000, scale = 9;
constexpr unsigned entry = blofeld::test::entry;
constexpr unsigned horizon = 1000000;
constexpr unsigned boot_address = 0xff0000;
constexpr Byte timer_on = 0x80, unmasked = 0xc0, loop_back = 0xfd;

void cold_reset_boundary() {
  const McuRunContract contract{
      .clock = {.oscillator_hz = oscillator,
                .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
      .memory_timing = McuRunMemory::internal_flash_reads};
  McuMachine mcu(blofeld::test::checkpoint_image(isa::wai), BLOFELD_DEFAULT_SLA, contract, {3});
  DspMachine dsp({.extal_hz = oscillator,
                  .timebase_hz = timebase,
                  .switch_timing = DspClockSwitch::functional_region_boundary});
  McuDspRuntime runtime({.reset_dsp_first = true,
                         .initial_reset = DspResetInputs{.mode_known = 15,
                                                         .mode_levels = 5,
                                                         .pinit = false,
                                                         .recovery_cycles = 2}},
                        mcu,
                        dsp);
  require(runtime.end_ticks() == 0 && mcu.system_reset_active() &&
              dsp.clock().hardware_reset_active() && dsp.clock().elapsed_ticks() == 0 &&
              dsp.cpu().state().progress.instructions == 0 && !runtime.faulted(),
          "Both cores remain in actual reset at time zero without inventing a clock period");
  mcu.set_reset_period_observer(
      [&](const McuResetPeriod period) { runtime.begin_reset_period(period, {}); });
  mcu.set_span_observer([&](McuSpan /*span*/) { runtime.synchronize({}); });
  constexpr unsigned edges = 128 + 3 + 64;
  require(mcu.advance_system_reset(edges, McuResetPad::high) == edges,
          "The real MCU reset sequence reaches its vector boundary");
  runtime.end_system_reset();
  constexpr std::uint64_t completed_ticks = ((std::uint64_t{edges} * 5) + 50) * scale;
  require(!mcu.system_reset_active() && mcu.cpu().state().pc == entry &&
              dsp.clock().hardware_reset_active() && runtime.end_ticks() == completed_ticks &&
              dsp.clock().elapsed_ticks() == completed_ticks &&
              dsp.cpu().state().progress.instructions == 0 && !runtime.faulted(),
          "Qualified MCU reset edges and vector time advance the held DSP without executing it");
}

void connected_execution() {
  constexpr auto program = isa::program(isa::opcode<Opcode::ldaa_immediate>(),
                                        std::array<Byte, 1>{timer_on},
                                        isa::opcode<Opcode::staa_extended>(),
                                        std::array<Byte, 2>{0, hc12::tscr1},
                                        isa::dex,
                                        isa::opcode<Opcode::bne_relative>(),
                                        std::array<Byte, 1>{loop_back},
                                        isa::wai);
  const McuRunContract contract{
      .clock = {.oscillator_hz = oscillator,
                .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
      .memory_timing = McuRunMemory::internal_flash_reads};
  McuMachine mcu(blofeld::test::checkpoint_image(program), BLOFELD_DEFAULT_SLA, contract);
  blofeld::test::drive_interrupts(mcu.bus());
  constexpr unsigned trips = 10000;
  mcu.cpu().set_state({.pc = entry, .x = trips, .sp = blofeld::test::stack, .ccr = unmasked});
  DspMachine dsp({.extal_hz = oscillator,
                  .timebase_hz = timebase,
                  .switch_timing = DspClockSwitch::functional_region_boundary});
  McuDspRuntime runtime({.initial_reset = DspResetInputs{.mode_known = 15,
                                                         .mode_levels = 5,
                                                         .pinit = false,
                                                         .recovery_cycles = 2}},
                        mcu,
                        dsp);
  constexpr auto jump = dsp56300::isa::opcode<dsp56300::isa::Opcode::jmp_absolute_short>().front();
  constexpr std::array bootstrap{jump};
  dsp.bus().install_rom({.start = boot_address, .words = bootstrap});
  dsp.bus().write({.address = 0, .value = dsp56300::isa::nop.front()});
  dsp.bus().write({.address = 1, .value = jump});
  runtime.release_dsp_reset();
  const auto inputs = [] {
    return DspDmaInputs{.timing = DspDmaTiming::reference_sampled_cycles,
                        .grant = DspDmaGrant::available};
  };
  unsigned spans = 0;
  unsigned writes = 0;
  mcu.set_span_observer([&](McuSpan /*span*/) {
    ++spans;
    runtime.synchronize(inputs);
  });
  mcu.bus().set_observer([&](const BusEvent &event) {
    require(event.write && event.address == hc12::tscr1 &&
                runtime.end_ticks() == mcu.run_clock().elapsed_ticks() * scale &&
                dsp.clock().elapsed_ticks() == runtime.end_ticks(),
            "Actual MCU I/O observes the DSP at the same completed device-time boundary");
    runtime.observe_write(event, [&] { ++writes; }, inputs);
  });
  const auto first = mcu.run_until(horizon);
  runtime.end_instruction();
  require(first.retired == 2 && writes == 1 && spans == 2 && !runtime.faulted(),
          "The MCU publishes preceding work and completes its real I/O without a bus-slot offer");
  const auto second = mcu.run_until(horizon);
  runtime.end_instruction();
  require(second.retired == 2 * trips + 1 && mcu.cpu().waiting() && spans == 3 &&
              dsp.cpu().state().progress.instructions != 0 &&
              runtime.end_ticks() == mcu.run_clock().elapsed_ticks() * scale &&
              dsp.clock().elapsed_ticks() == runtime.end_ticks() && !runtime.faulted(),
          "Both native cores execute a long connected interval on one coherent virtual timeline");
  const auto before = runtime.end_ticks();
  constexpr unsigned idle = 100;
  require(mcu.advance_wait(idle) == idle, "The MCU retains its actual WAI while the DSP runs");
  runtime.end_instruction();
  require(runtime.end_ticks() == before + (std::uint64_t{idle} * 10 * scale) && spans == 4 &&
              dsp.clock().elapsed_ticks() == runtime.end_ticks() && !runtime.faulted(),
          "One completed idle interval advances the DSP without executing MCU instructions");
}
}

int main() {
  try {
    cold_reset_boundary();
    connected_execution();
    std::cout << "Connected native MCU/DSP time, I/O effects and WAI passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
