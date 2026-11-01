// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp56300/core.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "mcu_dsp_reset_fixture.hpp"
#include "mcu_dsp_runtime_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <vector>

namespace blofeld::test::mcu_reset {
namespace {
dsp56300::Registers nop_registers(const std::uint64_t cycles) {
  constexpr unsigned reset_status = 0xc00300;
  constexpr unsigned reset_omr = 0x305;
  dsp56300::Registers expected{};
  expected.pc = mcu_dsp::boot;
  expected.sr = reset_status;
  expected.omr = reset_omr;
  expected.m.fill(dsp56300::word_mask);
  if (cycles >= 3) { expected.pc += static_cast<unsigned>((cycles - 1) / 2); }
  return expected;
}

std::uint64_t retirements(const std::uint64_t cycles) {
  return cycles < 2 ? 0 : (cycles - 2) / 2;
}
}

mcu_dsp::Options options(const std::span<const std::uint8_t> program) {
  return {.mcu_program = program, .dsp_program = {}, .mcu_hz = mcu_hz, .shared_address = u_data};
}

void configure(Fixture &f) {
  f.mcu.bus().write(ram, marker);
  f.dsp.bus().write({.space = DspSpace::x, .address = dsp_ram, .value = marker});
  f.mcu.bus().write(u_data, 0);
  f.mcu.bus().write(u_direction, 1);
  f.attach();
  require(f.runtime->end_ticks() == initial_ticks &&
              f.mcu.run_clock().timebase_hz() == mcu_timebase,
          "Actual MCU reset did not establish five literal60tick periods");
}

void held(Fixture &f,
          const std::uint64_t until,
          const std::uint64_t completed,
          const std::uint64_t clk2) {
  require(f.dsp.clock().hardware_reset_active() && f.runtime->completed_cycles() == completed &&
              f.dsp.clock().elapsed_ticks() == until &&
              f.dsp.clock().hardware_reset_ticks() == until - (completed * mcu_dsp::dsp_period) &&
              f.dsp.clock().timer_clk2_edges() == clk2 && f.runtime->end_ticks() == until &&
              f.dsp.cpu().progress() == dsp56300::Progress{} &&
              f.dsp.cpu().registers() == nop_registers(0) && f.mcu.bus().read(ram) == marker &&
              f.dsp.bus().read({.space = DspSpace::x, .address = dsp_ram}) == marker,
          "Native MCU time or retained RAM advanced DSP/F1/F2/DMA/CLK2 while held reset");
}

void observe_nops(Fixture &f, const std::uint64_t recovery_end, const std::uint64_t previous) {
  f.on_cycle = [&f, recovery_end, previous](const DspRuntimeCycle &cycle) {
    const auto count = (cycle.completion_ticks - recovery_end) / mcu_dsp::dsp_period;
    const auto retired = retirements(count);
    require(cycle.completion_ticks == recovery_end + (count * mcu_dsp::dsp_period) &&
                cycle.completion_ticks - cycle.start_ticks == mcu_dsp::dsp_period &&
                f.runtime->completed_cycles() == previous + count &&
                f.dsp.cpu().registers() == nop_registers(count) &&
                f.dsp.cpu().progress() ==
                    dsp56300::Progress{.instructions = retired, .estimated_cycles = retired},
            "Native NOP F1/F2,PCU,retirement or complete register state lost literal expectations");
  };
}

void running(Fixture &f, const Running expected) {
  const auto cycles = (expected.until - expected.recovery_end) / mcu_dsp::dsp_period;
  require(f.runtime->end_ticks() == expected.until &&
              f.mcu.run_clock().elapsed_ticks() * (mcu_dsp::common_hz / mcu_timebase) ==
                  expected.until &&
              !f.dsp.clock().hardware_reset_active() &&
              f.dsp.clock().hardware_reset_ticks() == expected.reset_ticks &&
              f.dsp.clock().elapsed_ticks() ==
                  expected.reset_ticks +
                      ((expected.previous_cycles + cycles) * mcu_dsp::dsp_period) &&
              f.runtime->completed_cycles() == expected.previous_cycles + cycles &&
              f.dsp.clock().timer_clk2_edges() == expected.previous_clk2 + (cycles / 2) &&
              f.dsp.cpu().registers() == nop_registers(cycles) &&
              f.dsp.cpu().progress().instructions == retirements(cycles) &&
              f.mcu.bus().read(ram) == marker &&
              f.dsp.bus().read({.space = DspSpace::x, .address = dsp_ram}) == marker,
          "Native hold/recovery/execution/CLK2 lifetime lost independent clocks,RAM or registers");
}

namespace {
void held_replay(const unsigned nops) {
  Fixture f(options());
  configure(f);
  f.run(nops);
  const auto cut = initial_ticks + (nops * mcu_period);
  held(f, cut);
  const auto snapshot = f.runtime->checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    f.restore(snapshot);
    held(f, cut);
    f.runtime->release_dsp_reset();
    observe_nops(f, cut + recovery);
    f.run(10);
    running(f,
            {.until = cut + (10 * mcu_period),
             .recovery_end = cut + recovery,
             .reset_ticks = cut + recovery});
  }
}

void native_wire(const unsigned padding) {
  // Actual LDAB #1;STAB extended PORTU,with independent1+3 IPBus periods.
  constexpr std::array<std::uint8_t, 5> store{0xc6, 1, 0x7b, 2, 0x78};
  std::vector<std::uint8_t> program(padding, mcu_dsp::nop);
  program.insert(program.end(), store.begin(), store.end());
  Fixture f(options(program));
  configure(f);
  const auto release = native_release + (padding * mcu_period);
  f.on_write = [&](const BusEvent &event) {
    const auto pins = f.mcu.bus().gpio_outputs(McuPort::u);
    require(event.value == 1 && (pins.mask & pins.known & pins.levels & 1U) != 0 &&
                f.runtime->end_ticks() == release,
            "Real PORTU store did not drive RESET high at its independently predicted write slot");
    f.runtime->release_dsp_reset();
    observe_nops(f, release + recovery);
  };
  f.run(padding + 2);
  require(f.writes.size() == 1 && f.runtime->end_ticks() == release + mcu_period &&
              f.dsp.clock().hardware_reset_remaining_ticks() == recovery - mcu_period,
          "Native STAB did not release reset before its genuine trailing O bus slot");
  f.run(10);
  running(f,
          {.until = release + (11 * mcu_period),
           .recovery_end = release + recovery,
           .reset_ticks = release + recovery});
}

void held_mcu_reset() {
  Fixture f(options());
  configure(f);
  f.runtime->begin_system_reset(McuResetSource::external, 3);
  f.attach();
  constexpr unsigned cut = 131;
  constexpr unsigned edges = 195;
  require(f.mcu.advance_system_reset(cut, McuResetPad::high) == cut,
          "Actual MCU SYSCLK reset cut did not complete");
  held(f, initial_ticks + (cut * sysclk_period));
  const auto snapshot = f.runtime->checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    f.restore(snapshot);
    require(f.mcu.advance_system_reset(edges - cut, McuResetPad::high) == edges - cut,
            "Restored native MCU reset did not complete its remaining SYSCLK edges");
    f.runtime->end_system_reset();
    constexpr auto until = initial_ticks + (edges * sysclk_period) + (5ULL * mcu_period);
    held(f, until);
    require(f.mcu.cpu().state().pc == mcu_dsp::entry,
            "Held-DSP replay did not preserve the actual MCU reset vector fetch");
  }
}
}
}

int main() {
  try {
    using namespace blofeld::test::mcu_reset;
    for (unsigned delay = 0; delay < 4; ++delay) {
      held_replay(delay + 1);
      native_wire(delay);
    }
    held_mcu_reset();
    running_reset();
    guards();
    std::cout << "Native MCU/dsp56300 reset wiring,retention,replay and guards passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
