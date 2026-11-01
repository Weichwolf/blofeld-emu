// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp56300/core.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu_dsp_reset_fixture.hpp"
#include "mcu_dsp_runtime_fixture.hpp"

#include <array>
#include <cstdint>

namespace blofeld::test::mcu_reset {
namespace {
void active_cycle() {
  constexpr std::array<std::uint8_t, 5> program{0xc6, 1, 0x7b, 2, 0x78};
  Fixture f(options(program));
  configure(f);
  f.on_write = [&](const BusEvent &) {
    f.runtime->release_dsp_reset();
    require(
        rejects([&] { static_cast<void>(f.runtime->checkpoint()); }).contains("healthy inactive"),
        "Successful nested reset release cleared the still-active outer signal guard");
  };
  unsigned refused = 0;
  f.on_cycle = [&](const DspRuntimeCycle &) {
    const auto registers = f.dsp.cpu().registers();
    const auto ticks = f.dsp.clock().elapsed_ticks();
    require(rejects([&] {
              f.runtime->begin_dsp_reset(mcu_dsp::reset_inputs());
            }).contains("consumed signal or inactive") &&
                rejects([&] {
                  f.runtime->release_dsp_reset();
                }).contains("consumed signal or inactive") &&
                f.dsp.cpu().registers() == registers && f.dsp.clock().elapsed_ticks() == ticks,
            "A native completion callback reset its active peer owner");
    ++refused;
  };
  f.run(5);
  require(refused == 3 && !f.runtime->faulted() && f.runtime->completed_cycles() == 3,
          "Reset permission escaped its signal callback into later native DSP completions");
}

void fractional_and_open_boundaries() {
  auto contract = options();
  contract.release = true;
  Fixture f(contract);
  configure(f);
  f.run(2);
  constexpr unsigned fractional = initial_ticks + (2 * mcu_period);
  constexpr unsigned recovery_end = initial_ticks + recovery;
  require(f.runtime->end_ticks() == fractional && f.dsp.clock().elapsed_ticks() == recovery_end &&
              rejects([&] {
                f.runtime->begin_dsp_reset(mcu_dsp::reset_inputs());
              }).contains("cannot discard unconsumed peer time") &&
              !f.runtime->faulted() && f.runtime->completed_cycles() == 0,
          "Reset discarded the30 uncompleted DSP ticks at a genuine consumed MCU boundary");
  require(f.mcu.cpu().step().instruction_executed,
          "Third actual MCU NOP did not execute through the aligned DSP boundary");
  const auto registers = f.dsp.cpu().registers();
  require(rejects([&] {
            f.runtime->begin_dsp_reset(mcu_dsp::reset_inputs());
          }).contains("cannot discard unconsumed peer time") &&
              !f.runtime->faulted() && f.dsp.cpu().registers() == registers,
          "An unclosed MCU instruction silently became a complete reset boundary");
  f.runtime->end_instruction();
  f.runtime->begin_dsp_reset(mcu_dsp::reset_inputs());
  held(f, initial_ticks + (3 * mcu_period), 2, 1);
}

void caught_signal_failure() {
  constexpr std::array<std::uint8_t, 5> program{0xc6, 1, 0x7b, 2, 0x78};
  Fixture f(options(program));
  configure(f);
  require(f.step().instruction_executed, "Native reset-write prefix did not execute LDAB");
  const auto saved = f.runtime->checkpoint();
  f.on_write = [&](const BusEvent &) {
    // Deliberately wrong board request: asserting again during held reset.
    require(!rejects([&] { f.runtime->begin_dsp_reset(mcu_dsp::reset_inputs()); }).empty() &&
                rejects([&] { f.runtime->end_instruction(); }).contains("healthy inactive"),
            "Failed nested reset cleared the still-active outer signal guard");
  };
  require(rejects([&] { static_cast<void>(f.step()); }).contains("faulted its runtime") &&
              f.runtime->faulted() && f.runtime->completed_cycles() == 0 &&
              f.dsp.clock().hardware_reset_active(),
          "A caught reset failure continued through the MCU signal callback as silent success");
  f.restore(saved);
  f.on_write = [&](const BusEvent &) { f.runtime->release_dsp_reset(); };
  require(f.step().instruction_executed && !f.runtime->faulted() &&
              f.dsp.clock().hardware_reset_remaining_ticks() == recovery - mcu_period,
          "Trusted native restore did not recover the same real MCU reset-release write");
}

void deferred_cycle_is_not_discarded() {
  constexpr std::array<std::uint8_t, 5> program{0xc6, 0, 0x7b, 2, 0x78};
  auto contract = options(program);
  contract.mcu_hz = mcu_dsp::dsp_hz;
  contract.release = true;
  contract.runtime.writes = McuWriteOrder::after_mcu;
  Fixture f(contract);
  f.mcu.bus().write(u_data, 1);
  f.mcu.bus().write(u_direction, 1);
  f.attach();
  f.on_write = [&](const BusEvent &) { f.runtime->begin_dsp_reset(mcu_dsp::reset_inputs()); };
  require(f.step().instruction_executed, "Actual MCU reset-assert prefix did not execute LDAB");
  constexpr unsigned write_tick = 720;
  constexpr unsigned last_native = 675;
  require(rejects([&] {
            static_cast<void>(f.step());
          }).contains("cannot discard unconsumed peer time") &&
              f.runtime->end_ticks() == write_tick &&
              f.dsp.clock().elapsed_ticks() == last_native && f.runtime->completed_cycles() == 3 &&
              !f.dsp.clock().hardware_reset_active() && f.dsp.cpu().progress().instructions == 0 &&
              f.dsp.cpu().registers().pc == mcu_dsp::boot + 1 && f.runtime->faulted(),
          "A reset observation discarded the genuinely deferred native retirement/tick interval");
}
}

void guards() {
  active_cycle();
  fractional_and_open_boundaries();
  caught_signal_failure();
  deferred_cycle_is_not_discarded();
}
}
