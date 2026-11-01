// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp56300/core.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "mcu_checkpoint_fixture.hpp"
#include "mcu_dsp_power_on_fixture.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <vector>

namespace blofeld::test::power_on {
namespace {
void startup(const unsigned synchronization, const bool held) {
  Fixture f(synchronization, !held);
  const auto zero = f.runtime->checkpoint();
  std::vector<Observation> expected;
  for (unsigned replay = 0; replay < 2; ++replay) {
    f.restore(zero, 1);
    // Two real SYSCLK edges satisfy the DSP's minimum RESET hold. Its own
    // recovery then runs while the MCU still drives its initial reset pulse.
    f.advance_reset(2, McuResetPad::unknown);
    require(f.runtime->end_ticks() == early_release && f.runtime->completed_cycles() == 0 &&
                f.dsp.clock().hardware_reset_ticks() == early_release,
            "Two actual MCU reset edges did not retain the DSP's independent minimum hold");
    if (!held) { f.runtime->release_dsp_reset(); }
    f.advance_reset(drive + synchronization - 2, McuResetPad::unknown);
    const auto cut = f.runtime->checkpoint();
    for (unsigned suffix = 0; suffix < 2; ++suffix) {
      f.restore(cut, 2);
      f.advance_reset(settle, McuResetPad::high);
      f.runtime->end_system_reset();
      const auto edges = drive + synchronization + settle;
      const auto until = (edges * reset_period) + (vector_cycles * bus_period);
      const auto cycles = held ? 0U : edges + (2 * vector_cycles) - 4;
      require(!f.runtime->faulted() && !f.mcu.system_reset_active() && f.slots == "VfPPP" &&
                  f.runtime->end_ticks() == until &&
                  f.mcu.run_clock().bus_cycles() == vector_cycles &&
                  f.mcu.run_clock().reset_sysclk_edges() == edges &&
                  f.mcu.cpu().state().pc == blofeld::test::entry &&
                  f.observed == std::vector<std::uint64_t>{0, reset_period} &&
                  f.dsp.clock().hardware_reset_active() == held &&
                  f.runtime->completed_cycles() == cycles && f.dsp.clock().elapsed_ticks() == until,
              "Actual POR edges/vector slots and restored native owners lost literal clocks or "
              "input order");
      if (held) {
        require(f.trace.empty() && f.dsp.cpu().progress() == dsp56300::Progress{} &&
                    f.dsp.clock().hardware_reset_ticks() == until &&
                    f.dsp.clock().timer_clk2_edges() == 0 &&
                    f.dsp.cpu().registers() == nop_registers(0),
                "Independently held DSP executed during the actual MCU POR/vector sequence");
      } else {
        require(f.dsp.cpu().registers() == nop_registers(cycles) &&
                    f.dsp.cpu().progress() == nop_progress(cycles) &&
                    f.dsp.clock().hardware_reset_ticks() == early_recovery_end &&
                    f.dsp.clock().timer_clk2_edges() == cycles / 2,
                "Real DSP recovery and NOP work left a hidden gap during MCU startup");
      }
      if (replay == 0 && suffix == 0) {
        expected = f.trace;
      } else {
        require(f.trace == expected,
                "Trusted startup replay changed a native DSP effect,fetch address or register");
      }
    }
  }
}

void independent_mcu_reset() {
  Fixture f(3, true);
  constexpr unsigned edges = drive + 3 + settle;
  constexpr unsigned first_end = (edges * reset_period) + (vector_cycles * bus_period);
  constexpr std::uint64_t first_cycles = 201;
  constexpr std::uint64_t first_retirement = 100;
  f.advance_reset(2, McuResetPad::unknown);
  f.runtime->release_dsp_reset();
  f.advance_reset(edges - 2, McuResetPad::high);
  f.runtime->end_system_reset();
  const auto pending = f.dsp.cpu().pending_instruction();
  require(f.runtime->completed_cycles() == first_cycles && f.runtime->end_ticks() == first_end &&
              pending && pending->effect.kind == dsp56300::EffectKind::retire &&
              f.dsp.cpu().registers() == nop_registers(first_cycles) &&
              f.dsp.cpu().progress() == nop_progress(first_cycles),
          "Independent MCU reset did not begin beside an actual pending DSP retirement");
  const auto dsp_epoch = f.dsp.clock().native_epoch();
  f.runtime->begin_system_reset(McuResetSource::external, 3);
  f.attach();
  const auto retained = f.dsp.cpu().pending_instruction();
  if (!pending || !retained) { throw Fault("MCU reset lost the actual pending DSP effect"); }
  require(retained->effect == pending->effect &&
              f.dsp.cpu().progress() == nop_progress(first_cycles) &&
              f.dsp.clock().native_epoch() == dsp_epoch && !f.dsp.clock().hardware_reset_active(),
          "Actual MCU-only reset changed DSP computation or reset its independent clock");
  // A read-only candidate query validates the original opaque effect capability.
  static_cast<void>(f.dsp.candidate_checkpoint(pending->ticket));
  require(f.dsp.cpu().progress() == nop_progress(first_cycles),
          "Validating the retained DSP capability published its pending retirement");
  f.advance_reset(1, McuResetPad::unknown);
  require(
      f.dsp.cpu().progress() == nop_progress(first_cycles + 1) &&
          f.dsp.cpu().progress().instructions == first_retirement &&
          !f.dsp.cpu().pending_instruction(),
      "An already pending DSP retirement did not complete during the first real MCU reset edge");
  const auto snapshot = f.runtime->checkpoint();
  std::vector<Observation> expected;
  for (unsigned suffix = 0; suffix < 2; ++suffix) {
    f.restore(snapshot, 2);
    f.advance_reset(edges - 1, McuResetPad::high);
    f.runtime->end_system_reset();
    constexpr std::uint64_t final_cycles = 406;
    require(f.runtime->completed_cycles() == final_cycles &&
                f.runtime->end_ticks() == std::uint64_t{2} * first_end && f.slots == "VfPPP" &&
                f.mcu.run_clock().bus_cycles() == std::uint64_t{2} * vector_cycles &&
                f.mcu.run_clock().reset_sysclk_edges() == std::uint64_t{2} * edges &&
                f.dsp.cpu().registers() == nop_registers(final_cycles) &&
                f.dsp.cpu().progress() == nop_progress(final_cycles) &&
                f.dsp.clock().native_epoch() == dsp_epoch &&
                f.dsp.clock().hardware_reset_ticks() == early_recovery_end &&
                f.dsp.clock().timer_clk2_edges() == final_cycles / 2,
            "Actual MCU-only reset and restored vector suffix lost continuing DSP clocks or state");
    if (suffix == 0) {
      expected = f.trace;
    } else {
      require(f.trace == expected,
              "MCU-only reset replay changed a continuing DSP effect or fetch");
    }
  }
}
}
}

int main() {
  try {
    using namespace blofeld::test::power_on;
    for (unsigned synchronization = 3; synchronization <= 6; ++synchronization) {
      for (const bool held : {false, true}) { startup(synchronization, held); }
    }
    independent_mcu_reset();
    guards();
    missing_rom();
    std::cout << "Native MCU/DSP POR:8 profiles,32 suffixes,time-zero pads and strict missing-ROM "
                 "controls passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
