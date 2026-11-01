// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "mcu_checkpoint_fixture.hpp"
#include "mcu_dsp_power_on_fixture.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace blofeld::test::power_on {
namespace {
std::string
refused_rom(Fixture &f, const dsp56300::Registers &before, const std::uint64_t recovery_end) {
  try {
    f.step();
  } catch (const Fault &error) {
    const std::string message(error.what());
    require(message.contains("Unknown DSP program sample read P:0xff0000") &&
                message.contains("pc=0xff0000") && message.contains("omr=0x000305") &&
                f.runtime->faulted() && f.runtime->completed_cycles() == 2 &&
                f.dsp.clock().hardware_reset_ticks() == recovery_end &&
                f.dsp.clock().elapsed_ticks() == recovery_end + (std::uint64_t{2} * reset_period) &&
                f.dsp.cpu().registers() == before && f.dsp.cpu().progress() == dsp56300::Progress{},
            "Missing ROM lost its real captured-word failure,CPU state or admitted clock prefix");
    return message;
  }
  throw Fault("Missing ROM was replaced by a successful boot or fabricated NOP");
}
}

void guards() {
  McuMachine mcu(checkpoint_image(), BLOFELD_DEFAULT_SLA, contract(), {3});
  OwnDspMachine dsp(clocks);
  const auto before = dsp.cpu().registers();
  auto unqualified = inputs();
  unqualified.reset_dsp_first = false;
  bool rejected = false;
  try {
    const McuDspRuntime invalid(unqualified, mcu, dsp);
  } catch (const Fault &error) {
    rejected = std::string(error.what()).contains("Unqualified MCU/DSP initial reset ordering");
  }
  require(rejected && mcu.run_clock().elapsed_ticks() == 0 && mcu.run_clock().bus_cycles() == 0 &&
              dsp.clock().elapsed_ticks() == 0 && dsp.cpu().registers() == before &&
              !dsp.clock().hardware_reset_active(),
          "An unqualified initial reset order acquired DSP ownership or consumed native MCU time");
}

void missing_rom() {
  Fixture f(3, false);
  constexpr unsigned edges = drive + 3 + settle;
  constexpr unsigned startup_end = (edges * reset_period) + (vector_cycles * bus_period);
  constexpr std::uint64_t fresh_periods = 8;
  const std::array<std::uint32_t, fresh_periods> authored_nops{};
  f.advance_reset(edges, McuResetPad::high);
  f.runtime->end_system_reset();
  f.recovery_end = startup_end + bus_period;
  f.runtime->release_dsp_reset();
  f.step();
  const auto before = f.dsp.cpu().registers();
  const auto fresh = f.runtime->checkpoint();
  require(!f.dsp.clock().hardware_reset_active() && f.runtime->completed_cycles() == 0 &&
              f.dsp.clock().hardware_reset_ticks() == f.recovery_end && before == nop_registers(0),
          "One actual MCU NOP did not supply exactly two real DSP recovery clocks");
  f.step();
  require(f.runtime->completed_cycles() == 2 && f.trace.size() == 2 &&
              f.trace.front().issued == boot && f.trace.back().captured &&
              f.dsp.cpu().registers() == before && f.dsp.cpu().progress() == dsp56300::Progress{},
          "Real F1/F2 did not capture the missing ROM without native execution");
  const auto absent = f.runtime->checkpoint();
  std::string expected;
  for (const bool install : {false, true}) {
    f.restore(absent, 2);
    if (install) {
      f.dsp.bus().install_rom({.space = DspSpace::p, .start = boot, .words = authored_nops});
    }
    const auto failure = refused_rom(f, before, f.recovery_end);
    if (!install) {
      expected = failure;
    } else {
      require(failure == expected,
              "Installing ROM after F2 rewrote the already captured unknown program word");
    }
  }
  f.restore(fresh, 2);
  f.dsp.bus().install_rom({.space = DspSpace::p, .start = boot, .words = authored_nops});
  for (unsigned i = 0; i < fresh_periods / 2; ++i) { f.step(); }
  require(!f.runtime->faulted() && f.runtime->completed_cycles() == fresh_periods &&
              f.dsp.clock().elapsed_ticks() == f.recovery_end + (fresh_periods * reset_period) &&
              f.dsp.cpu().registers() == nop_registers(fresh_periods) &&
              f.dsp.cpu().progress() == nop_progress(fresh_periods),
          "Trusted pre-F1 restore did not fetch authored ROM and retire three independently "
          "expected NOPs");
}
}
