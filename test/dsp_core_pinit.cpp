// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"
#include "dsp_core_fixture.hpp"

#include <array>
#include <cstdint>
#include <iostream>
#include <string>

namespace blofeld::test::core {
namespace {
constexpr OwnDspResetInputs high_pinit{
    .mode_known = 15, .mode_levels = 5, .pinit = true, .recovery_cycles = 3};
constexpr unsigned period = 45, address = 0x10, cookie = 0x123456;
constexpr unsigned reset_control = 0x04601d, reset_mode = 0x305, reset_status = 0xc00300;
constexpr unsigned boot = 0xff0000;
constexpr std::array<std::uint64_t, 7> cuts{0, 1, 44, 45, 89, 90, 135};

void require(const bool condition, const char *message) {
  if (!condition) { throw Fault(message); }
}

std::string rejected(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("PINIT-high execution/recovery must remain gated");
}

void held(OwnDspMachine &machine, const std::uint64_t ticks) {
  const auto registers = machine.cpu().registers();
  const auto reset = machine.clock().hardware_reset_state();
  require(registers.pc == boot && registers.omr == reset_mode && registers.sr == reset_status &&
              registers.sp == 0 && registers.sc == 0 && registers.vba == 0 &&
              machine.cpu().progress() == dsp56300::Progress{},
          "Known high PINIT captures independent reset register values without an instruction");
  for (const auto modifier : registers.m) {
    require(modifier == dsp56300::word_mask, "Actual reset restores linear address modifiers");
  }
  require(machine.bus().pll().read() == reset_control &&
              machine.bus().read({.space = DspSpace::x, .address = address}) == cookie,
          "PINIT sets PEN in the documented reset PCTL while retained RAM stays unchanged");
  require(reset && reset->phase == DspHardwareResetPhase::asserted &&
              reset->asserted_extal_cycles == ticks / period &&
              reset->phase_ticks == ticks % period && reset->recovery_extal_cycles == 0 &&
              reset->required_recovery_cycles == 3 && machine.clock().elapsed_ticks() == ticks &&
              machine.clock().hardware_reset_ticks() == ticks &&
              machine.clock().native_cycle_units() == 0,
          "High-PINIT assertion accounts only for supplied whole/fractional EXTAL hold time");
}

void release_diagnostic(const std::string &error) {
  require(error.contains("reset-release") && error.contains("PLL recovery") &&
              error.contains("X:0xfffffd") && error.contains("value=0x04601d") &&
              error.contains("pc=0xff0000"),
          "Unqualified PLL-enabled release retains operation, register, value and CPU context");
}

void chip_cuts(const std::uint64_t cut) {
  OwnDspMachine source(dma_test::native_clocks);
  source.bus().write({.space = DspSpace::x, .address = address, .value = cookie});
  source.begin_hardware_reset(high_pinit);
  if (cut != 0) {
    require(source.advance_hardware_reset_ticks(cut) == cut, "Exact high-PINIT reset hold");
  }
  const auto checkpoint = source.checkpoint();
  OwnDspMachine fresh(dma_test::native_clocks);
  fresh.restore(checkpoint);
  for (auto *machine : {&source, &fresh}) {
    held(*machine, cut);
    const auto before = machine->cpu().registers();
    release_diagnostic(rejected([&] { machine->release_hardware_reset(); }));
    static_cast<void>(rejected([&] { static_cast<void>(machine->begin_fetch()); }));
    held(*machine, cut);
    require(machine->cpu().registers() == before && !machine->clock().faulted(),
            "Rejected release/execution preserves all native state and a healthy asserted reset");
    require(machine->advance_hardware_reset_ticks(1) == 1, "Held reset still accepts real time");
    held(*machine, cut + 1);
  }
}

void timeline_cuts(const std::uint64_t cut) {
  OwnDspMachine source(dma_test::native_clocks);
  source.bus().write({.space = DspSpace::x, .address = address, .value = cookie});
  OwnDspTimeline owner(source, OwnDspAutomaticTiming::one_effect_per_cycle, 0, high_pinit);
  if (cut != 0) { require(owner.advance_reset_to(cut) == cut, "Clocked high-PINIT reset hold"); }
  const auto checkpoint = owner.checkpoint();
  OwnDspMachine fresh(dma_test::native_clocks);
  OwnDspTimeline replay(fresh, checkpoint);
  const auto verify = [cut](OwnDspMachine &machine, OwnDspTimeline &timeline) {
    held(machine, cut);
    const auto before = machine.cpu().registers();
    release_diagnostic(rejected([&] { timeline.release_hardware_reset(); }));
    static_cast<void>(rejected([&] { static_cast<void>(timeline.next_deadline()); }));
    held(machine, cut);
    require(machine.cpu().registers() == before && !timeline.faulted(),
            "The timeline cannot bypass PLL recovery validation or alter the held state");
    require(timeline.advance_reset_to(cut + 1) == 1,
            "A rejected timeline release retains its exact valid common-time boundary");
    held(machine, cut + 1);
  };
  verify(source, owner);
  verify(fresh, replay);
}
}

void pinit_checks() {
  for (const auto cut : cuts) {
    chip_cuts(cut);
    timeline_cuts(cut);
  }
  std::cout << "PINIT-high reset snapshots=" << cuts.size() * 2
            << " original/fresh owners=" << cuts.size() * 4 << '\n';
}
}
