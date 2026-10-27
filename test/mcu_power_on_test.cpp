// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "mcu_checkpoint_fixture.hpp"
#include "storage/flash_image.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using blofeld::test::require;
constexpr unsigned oscillator = 12000000, drive = 128, settle = 64;
constexpr unsigned oscillator_ticks = 5, vector_periods = 5, vector_edges = 10;
constexpr unsigned reset_vector = 0xfffe, por_flag = 0x40, marker = 0x5a;
constexpr unsigned nominal_reset_edges = 195;

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected explicit power-on refusal");
}

McuRunContract contract(const unsigned phase = 0) {
  return {.clock = {.oscillator_hz = oscillator,
                    .initial_oscillator_phase_ticks = phase,
                    .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = McuRunMemory::unstretched_internal};
}

void check_fresh(McuMachine &machine) {
  const auto reset = machine.system_reset_state();
  require(reset && reset->requested == McuResetSource::power_on &&
              reset->selected == McuResetSource::power_on && reset->edges == 0 &&
              reset->phase == McuResetPhase::driving && machine.system_reset_active() &&
              machine.run_clock().elapsed_ticks() == 0 && machine.run_clock().bus_cycles() == 0 &&
              machine.run_clock().oscillator_cycles() == 0 &&
              machine.run_clock().reset_sysclk_edges() == 0 &&
              machine.run_clock().native_epoch() == 1 && !machine.timing_faulted() &&
              machine.bus().read(hc12::crg_flags) == por_flag,
          "Fresh POR is held at time zero, with PORF and no hidden vector periods");
  machine.validate_reset_boundary();
  require(rejects([&] {
            static_cast<void>(machine.bus().read(blofeld::test::ram));
          }).contains("Uninitialized"),
          "Cold volatile SRAM is unknown rather than inherited or fabricated");
}

void test_all_phases_and_replay() {
  const auto nvm = blofeld::test::checkpoint_image();
  for (unsigned sync = 3; sync <= 6; ++sync) {
    for (const unsigned phase : {0U, 4U}) {
      McuMachine machine(nvm, BLOFELD_DEFAULT_SLA, contract(phase), {sync});
      check_fresh(machine);
      const auto start = machine.checkpoint();
      for (unsigned replay = 0; replay < 2; ++replay) {
        machine.restore(start);
        unsigned edges = 0;
        unsigned vector = 0;
        std::uint64_t elapsed = 0;
        std::string slots;
        machine.set_reset_period_observer([&](const McuResetPeriod period) {
          require(period.native_epoch == 1 &&
                      period.duration_ticks ==
                          (edges == 0 ? oscillator_ticks - phase : oscillator_ticks) &&
                      machine.run_clock().elapsed_ticks() == elapsed,
                  "Every qualified initial SYSCLK edge is offered before its actual clock commit");
          ++edges;
          elapsed += period.duration_ticks;
        });
        machine.set_cycle_observer([&](const Hc12BusCycle cycle) {
          require(edges == drive + sync + settle &&
                      machine.system_reset_state()->phase == McuResetPhase::ready,
                  "Native vector periods begin only after literal128+n+64 SYSCLK edges");
          slots += cycle.kind;
          if (cycle.kind == 'V' && cycle.address) { vector = *cycle.address; }
        });
        require(machine.advance_system_reset(drive + sync, McuResetPad::unknown) == drive + sync &&
                    slots.empty() && machine.run_clock().bus_cycles() == 0,
                "Unknown external pad is legal throughout the internal drive phase");
        require(machine.advance_system_reset(settle, McuResetPad::high) == settle &&
                    !machine.system_reset_active() && !machine.timing_faulted() &&
                    slots == "VfPPP" && vector == reset_vector &&
                    machine.cpu().state().pc == blofeld::test::entry &&
                    machine.cpu().state().sp == hc12::reset_sp &&
                    machine.cpu().state().ccr == hc12::reset_ccr &&
                    machine.run_clock().bus_cycles() == vector_periods &&
                    machine.run_clock().reset_sysclk_edges() == edges &&
                    machine.run_clock().oscillator_cycles() == edges + vector_edges &&
                    machine.run_clock().elapsed_ticks() ==
                        (edges + vector_edges) * oscillator_ticks - phase &&
                    machine.bus().read(hc12::crg_flags) == por_flag,
                "POR fetches literalFFFE with five actual native cycles and exact initial phase");
        const auto after = machine.bus().power_off_flash_image();
        require(after.bytes == nvm.bytes && after.known == nvm.known && after.fresh == nvm.fresh,
                "Cold start and replay preserve the complete nonvolatile image and erase history");
        machine.validate_run_boundary();
      }
    }
  }
}

void test_hold_and_unknown_pad() {
  McuMachine machine(blofeld::test::checkpoint_image(), BLOFELD_DEFAULT_SLA, contract(), {3});
  constexpr unsigned sample = nominal_reset_edges;
  constexpr unsigned extra_hold = 7;
  machine.advance_system_reset(sample - 1, McuResetPad::unknown);
  const auto held = machine.checkpoint();
  const auto ticks = machine.run_clock().elapsed_ticks();
  const auto diagnostic = rejects([&] { machine.advance_system_reset(1, McuResetPad::unknown); });
  const auto refused = machine.system_reset_state();
  require(diagnostic.contains("Unknown MCU RESET pad") && refused && machine.timing_faulted() &&
              machine.run_clock().elapsed_ticks() == ticks &&
              machine.run_clock().bus_cycles() == 0 && refused->edges == sample - 1,
          "Missing sample quarantines startup without committing any vector or unqualified edge");
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(held);
    const auto completed = machine.advance_system_reset(extra_hold, McuResetPad::low);
    const auto reset = machine.system_reset_state();
    require(completed == extra_hold && reset && reset->phase == McuResetPhase::external_hold &&
                reset->requested == McuResetSource::power_on &&
                reset->selected == McuResetSource::external &&
                machine.run_clock().bus_cycles() == 0,
            "An external low extends POR without inventing a ready pad or vector");
    require(machine.advance_system_reset(1, McuResetPad::high) == 1 &&
                machine.run_clock().elapsed_ticks() ==
                    static_cast<std::uint64_t>(sample + extra_hold + vector_edges) *
                        oscillator_ticks &&
                machine.cpu().state().pc == blofeld::test::entry &&
                machine.bus().read(hc12::crg_flags) == por_flag,
            "External release fetches FFFE while retaining the original POR cause flag");
  }
}

void test_cold_and_warm_identity() {
  const auto nvm = blofeld::test::checkpoint_image();
  McuMachine machine(nvm, BLOFELD_DEFAULT_SLA, contract(), {3});
  machine.advance_system_reset(nominal_reset_edges, McuResetPad::high);
  machine.bus().write(hc12::crg_flags, por_flag);
  machine.bus().write(blofeld::test::ram, marker);
  require(rejects([&] {
            machine.begin_system_reset(McuResetSource::power_on, 3);
          }).contains("fresh machine") &&
              !machine.system_reset_active() && !machine.timing_faulted(),
          "Power-on cannot be requested as a warm reset that silently retains volatile state");
  machine.begin_system_reset(McuResetSource::external, 3);
  require(machine.bus().read(hc12::crg_flags) == 0 &&
              machine.bus().read(blofeld::test::ram) == marker &&
              machine.run_clock().native_epoch() == 2,
          "Warm reset retains actual SRAM and a software-cleared PORF instead of faking POR");
  machine.advance_system_reset(nominal_reset_edges, McuResetPad::high);
  McuMachine fresh(machine.bus().power_off_flash_image(), BLOFELD_DEFAULT_SLA, contract(), {3});
  check_fresh(fresh);
}

void test_startup_guards() {
  const auto nvm = blofeld::test::checkpoint_image();
  for (const unsigned sync : {0U, 2U, 7U}) {
    require(rejects([&] {
              const McuMachine machine(nvm, BLOFELD_DEFAULT_SLA, contract(), {sync});
            }).contains("explicit3..6"),
            "Initial reset requires explicitly qualified synchronization rather than a default");
  }
  McuMachine machine(nvm, BLOFELD_DEFAULT_SLA, contract(), {3});
  require(rejects([&] { machine.cpu().step(); }).contains("incomplete execution") &&
              rejects([&] { machine.cpu().reset(); }).contains("asserted system reset") &&
              machine.run_clock().elapsed_ticks() == 0 && machine.run_clock().bus_cycles() == 0,
          "Direct CPU entry or reset cannot bypass held initial reset with an early vector");
}
}

int main() {
  try {
    test_all_phases_and_replay();
    test_hold_and_unknown_pad();
    test_cold_and_warm_identity();
    test_startup_guards();
    std::cout
        << "Initial POR: eight phase profiles, eighteen suffixes and cold/warm guards passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
