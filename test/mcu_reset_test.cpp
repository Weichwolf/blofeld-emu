// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_cop.hpp"
#include "mcu/mcu_flash.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "mcu_checkpoint_fixture.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <string>

namespace {
using namespace blofeld;
using blofeld::test::require;
constexpr unsigned oscillator_hz = 12000000, common_hz = 540000000;
constexpr unsigned minimum_sync = 3, maximum_sync = 6, drive = 128, settle = 64;
constexpr unsigned nominal_reset_edges = 195, vector_bus_cycles = 5, vector_osc_edges = 10;
constexpr std::uint64_t period_ticks = 5;
constexpr unsigned initial_ticks = 50, external_release_ticks = 1105;
constexpr unsigned cop_vector = 0xfffa, external_vector = 0xfffe, rti_control = 0x12,
                   flash_bus_hz = 6000000;
constexpr unsigned target = 0x8234, target_offset = 0x234, sector_offset = 0x200,
                   sector_bytes = 512;
constexpr unsigned program = 0x20, erase = 0x40, flash_status = 0x105, flash_command = 0x106;
constexpr unsigned launched = 0x80, idle_status = 0xc0, flash_divider = 0x48;
constexpr unsigned data = 0x1234, erased = 0xff, protection = 0x7f0d, security = 0x7f0f;
constexpr unsigned ram_value = 0x66, relocated_registers = 0x40, pll_select = 0x80;
constexpr unsigned timeout_edges = 16384, timeout_ticks = 81920, timeout_bus_cycles = 8196;
constexpr std::array<std::uint8_t, 11> kick_code{
    0x86, 1, 0x7a, 0, 0x3c, 0x86, 0xf0, 0x7a, 0, 0x3f, 0xa7}; // Own native caller.

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected strict system-reset rejection");
}

McuRunContract contract() {
  return {.clock = {.oscillator_hz = oscillator_hz,
                    .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = McuRunMemory::internal_flash_reads};
}

McuFlashImage image(const bool kick = false) {
  auto result = blofeld::test::checkpoint_image(kick ? std::span(kick_code)
                                                     : std::span<const std::uint8_t>{});
  result.bytes[hc12::cop_reset_vector - blofeld::test::entry] = blofeld::test::handler >> 8U;
  result.bytes[hc12::cop_reset_vector - blofeld::test::entry + 1] = 0;
  result.bytes[target_offset] = erased;
  result.bytes[target_offset + 1] = erased;
  result.fresh.set(target_offset / 2);
  return result;
}

void request(McuMachine &machine) {
  blofeld::test::drive_interrupts(machine.bus());
  for (unsigned n = 0; n < 3; ++n) { machine.cpu().step(); }
  require(rejects([&] { machine.cpu().step(); }).contains("invalid ARMCOP service") &&
              machine.bus().cop_state().reset == McuCopReset::invalid_service,
          "Own native instructions request the actual COP reset before reset sequencing");
}

void test_sequence_boundaries() {
  for (const auto source : {McuResetSource::external, McuResetSource::cop}) {
    for (unsigned sync = minimum_sync; sync <= maximum_sync; ++sync) {
      McuResetSequence sequence(source, sync);
      const auto release = drive + sync;
      for (unsigned edge = 1; edge <= release + settle; ++edge) {
        require(sequence.active() && sequence.drives_low() == (edge <= release),
                "Reset is driven for exactly128+n edges and internally held through settling");
        sequence.edge(edge == release + settle ? McuResetPad::high : McuResetPad::unknown);
        if (edge < release + settle) {
          require(sequence.state().phase != McuResetPhase::ready,
                  "No early vector selection during either reset phase");
        }
      }
      require(sequence.state().edges == release + settle &&
                  sequence.state().phase == McuResetPhase::ready &&
                  sequence.vector() ==
                      (source == McuResetSource::cop ? cop_vector : external_vector),
              "Literal CRG bounds choose the independent COP/external vector addresses");
      sequence.complete();
      require(!sequence.active(), "Native vector completion ends the reset episode");
      static_cast<void>(rejects([&] { sequence.edge(McuResetPad::high); }));
    }
  }
  for (const unsigned sync : {0U, 1U, 2U, 7U, erased}) {
    static_cast<void>(rejects([&] { const McuResetSequence sequence(McuResetSource::cop, sync); }));
  }
  McuResetSequence unknown(McuResetSource::cop, minimum_sync);
  for (unsigned n = 0; n < nominal_reset_edges - 1; ++n) { unknown.edge(McuResetPad::unknown); }
  const auto before = unknown.state();
  require(rejects([&] { unknown.edge(McuResetPad::unknown); }).contains("Unknown MCU RESET pad") &&
              unknown.state() == before,
          "Unknown source-sampling pads reject without committing the unqualified edge");
  auto copied = unknown;
  copied.edge(McuResetPad::low);
  for (unsigned n = 0; n < 5; ++n) { copied.edge(McuResetPad::low); }
  copied.edge(McuResetPad::high);
  require(copied.vector() == external_vector && unknown.state() == before,
          "External low overrides a COP request; release selects external reset, copies stay "
          "independent");
}

void test_owned_cop_reset_and_replay() {
  const auto nvm = image(true);
  McuMachine machine(nvm, BLOFELD_DEFAULT_SLA, contract());
  request(machine);
  const auto cycles = machine.run_clock().bus_cycles();
  const auto ticks = machine.run_clock().elapsed_ticks();
  const auto osc = machine.run_clock().oscillator_cycles();
  machine.begin_system_reset(McuResetSource::cop, minimum_sync);
  require(machine.system_reset_active() && !machine.timing_faulted() &&
              machine.run_clock().native_epoch() == 2 && machine.bus().cop_state() == McuCopState{},
          "The actual pending cause starts a new held native epoch and resets the COP module");
  require(
      rejects([&] { machine.cpu().step(); }).contains("incomplete execution") &&
          rejects([&] { machine.cpu().reset(); }).contains("asserted system reset") &&
          machine.run_clock().elapsed_ticks() == ticks &&
          machine.run_clock().bus_cycles() == cycles,
      "The uninitialized held CPU cannot execute or bypass the asserted reset with a vector fetch");
  require(machine.advance_system_reset(drive + minimum_sync, McuResetPad::unknown) ==
              drive + minimum_sync,
          "The complete drive phase needs no fabricated external pad value");
  const auto held = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(held);
    std::string slots;
    unsigned vector = 0;
    machine.set_cycle_observer([&](const Hc12BusCycle cycle) {
      slots += cycle.kind;
      if (cycle.kind == 'V' && cycle.address) { vector = *cycle.address; }
    });
    require(
        machine.advance_system_reset(settle, McuResetPad::high) == settle &&
            !machine.system_reset_active() && !machine.timing_faulted() && slots == "VfPPP" &&
            vector == cop_vector && machine.cpu().state().pc == blofeld::test::handler &&
            machine.cpu().state().sp == hc12::reset_sp &&
            machine.cpu().state().ccr == hc12::reset_ccr && !machine.cpu().waiting() &&
            machine.run_clock().bus_cycles() == cycles + vector_bus_cycles &&
            machine.run_clock().oscillator_cycles() ==
                osc + nominal_reset_edges + vector_osc_edges &&
            machine.run_clock().elapsed_ticks() ==
                ticks + (nominal_reset_edges + vector_osc_edges) * period_ticks,
        "Trusted mid-reset replay fetches COPfffa in five native cycles, with exact lifetime time");
    const auto after = machine.bus().power_off_flash_image();
    require(after.bytes == nvm.bytes && after.known == nvm.known && after.fresh == nvm.fresh,
            "COP reset retains the entire NVM and erase history");
    machine.validate_run_boundary();
  }
}

void test_external_hold_modules_and_unknown_pad() {
  McuMachine machine(image(), BLOFELD_DEFAULT_SLA, contract());
  require(machine.run_clock().elapsed_ticks() == initial_ticks,
          "Initial native reset consumes five reference cycles");
  machine.bus().write(blofeld::test::ram, ram_value);
  machine.bus().write(hc12::usb_control, erase);
  machine.bus().observe_phy({.edges = 15});
  machine.bus().write(hc12::clock_select, pll_select);
  machine.bus().write(hc12::rti_control, rti_control);
  machine.bus().write(hc12::initrg, relocated_registers);
  machine.bus().retire_instruction();
  machine.begin_system_reset(McuResetSource::external, minimum_sync);
  require(machine.bus().read(hc12::initrg) == 0 && machine.bus().read(hc12::rti_control) == 0 &&
              machine.bus().read(hc12::clock_select) == 0 &&
              machine.bus().read(hc12::sci_control2) == 0 &&
              machine.bus().read(blofeld::test::ram) == ram_value,
          "System reset restores mapping/clock/SCI defaults while retaining known SRAM");
  machine.advance_system_reset(nominal_reset_edges - 1, McuResetPad::unknown);
  const auto before_sample = machine.checkpoint();
  const auto ticks = machine.run_clock().elapsed_ticks();
  require(rejects([&] {
            machine.advance_system_reset(1, McuResetPad::unknown);
          }).contains("Unknown MCU RESET pad") &&
              machine.timing_faulted() && machine.run_clock().elapsed_ticks() == ticks,
          "Unknown reset arbitration is visible, faults the owner and consumes no guessed edge");
  machine.restore(before_sample);
  machine.advance_system_reset(1, McuResetPad::low);
  const auto held_state = machine.system_reset_state();
  require(held_state && held_state->phase == McuResetPhase::external_hold,
          "An externally held low pad keeps the CPU reset asserted beyond the fixed sequence");
  const auto held = machine.checkpoint();
  machine.advance_system_reset(5, McuResetPad::low);
  machine.advance_system_reset(1, McuResetPad::high);
  require(
      machine.cpu().state().pc == blofeld::test::entry &&
          machine.run_clock().elapsed_ticks() == external_release_ticks &&
          machine.run_clock().bus_cycles() == 2ULL * vector_bus_cycles,
      "Known external release fetchesfffe after the literal extra hold, using OSC rather than PHY");
  machine.restore(held);
  machine.advance_system_reset(6, McuResetPad::high);
  const auto released = machine.system_reset_state();
  require(released && released->edges == nominal_reset_edges + 1,
          "Reset advancement stops on the first qualified release instead of consuming the whole "
          "offer");
}

void test_timeout_prefix_and_repeat_reset() {
  constexpr std::array<std::uint8_t, 2> code{0x3e, 0xa7}; // Own WAI;NOP.
  auto nvm = image();
  nvm.bytes[0] = code[0];
  McuMachine machine(nvm, BLOFELD_DEFAULT_SLA, contract());
  blofeld::test::drive_interrupts(machine.bus());
  machine.bus().write(hc12::cop_control, 1);
  const auto before = machine.run_clock().elapsed_ticks();
  machine.cpu().set_state(
      {.pc = blofeld::test::entry, .sp = blofeld::test::stack, .ccr = hc12::reset_ccr});
  machine.cpu().step();
  require(rejects([&] {
            machine.advance_wait(timeout_edges);
          }).contains("COP system reset (timeout)") &&
              machine.run_clock().bus_cycles() == timeout_bus_cycles &&
              machine.run_clock().elapsed_ticks() == before + timeout_ticks,
          "Timeout preserves the actual first-event time, including the uncompleted IPBus period");
  const auto ticks = machine.run_clock().elapsed_ticks();
  machine.begin_system_reset(McuResetSource::cop, minimum_sync);
  machine.advance_system_reset(nominal_reset_edges, McuResetPad::high);
  require(machine.run_clock().elapsed_ticks() ==
                  ticks + (nominal_reset_edges + vector_osc_edges) * period_ticks &&
              machine.cpu().state().pc == blofeld::test::handler,
          "A timed-out WAI restarts through COP vector without losing the partial-cycle edge");
  machine.begin_system_reset(McuResetSource::external, maximum_sync);
  machine.advance_system_reset(drive + maximum_sync + settle, McuResetPad::high);
  require(machine.run_clock().native_epoch() == 3 &&
              machine.cpu().state().pc == blofeld::test::entry &&
              machine.run_clock().reset_sysclk_edges() ==
                  2ULL * nominal_reset_edges + maximum_sync - minimum_sync,
          "Repeated source-specific resets preserve lifetime clocks and advance native epochs");
}

enum class FlashPhase : std::uint8_t { staged, selected, buffered, active, complete };

McuFlashImage flash_expected(const unsigned command, const FlashPhase phase) {
  auto expected = image();
  const auto begin = command == program ? target_offset : sector_offset;
  const auto size = command == program ? 2 : sector_bytes;
  if (phase == FlashPhase::active) {
    expected.invalidate({.offset = static_cast<std::uint16_t>(begin), .size = size});
  } else if (phase == FlashPhase::complete && command == program) {
    expected.bytes[target_offset] = data >> 8U;
    expected.bytes[target_offset + 1] = data & erased;
    expected.fresh.reset(target_offset / 2);
  } else if (phase == FlashPhase::complete) {
    for (unsigned n = begin; n < begin + size; ++n) {
      expected.bytes.at(n) = erased;
      expected.fresh.set(n / 2);
    }
  }
  return expected;
}

void flash_prepare(McuBus &bus, const unsigned command, const FlashPhase phase) {
  bus.observe_flash_clock({.oscillator_hz = oscillator_hz, .bus_hz = flash_bus_hz});
  bus.write(hc12::flash_clock_divider, flash_divider);
  bus.write_memory({.address = target, .size = 2, .value = data});
  if (phase >= FlashPhase::selected) {
    bus.write(flash_command, static_cast<std::uint8_t>(command));
  }
  if (phase >= FlashPhase::buffered) { bus.write(flash_status, launched); }
  if (phase >= FlashPhase::active) { bus.advance_flash(FlashMilestone::transfer_buffer); }
  if (phase >= FlashPhase::complete) { bus.advance_flash(FlashMilestone::complete_command); }
}

void test_flash_interruption() {
  for (const auto command : {program, erase}) {
    for (const auto phase : {FlashPhase::staged,
                             FlashPhase::selected,
                             FlashPhase::buffered,
                             FlashPhase::active,
                             FlashPhase::complete}) {
      const auto expected = flash_expected(command, phase);
      McuMachine machine(image(), BLOFELD_DEFAULT_SLA, contract());
      flash_prepare(machine.bus(), command, phase);
      machine.begin_system_reset(McuResetSource::external, minimum_sync);
      machine.advance_system_reset(nominal_reset_edges, McuResetPad::high);
      const auto actual = machine.bus().power_off_flash_image();
      require(actual.bytes == expected.bytes && actual.known == expected.known &&
                  actual.fresh == expected.fresh &&
                  machine.bus().read(hc12::flash_clock_divider) == 0 &&
                  machine.bus().read(flash_status) == idle_status &&
                  machine.bus().read(hc12::flash_security) == expected.bytes[security] &&
                  machine.bus().read(hc12::flash_protection) == expected.bytes[protection],
              "All five program/erase phases retain exact NVM/knowledge/history and reload "
              "security/protection");
    }
  }
}

void test_admission_and_unknown_vector() {
  McuMachine healthy(image(true), BLOFELD_DEFAULT_SLA, contract());
  const auto ticks = healthy.run_clock().elapsed_ticks();
  require(rejects([&] {
            healthy.begin_system_reset(McuResetSource::cop, minimum_sync);
          }).contains("pending watchdog request") &&
              rejects([&] {
                healthy.begin_system_reset(McuResetSource::external, 0);
              }).contains("3..6") &&
              healthy.run_clock().native_epoch() == 1 &&
              healthy.run_clock().elapsed_ticks() == ticks && !healthy.system_reset_active(),
          "Unqualified reset source/timing is rejected before replacing any native state");
  auto nvm = image(true);
  nvm.invalidate({.offset = cop_vector - blofeld::test::entry, .size = 2});
  McuMachine unknown(nvm, BLOFELD_DEFAULT_SLA, contract());
  request(unknown);
  unknown.begin_system_reset(McuResetSource::cop, minimum_sync);
  unknown.advance_system_reset(drive + minimum_sync, McuResetPad::unknown);
  const auto held = unknown.checkpoint();
  require(
      rejects([&] {
        unknown.advance_system_reset(settle, McuResetPad::high);
      }).contains("0xfffa") &&
          unknown.timing_faulted() && unknown.system_reset_active(),
      "An unknown COP vector faults during the real native vector read without completing reset");
  unknown.restore(held);
  unknown.advance_system_reset(settle, McuResetPad::low);
  unknown.advance_system_reset(1, McuResetPad::high);
  require(!unknown.system_reset_active() && unknown.cpu().state().pc == blofeld::test::entry &&
              !unknown.bus().power_off_flash_image().known.test(cop_vector - blofeld::test::entry),
          "External-low arbitration fetches the independent known external vector, without "
          "repairing COP NVM");
}

void test_stale_joint_epoch() {
  McuMachine mcu(image(), BLOFELD_DEFAULT_SLA, contract());
  DspMachine dsp(
      DspRunContract{.clock = {.extal_hz = oscillator_hz,
                               .timebase_hz = common_hz,
                               .switch_timing = DspClockSwitch::reference_instruction_boundary},
                     .timing = DspRunTiming::native_internal_estimates});
  McuDspSchedule schedule(
      {.dsp = {.observation = DspStepObservation::reference_completion_boundary},
       .maximum_dsp_steps_per_cycle = 16},
      mcu,
      dsp);
  const auto ticks = dsp.run_clock().elapsed_ticks();
  mcu.begin_system_reset(McuResetSource::external, minimum_sync);
  require(schedule.faulted() &&
              rejects([&] { schedule.end_instruction(); }).contains("healthy inactive") &&
              dsp.run_clock().elapsed_ticks() == ticks,
          "Old joint scheduling rejects the changed MCU epoch before advancing an independent DSP");
}
}

int main() {
  try {
    test_sequence_boundaries();
    test_owned_cop_reset_and_replay();
    test_external_hold_modules_and_unknown_pad();
    test_timeout_prefix_and_repeat_reset();
    test_flash_interruption();
    test_admission_and_unknown_vector();
    test_stale_joint_epoch();
    std::cout << "CRG reset pulse/arbitration, native vectors, owned time/checkpoints and NVM "
                 "interruption verified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
