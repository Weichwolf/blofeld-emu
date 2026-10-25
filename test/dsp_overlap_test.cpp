// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_overlap_fixture.hpp"
#include "dsp_writeback_fixture.hpp"
#include "native_access_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <exception>
#include <iostream>

namespace {
using namespace blofeld;
using namespace blofeld::test;
using namespace blofeld::test::overlap;
using native_access::fault;
constexpr unsigned address = 7;
constexpr unsigned initial_low = 0x55;
constexpr unsigned initial_middle = 0x66;

void setup(DspProgram &code) {
  code.immediate("x0", half);
  code.immediate("x1", 2);
  code.immediate("y0", half);
  code.immediate("a0", initial_low);
  code.immediate("a1", initial_middle);
  code.immediate("a2", 0);
  code.immediate("b0", 0);
  code.immediate("b1", 1);
  code.immediate("b2", 0);
  code.immediate("r0", address);
  code.immediate("m0", address);
  code.immediate("sr", masked_status | carry | overflow | zero);
}

void two_actual_results() {
  DspMachine machine;
  DspProgram code;
  setup(code);
  const auto body = code.pc();
  code.emit("mpy y0,x0,a");
  code.emit("add x1,b x:(r0)+,x0");
  code.install(machine.bus());
  machine.bus().write({.space = DspSpace::x, .address = address, .value = reference::first});
  checkpoint::until(machine, body);
  const auto before = machine.cpu().debug_state();
  auto progress = await_retirement(machine, machine.begin_reference_instruction(execution));
  const auto old_ticket = reference::ticket(progress);
  progress = machine.overlap_reference_instruction(old_ticket);
  require(machine.cpu().debug_state() == before && progress.pending &&
              std::get<DspNativeAccess>(progress.pending->effect).target.address == body + 1 &&
              machine.reference_retirement().has_value(),
          "The actual successor starts while the completed native MPY remains unpublished");
  static_cast<void>(
      fault([&] { static_cast<void>(machine.retire_reference_overlap(old_ticket)); }));
  progress = writeback::await_move(machine, progress);
  auto visible = before;
  visible.cpu.r[0] = 0;
  require(machine.cpu().debug_state() == visible,
          "The younger actual native R update does not publish the older multiplier result");
  progress = machine.advance_reference_access(reference::ticket(progress));
  visible.cpu.x = 2 * unit + reference::first;
  require(machine.cpu().debug_state() == visible,
          "The younger E1 load is visible while both actual arithmetic results remain pending");
  const auto current = reference::ticket(progress);
  const auto previous = previous_ticket(machine);
  static_cast<void>(fault([&] { static_cast<void>(machine.advance_reference_access(current)); }));
  static_cast<void>(
      fault([&] { static_cast<void>(machine.overlap_reference_instruction(current)); }));
  static_cast<void>(fault([&] { static_cast<void>(machine.advance_reference_access(previous)); }));
  require(machine.cpu().debug_state() == visible,
          "Out-of-order retirement, a third candidate and crossed ticket roles refuse purely");
  const auto saved = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    DspMachine fork;
    fork.restore(saved);
    static_cast<void>(fault([&] { static_cast<void>(fork.retire_reference_overlap(previous)); }));
    fork.bus().write({.space = DspSpace::x, .address = address, .value = reference::third});
    const auto next_ticket = reference::ticket(fork);
    const auto retired = fork.retire_reference_overlap(previous_ticket(fork));
    auto expected = visible;
    expected.cpu.pc = body + 1;
    expected.cpu.a = quarter;
    expected.cpu.sr = masked_status | carry | unnormalized;
    ++expected.native_instructions;
    ++expected.native_cycles; // One nominal MPY unit, not a calibrated stage clock.
    const auto actual = fork.cpu().debug_state();
    require(retired.pc == body && actual == expected && !fork.reference_retirement(),
            "Older publication retains every younger accepted R/X field and retires only MPY");
    static_cast<void>(fault([&] { static_cast<void>(fork.retire_reference_overlap(previous)); }));
    const auto after_old = fork.checkpoint();
    require(fork.advance_reference_access(next_ticket).retired,
            "The still-valid younger ticket retires without instruction or bus replay");
    expected.cpu.b = 3 * unit;
    expected.cpu.sr = masked_status | unnormalized;
    expected.cpu.pc = body + 2;
    require(fork.cpu().state() == expected.cpu &&
                fork.cpu().debug_state().native_instructions == before.native_instructions + 2,
            "Both native arithmetic results and literal CCR/PC values retire exactly once");
    DspMachine second;
    second.restore(after_old);
    require(writeback::retire(second, {.pending = second.reference_pending()}).retired &&
                second.cpu().state() == expected.cpu,
            "A fresh post-predecessor fork retains the younger actual completed result");
  }
  machine.cpu().reset_spi_boot();
  const auto reset = machine.cpu().debug_state();
  require(!machine.reference_pending() && !machine.reference_retirement() && reset.cpu.pc == boot &&
              reset.cpu.omr == reset_omr,
          "Actual hardware reset discards both candidate records and restores the reset context");
  static_cast<void>(fault([&] { static_cast<void>(machine.retire_reference_overlap(previous)); }));
  static_cast<void>(fault([&] { static_cast<void>(machine.advance_reference_access(current)); }));
  require(machine.cpu().debug_state() == reset,
          "Neither old ticket can publish a pre-reset result into the freshly reset core");
}

void younger_accumulator_slice_survives() {
  DspMachine machine;
  DspProgram code;
  setup(code);
  const auto body = code.pc();
  code.emit("mpy y0,x0,a");
  code.emit("move x:(r0)+,a1");
  code.install(machine.bus());
  constexpr unsigned normalized_word = 0x600000;
  machine.bus().write({.space = DspSpace::x, .address = address, .value = normalized_word});
  checkpoint::until(machine, body);
  auto progress = await_retirement(machine, machine.begin_reference_instruction(execution));
  progress = machine.overlap_reference_instruction(reference::ticket(progress));
  progress = writeback::await_move(machine, progress);
  progress = machine.advance_reference_access(reference::ticket(progress));
  const auto younger = reference::ticket(progress);
  require(machine.cpu().state().a == normalized_word * unit + initial_low,
          "The younger A1 field publishes while the older actual MPY lower field remains held");
  static_cast<void>(machine.retire_reference_overlap(previous_ticket(machine)));
  require(machine.cpu().state().a == normalized_word * unit &&
              machine.cpu().state().sr == (masked_status | carry | unnormalized),
          "Older publication retains younger A1, supplies its real A0 and preserves older MPY CCR");
  require(
      machine.advance_reference_access(younger).retired &&
          machine.cpu().state().a == normalized_word * unit &&
          machine.cpu().state().sr == (masked_status | carry | unnormalized),
      "The normalized loaded A1 never recomputes the predecessor's unnormalized arithmetic CCR");
}

void dependent_capture_chain() {
  DspMachine machine;
  DspProgram code;
  code.immediate("x1", 1);
  code.immediate("a0", 0);
  code.immediate("a1", 0);
  code.immediate("a2", 0);
  code.immediate("sr", masked_status);
  const auto body = code.pc();
  constexpr unsigned length = 12;
  for (unsigned i = 0; i < length; ++i) { code.emit("add x1,a"); }
  code.install(machine.bus());
  checkpoint::until(machine, body);
  const auto count = machine.cpu().debug_state().native_instructions;
  auto progress = await_retirement(machine, machine.begin_reference_instruction(execution));
  for (unsigned index = 1; index < length; ++index) {
    progress = machine.overlap_reference_instruction(reference::ticket(progress));
    progress = await_retirement(machine, progress);
    require(machine.cpu().state().a == (index - 1) * unit,
            "Functional successor capture never exposes either pending ADD result early");
    static_cast<void>(machine.retire_reference_overlap(previous_ticket(machine)));
    require(machine.cpu().state().a == index * unit &&
                machine.cpu().debug_state().native_instructions == count + index,
            "The older native result publishes once through a sustained two-candidate chain");
  }
  require(writeback::retire(machine, progress).retired &&
              machine.cpu().state().a == length * unit && machine.cpu().state().pc == body + length,
          "The final real dependent ADD result drains without host arithmetic replacement");
}

void refused_contexts() {
  for (const auto contract : {execution, DspReferenceExecution::access_and_retirement_boundaries}) {
    DspMachine machine;
    DspProgram code;
    if (contract == execution) {
      code.word(reference::long_jump);
      code.word(boot);
    } else {
      code.emit("nop");
    }
    code.install(machine.bus());
    auto progress = await_retirement(machine, machine.begin_reference_instruction(contract));
    const auto before = machine.cpu().debug_state();
    const auto error = fault([&] {
      static_cast<void>(machine.overlap_reference_instruction(reference::ticket(progress)));
    });
    require(error.contains("overlap") && machine.cpu().debug_state() == before &&
                !machine.reference_retirement(),
            "Redirects and an unmatched execution contract refuse before ownership changes");
    require(machine.advance_reference_access(reference::ticket(progress)).retired,
            "A pure refused handoff leaves the actual ordinary result available to retire");
  }
}
}

int main() {
  try {
    two_actual_results();
    younger_accumulator_slice_survives();
    dependent_capture_chain();
    refused_contexts();
    overlap::clock_checks();
    std::cout << "Actual two-candidate native overlap and ordered publication checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
