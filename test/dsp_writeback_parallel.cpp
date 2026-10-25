// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_shi.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_writeback_fixture.hpp"
#include "native_access_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <array>
#include <cstdint>
#include <variant>

namespace blofeld::test::writeback {
namespace {
using native_access::fault;
constexpr unsigned address = 7;
constexpr unsigned old_operand = 0x55;
constexpr unsigned old_middle = 0x66;
constexpr std::uint64_t unit = std::uint64_t{1} << word_bits;

void parallel_add() {
  DspMachine machine;
  DspProgram code;
  code.immediate("r0", address);
  code.immediate("m0", address);
  code.immediate("x0", old_operand);
  code.immediate("x1", 2);
  code.immediate("a0", 0);
  code.immediate("a1", 1);
  code.immediate("a2", 0);
  code.immediate("sr", masked_status | carry | overflow | zero | negative);
  const auto body = code.pc();
  code.emit("add x1,a x:(r0)+,x0");
  code.install(machine.bus());
  machine.bus().write({.space = DspSpace::x, .address = address, .value = reference::first});
  checkpoint::run(machine, 8);
  const auto before = machine.cpu().debug_state();
  auto expected = before;
  expected.cpu.r[0] = 0;
  auto progress = await_move(machine, machine.begin_reference_instruction(execution));
  require(move(progress).fields == DspMoveFields{.x = 1} && machine.cpu().debug_state() == expected,
          "The actual read and native ADD complete privately while only AG2 R is visible");
  const auto saved = machine.checkpoint();
  const auto stale = reference::ticket(progress);
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(saved);
    static_cast<void>(fault([&] { static_cast<void>(machine.advance_reference_access(stale)); }));
    const auto e1_ticket = reference::ticket(machine);
    progress = machine.advance_reference_access(e1_ticket);
    expected.cpu.x = 2 * unit + reference::first;
    require(
        machine.cpu().debug_state() == expected,
        "E1 publishes the actual X0 load while A, CCR, PC and counters remain at their old values");
    static_cast<void>(
        fault([&] { static_cast<void>(machine.advance_reference_access(e1_ticket)); }));
    const auto after_e1 = machine.checkpoint();
    DspMachine fork;
    fork.restore(after_e1);
    fork.bus().write({.space = DspSpace::x, .address = address, .value = reference::third});
    progress = retire(fork, {.pending = fork.reference_pending()});
    auto final = expected;
    final.cpu.a = 3 * unit;
    final.cpu.sr = masked_status | unnormalized;
    final.cpu.pc = body + 1;
    require(progress.retired && fork.cpu().state() == final.cpu &&
                fork.cpu().debug_state().native_instructions == before.native_instructions + 1,
            "Later E2 retires independently expected ADD/CCR once without rereading changed data");
    expected.cpu.x = before.cpu.x;
  }
}

void multiply_keeps_original_operands() {
  constexpr unsigned half = 0x400000;
  DspMachine machine;
  DspProgram code;
  code.immediate("r0", address);
  code.immediate("m0", address);
  code.immediate("x0", half);
  code.immediate("y0", half);
  code.immediate("a0", old_operand);
  code.immediate("a1", old_middle);
  code.immediate("a2", 0);
  code.immediate("sr", masked_status | carry);
  code.emit("mpy y0,x0,a x:(r0)+,x0");
  code.install(machine.bus());
  machine.bus().write({.space = DspSpace::x, .address = address, .value = reference::first});
  checkpoint::run(machine, 8);
  const auto before = machine.cpu().debug_state();
  auto progress = await_move(machine, machine.begin_reference_instruction(execution));
  progress = machine.advance_reference_access(reference::ticket(progress));
  require(machine.cpu().state().x == reference::first && machine.cpu().state().a == before.cpu.a &&
              machine.cpu().state().sr == before.cpu.sr,
          "E1 changes the next X operand while the genuine multiplier result remains unpublished");
  constexpr std::uint64_t quarter = std::uint64_t{1} << 45U;
  require(retire(machine, progress).retired && machine.cpu().state().a == quarter,
          "E2 uses the already computed signed fractional half-times-half result, not the new X0");
}

void transfers_and_unchanged_write() {
  struct Input {
    const char *text;
    std::uint64_t result;
  };

  constexpr std::array cases{Input{.text = "move #<$12,x0", .result = 0x112233120000},
                             Input{.text = "move #>$123456,x0", .result = 0x112233123456},
                             Input{.text = "move y0,x0", .result = 0x112233556677},
                             Input{.text = "move x0,x0", .result = seed_x}};
  for (const auto input : cases) {
    DspMachine machine;
    DspProgram code;
    seed(code);
    code.emit(input.text);
    code.install(machine.bus());
    constexpr unsigned seed_steps = 11;
    checkpoint::run(machine, seed_steps);
    const auto before = machine.cpu().debug_state();
    auto progress = await_move(machine, machine.begin_reference_instruction(execution));
    require(move(progress).fields == DspMoveFields{.x = 1} && machine.cpu().debug_state() == before,
            "A genuine transfer emits its destination event even when no bit will change");
    const auto ticket = reference::ticket(progress);
    progress = machine.advance_reference_access(ticket);
    require(
        machine.cpu().state().x == input.result && machine.cpu().state().pc == before.cpu.pc,
        "E1 preserves the literal native immediate/register conversion before later retirement");
    static_cast<void>(fault([&] { static_cast<void>(machine.advance_reference_access(ticket)); }));
    require(retire(machine, progress).retired, "The transfer retires after its distinct E1 ticket");
  }
}

void no_writeback_for_store() {
  DspMachine machine;
  DspProgram code;
  code.immediate("r0", address);
  code.immediate("m0", address);
  code.immediate("x0", reference::first);
  code.immediate("x1", 2);
  code.immediate("a0", 0);
  code.immediate("a1", 1);
  code.immediate("a2", 0);
  code.emit("add x1,a x0,x:(r0)+");
  code.install(machine.bus());
  checkpoint::run(machine, 7);
  auto progress = machine.begin_reference_instruction(execution);
  for (unsigned index = 0; index < 3; ++index) {
    progress = machine.advance_reference_access(reference::ticket(progress));
  }
  require(progress.pending &&
              std::holds_alternative<DspInstructionRetire>(progress.pending->effect) &&
              machine.bus().read({.space = DspSpace::x, .address = address}) == reference::first &&
              machine.cpu().state().a == unit && machine.cpu().state().r[0] == 0,
          "A real parallel store commits without inventing an E1 destination or exposing its ADD "
          "result");
  require(retire(machine, progress).retired && machine.cpu().state().a == 3 * unit,
          "Only later E2 publishes the actual parallel-store ADD result");
}

void overlap_is_not_guessed() {
  DspMachine machine;
  DspProgram code;
  code.immediate("r0", address);
  code.emit("add x1,a x:(r0),a");
  code.install(machine.bus());
  machine.bus().write({.space = DspSpace::x, .address = address, .value = reference::first});
  machine.cpu().step();
  const auto before = machine.cpu().debug_state();
  const auto progress = machine.begin_reference_instruction(execution);
  const auto error = fault(
      [&] { static_cast<void>(machine.advance_reference_access(reference::ticket(progress))); });
  require(
      error.contains("overlapping native ALU/move-writeback accumulator") &&
          machine.cpu().debug_state() == before &&
          machine.bus().read({.space = DspSpace::x, .address = address}) == reference::first,
      "Unknown E1/E2 destination overlap refuses before the actual data read and CPU publication");
}

void control_target_does_not_consume_fifo() {
  constexpr unsigned control = 0xffff91;
  constexpr unsigned configuration = 0xa9;
  DspMachine machine;
  DspProgram code;
  code.immediate("r0", reference::fifo);
  code.emit("move x:(r0),sr");
  code.install(machine.bus());
  machine.cpu().step();
  machine.bus().shi().observe_spi({.mosi = false});
  machine.bus().write({.space = DspSpace::x, .address = control, .value = configuration});
  reference::send(machine.bus().shi(), reference::first);
  const auto before = machine.cpu().debug_state();
  const auto progress = machine.begin_reference_instruction(execution);
  const auto error = fault(
      [&] { static_cast<void>(machine.advance_reference_access(reference::ticket(progress))); });
  require(error.contains("move-writeback control destination") &&
              error.contains("read X:0xffff94") && error.contains("omr=0x000305") &&
              machine.cpu().debug_state() == before &&
              machine.bus().read({.space = DspSpace::x, .address = reference::fifo}) ==
                  reference::first,
          "Unqualified control publication refuses with actual address/context before destructive "
          "input");
}

void computational_control_target_refuses_before_execution() {
  DspMachine machine;
  DspProgram code;
  // This would request compatibility mode if actually executed.
  code.emit("move #>$c02300,sr");
  code.install(machine.bus());
  const auto before = machine.cpu().debug_state();
  const auto progress = machine.begin_reference_instruction(execution);
  const auto error = fault(
      [&] { static_cast<void>(machine.advance_reference_access(reference::ticket(progress))); });
  require(error.contains("move-writeback control destination") && error.contains("pc=0xff0000") &&
              error.contains("omr=0x000305") && machine.cpu().debug_state() == before,
          "An unqualified computational control target rejects after actual opcode admission, "
          "before extension consumption or any changed CPU mode");
}

void load_replaces_prior_address_result() {
  DspMachine machine;
  DspProgram code;
  code.immediate("r0", address);
  code.immediate("m0", address);
  const auto body = code.pc();
  code.emit("move x:(r0)+,r0");
  code.install(machine.bus());
  machine.bus().write({.space = DspSpace::x, .address = address, .value = reference::first});
  checkpoint::run(machine, 2);
  auto progress = await_move(machine, machine.begin_reference_instruction(execution));
  require(move(progress).fields == DspMoveFields{.r = 1} && machine.cpu().state().r[0] == 0,
          "The earlier native wrapped address precedes the later control-load result");
  progress = machine.advance_reference_access(reference::ticket(progress));
  require(machine.cpu().state().r[0] == reference::first && machine.cpu().state().pc == body,
          "E1 overwrites the earlier accepted AG2 R value with the actual loaded R result");
  const auto saved = machine.checkpoint();
  machine.restore(saved);
  const auto retired = retire(machine, {.pending = machine.reference_pending()});
  require(retired.retired && machine.cpu().state().r[0] == reference::first &&
              machine.cpu().state().pc == body + 1,
          "A post-E1 restore preserves the latest R value through its genuine later E2");
}

void stack_source_refuses_before_pop() {
  DspMachine machine;
  DspProgram code;
  code.emit("move ssh,x0");
  code.install(machine.bus());
  const auto before = machine.cpu().debug_state();
  const auto progress = machine.begin_reference_instruction(execution);
  const auto error = fault(
      [&] { static_cast<void>(machine.advance_reference_access(reference::ticket(progress))); });
  require(error.contains("SSH source side effects") && error.contains("pc=0xff0000") &&
              machine.cpu().debug_state() == before,
          "Unqualified source-side stack effects refuse before a native pop or result publication");
}
}

void parallel_checks() {
  parallel_add();
  multiply_keeps_original_operands();
  transfers_and_unchanged_write();
  no_writeback_for_store();
  overlap_is_not_guessed();
  control_target_does_not_consume_fifo();
  computational_control_target_refuses_before_execution();
  load_replaces_prior_address_result();
  stack_source_refuses_before_pop();
}
}
