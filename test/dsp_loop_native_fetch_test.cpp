// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <cstddef>
#include <exception>
#include <iostream>
#include <variant>

namespace blofeld::test::loop_native {
namespace {
constexpr unsigned old_count = 7;
constexpr unsigned old_last = 0x321;
constexpr unsigned immediate_do = 0x060080;
constexpr unsigned register_do_x0 = 0x06c400;
constexpr unsigned forever_do = 0x000203;
constexpr unsigned loop_flag = 0x8000;
constexpr unsigned trailing_words = 16;
constexpr unsigned guards = 4;
constexpr unsigned preamble_steps = 3 + guards;
constexpr auto execution = DspReferenceExecution::access_and_retirement_boundaries;

DspReferencePending pending(const DspMachine &machine) {
  const auto current = machine.reference_pending();
  if (!current) { throw Fault("Expected actual native-loop admission boundary"); }
  return *current;
}

void fetched(DspMachine &machine) {
  const auto before = machine.reference_fetch();
  const auto cpu = machine.cpu().debug_state();
  const auto first = machine.advance_reference_fetch(DspProgramFetchStage::address);
  require(first.pending_address == before.next_address && first.used == before.used,
          "Actual loop F1 must issue the next selected address without CPU execution");
  const auto second = machine.advance_reference_fetch(DspProgramFetchStage::read);
  require(second.used == before.used + 1 && !second.pending_address &&
              second.words.at(before.used).address == before.next_address &&
              machine.cpu().debug_state() == cpu,
          "Actual loop F2 must capture one P word while the native CPU result remains held");
}

void preamble(DspProgram &program) {
  program.immediate("sr", masked_status);
  program.immediate("la", old_last);
  program.immediate("lc", old_count);
  for (unsigned index = 0; index < guards; ++index) { program.word(0); }
}

struct Case {
  unsigned count;
  unsigned size;
  std::size_t ahead;
  bool restore = false;
  bool register_count = false;
};

DspDebugState run_body(DspMachine &machine, const Case sample, const unsigned body) {
  for (unsigned index = 0; index < sample.count * sample.size; ++index) {
    const auto pc = body + (index % sample.size);
    require(machine.cpu().state().pc == pc,
            "Actual native-loop PC must follow the independent repeated body address");
    const auto before = machine.cpu().debug_state();
    const auto start = machine.begin_reference_instruction(execution);
    require(start.pending.has_value(), "Native NOP must request its actual queued opcode");
    while (machine.reference_fetch().used < sample.ahead) { fetched(machine); }
    auto progress = machine.advance_reference_access(pending(machine).ticket);
    require(progress.pending &&
                std::holds_alternative<DspInstructionRetire>(progress.pending->effect) &&
                !progress.retired && machine.cpu().debug_state() == before,
            "Actual native E1 result must stay held before its retirement ticket");
    const bool end = (index + 1) % sample.size == 0;
    const bool final = index + 1 == sample.count * sample.size;
    unsigned next = pc + 1;
    if (end) { next = final ? body + sample.size : body; }
    if (!progress.pending) { throw Fault("Native loop result lost its held retirement"); }
    const auto retirement = std::get<DspInstructionRetire>(progress.pending->effect);
    require(retirement.pc == pc && retirement.next_pc == next &&
                retirement.redirected == (end && !final),
            "Held native result must independently choose the proper fallthrough or zero-overhead "
            "backedge");
    if (machine.reference_fetch().used == 0) { fetched(machine); }
    progress = machine.advance_reference_access(pending(machine).ticket);
    require(progress.retired && !progress.pending && machine.cpu().state().pc == next,
            "Only actual native retirement may publish the loop-end result");
  }
  return machine.cpu().debug_state();
}

void one(const Case sample) {
  DspProgram program;
  if (sample.register_count) { program.immediate("x0", sample.count); }
  preamble(program);
  const auto setup = program.pc();
  const auto body = setup + 2;
  const auto after = body + sample.size;
  program.word(sample.register_count ? register_do_x0 : (immediate_do | (sample.count << 8)));
  program.word(after - 1);
  for (unsigned index = 0; index < sample.size + trailing_words; ++index) { program.word(0); }
  DspMachine machine;
  program.install(machine.bus());
  checkpoint::run(machine, preamble_steps + static_cast<unsigned>(sample.register_count));
  const auto before = machine.cpu().debug_state();
  require(!sample.register_count || (before.cpu.x & word_mask) == sample.count,
          "The real count-source move must establish the independent X0 operand");
  require(before.cpu.pc == setup && before.cpu.la == old_last && before.cpu.lc == old_count &&
              before.cpu.sp == 0 && before.sc == 0,
          "Own preamble must establish the literal native-loop inputs");
  const auto start = machine.begin_reference_instruction(execution);
  require(start.pending.has_value(), "Actual DO must yield at its native opcode request");
  static_cast<void>(machine.start_reference_fetch(pending(machine).ticket));
  fetched(machine);
  fetched(machine);
  auto progress = machine.advance_reference_access(pending(machine).ticket);
  require(progress.pending && !progress.retired && machine.cpu().debug_state() == before,
          "DO opcode admission must await its actual extension without architectural changes");
  progress = machine.advance_reference_access(pending(machine).ticket);
  require(progress.pending &&
              std::holds_alternative<DspInstructionRetire>(progress.pending->effect) &&
              !progress.retired && machine.cpu().debug_state() == before &&
              machine.reference_fetch().used == 0 && !machine.reference_fetch().pending_address,
          "Actual DO candidate must bind its consumed F2 pair before native publication");
  while (machine.reference_fetch().used < sample.ahead) { fetched(machine); }
  if (sample.count == 0) {
    const auto fetch = machine.reference_fetch();
    for (std::size_t index = 0; index < fetch.used; ++index) {
      require(
          fetch.words.at(index).address >= after,
          "An annulled DO must select real F2 after its body rather than reading skipped words");
    }
  }
  progress = machine.advance_reference_access(pending(machine).ticket);
  require(progress.retired && machine.cpu().state().pc == (sample.count == 0 ? after : body) &&
              machine.cpu().state().lc == (sample.count == 0 ? old_count : sample.count),
          "Real native DO retirement must supply count/PC to the connected frontend");
  const auto saved = machine.checkpoint();
  const auto final = run_body(machine, sample, body);
  auto expected = before;
  expected.cpu.pc = after;
  expected.stack_high.at(1) = old_last;
  expected.stack_low.at(1) = old_count;
  expected.stack_high.at(2) = body;
  expected.stack_low.at(2) = before.cpu.sr;
  expected.native_instructions += 1 + sample.count * sample.size;
  require(final.cpu == expected.cpu && final.stack_high == expected.stack_high &&
              final.stack_low == expected.stack_low && final.sc == expected.sc &&
              final.ep == expected.ep && final.sz == expected.sz &&
              final.native_instructions == expected.native_instructions,
          "Native DO frames/counts/flags and all NOP state must match independent instruction "
          "expectations");
  // Native diagnostic cycle count is not an elapsed-clock oracle.
  if (sample.restore) {
    machine.restore(saved);
    const auto replayed = run_body(machine, sample, body);
    require(replayed == final,
            "Trusted restore must replay real native loop results and retained F2 words");
  }
}

DspInstructionRetire execute_pending(DspMachine &machine, const unsigned words) {
  const auto before = machine.cpu().debug_state();
  while (machine.reference_fetch().used < words) { fetched(machine); }
  for (unsigned index = 0; index < words; ++index) {
    const auto selected = pending(machine);
    const auto *access = std::get_if<DspNativeAccess>(&selected.effect);
    const auto kind = index == 0 ? DspNativeAccessKind::opcode : DspNativeAccessKind::extension;
    require(access != nullptr && access->kind == kind &&
                access->target.address == before.cpu.pc + index,
            "Each native word admission must use its actual ordered P-bus request");
    static_cast<void>(machine.advance_reference_access(selected.ticket));
    require(machine.cpu().debug_state() == before,
            "Native program reads and held computation cannot publish partial CPU state");
  }
  const auto selected = pending(machine);
  const auto *retirement = std::get_if<DspInstructionRetire>(&selected.effect);
  if (retirement == nullptr) { throw Fault("Expected actual held native instruction result"); }
  const auto result = *retirement;
  if (machine.reference_fetch().used == 0) { fetched(machine); }
  const auto progress = machine.advance_reference_access(selected.ticket);
  require(progress.retired && !progress.pending,
          "Native instruction must retire only its actual result");
  return result;
}

DspInstructionRetire execute_next(DspMachine &machine, const unsigned words) {
  static_cast<void>(machine.begin_reference_instruction(execution));
  return execute_pending(machine, words);
}

void start_fetch(DspMachine &machine) {
  static_cast<void>(machine.begin_reference_instruction(execution));
  static_cast<void>(machine.start_reference_fetch(pending(machine).ticket));
}

void forever_loop() {
  DspProgram program;
  preamble(program);
  const auto body = program.pc() + 2;
  program.word(forever_do);
  program.word(body);
  for (unsigned index = 0; index < trailing_words; ++index) { program.word(0); }
  DspMachine machine;
  program.install(machine.bus());
  checkpoint::run(machine, preamble_steps);
  const auto before = machine.cpu().debug_state();
  start_fetch(machine);
  static_cast<void>(execute_pending(machine, 2));
  constexpr unsigned iterations = 8;
  for (unsigned index = 0; index < iterations; ++index) {
    while (machine.reference_fetch().used < dsp_program_fetch_capacity) { fetched(machine); }
    const auto actual = execute_next(machine, 1);
    require(actual.pc == body && actual.next_pc == body && actual.redirected,
            "A real forever-loop backedge remains taken even when source and target PC are equal");
  }
  const auto after = machine.cpu().debug_state();
  constexpr unsigned both_flags = 0x18000;
  require(after.cpu.pc == body && after.cpu.la == body && after.cpu.lc == old_count &&
              after.cpu.sp == 2 && after.sc == 2 && after.cpu.sr == (before.cpu.sr | both_flags) &&
              after.native_instructions == before.native_instructions + 1 + iterations &&
              after.stack_high.at(1) == old_last && after.stack_low.at(1) == old_count &&
              after.stack_high.at(2) == body && after.stack_low.at(2) == before.cpu.sr,
          "Actual DO FOREVER must retain LC, its two physical frames and both loop flags");
}

void nested_loops() {
  DspProgram program;
  preamble(program);
  const auto outer = program.pc();
  const auto child = outer + 2;
  constexpr unsigned child_count = 3;
  constexpr unsigned outer_count = 2;
  constexpr unsigned child_size = 2;
  constexpr unsigned tail = 4;
  const auto child_body = child + 2;
  const auto child_after = child_body + child_size;
  const auto outer_after = child_after + tail;
  program.word(immediate_do | (outer_count << 8));
  program.word(outer_after - 1);
  program.word(immediate_do | (child_count << 8));
  program.word(child_after - 1);
  for (unsigned index = 0; index < child_size + tail + trailing_words; ++index) { program.word(0); }
  DspMachine machine;
  program.install(machine.bus());
  checkpoint::run(machine, preamble_steps);
  const auto before = machine.cpu().debug_state();
  start_fetch(machine);
  static_cast<void>(execute_pending(machine, 2));
  for (unsigned outer_index = 0; outer_index < outer_count; ++outer_index) {
    require(machine.cpu().state().pc == child, "Native outer loop must reach its actual child DO");
    static_cast<void>(execute_next(machine, 2));
    for (unsigned index = 0; index < child_count * child_size; ++index) {
      require(machine.cpu().state().pc == child_body + (index % child_size),
              "Native nested body PC must agree with independent two-by-three iteration semantics");
      static_cast<void>(execute_next(machine, 1));
    }
    for (unsigned index = 0; index < tail; ++index) {
      require(machine.cpu().state().pc == child_after + index,
              "Child native unwind must expose its original outer-loop tail");
      static_cast<void>(execute_next(machine, 1));
    }
  }
  const auto after = machine.cpu().debug_state();
  const auto instructions = 1 + (outer_count * (1 + (child_count * child_size) + tail));
  require(after.cpu.pc == outer_after && after.cpu.lc == old_count && after.cpu.la == old_last &&
              after.cpu.sp == 0 && after.sc == 0 && after.cpu.sr == before.cpu.sr &&
              after.native_instructions == before.native_instructions + instructions &&
              after.stack_high.at(3) == outer_after - 1 && after.stack_low.at(3) == 1 &&
              after.stack_high.at(4) == child_body &&
              after.stack_low.at(4) == (before.cpu.sr | loop_flag),
          "Both actual native frames must unwind while retaining the last independently predicted "
          "inner slots");
}

void two_word_tail() {
  DspProgram program;
  preamble(program);
  const auto body = program.pc() + 2;
  constexpr unsigned count = 3;
  constexpr unsigned immediate_x0 = 0x44f400;
  constexpr unsigned value = 0x55;
  program.word(immediate_do | (count << 8));
  program.word(body + 1);
  program.word(immediate_x0);
  program.word(value);
  for (unsigned index = 0; index < trailing_words; ++index) { program.word(0); }
  DspMachine machine;
  program.install(machine.bus());
  checkpoint::run(machine, preamble_steps);
  const auto before = machine.cpu().debug_state();
  start_fetch(machine);
  static_cast<void>(execute_pending(machine, 2));
  for (unsigned index = 0; index < count; ++index) {
    while (machine.reference_fetch().used < dsp_program_fetch_capacity) { fetched(machine); }
    const auto actual = execute_next(machine, 2);
    require(actual.pc == body && actual.next_pc == (index + 1 == count ? body + 2 : body) &&
                machine.cpu().state().x == value,
            "Each actual two-word load must consume its own extension and confirm the proper "
            "native loop end");
  }
  require(
      machine.cpu().state().pc == body + 2 && machine.cpu().state().sp == 0 &&
          machine.cpu().state().lc == old_count && machine.cpu().state().la == old_last &&
          machine.cpu().debug_state().native_instructions == before.native_instructions + 1 + count,
      "Two-word native tails must restore the original loop context after all three real loads");
}

void forbidden_source() {
  DspProgram program;
  preamble(program);
  const auto setup = program.pc();
  constexpr unsigned do_sp = 0x06fb00;
  program.word(do_sp);
  program.word(setup + 3);
  program.word(0);
  program.word(0);
  DspMachine machine;
  program.install(machine.bus());
  checkpoint::run(machine, preamble_steps);
  const auto before = machine.cpu().debug_state();
  static_cast<void>(machine.begin_reference_instruction(execution));
  static_cast<void>(machine.start_reference_fetch(pending(machine).ticket));
  fetched(machine);
  fetched(machine);
  const auto fetch = machine.reference_fetch();
  const auto diagnostic = checkpoint::rejects(
      [&] { static_cast<void>(machine.advance_reference_access(pending(machine).ticket)); },
      "Unqualified native DO SP source ordering must fail before opcode consumption");
  require(
      diagnostic.contains("Unqualified native program-fetch control") &&
          machine.cpu().debug_state() == before && machine.reference_fetch() == fetch,
      "Known unsupported source must preserve actual native/frontend state and visible context");
}
}
}

int main() {
  try {
    using namespace blofeld::test::loop_native;
    for (const unsigned size : {1U, 2U, 6U}) {
      for (const unsigned count : {0U, 1U, 2U, 3U, 17U}) {
        for (const std::size_t ahead :
             {std::size_t{1}, std::size_t{4}, blofeld::dsp_program_fetch_capacity}) {
          one({.count = count, .size = size, .ahead = ahead, .restore = size == 1 && count == 3});
        }
      }
    }
    for (const unsigned size : {1U, 6U}) {
      for (const unsigned count : {0U, 1U, 3U, 17U}) {
        one({.count = count,
             .size = size,
             .ahead = blofeld::dsp_program_fetch_capacity,
             .restore = count == 3,
             .register_count = true});
      }
    }
    forever_loop();
    nested_loops();
    two_word_tail();
    forbidden_source();
    std::cout << "Native DO/F1/F2 count, nesting, forever, two-word and state checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
