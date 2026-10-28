// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_loop_fetch.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstddef>
#include <deque>
#include <exception>
#include <iostream>
#include <optional>

namespace blofeld::test::loop_fetch {
namespace {
constexpr unsigned control_mask = 0xffff00;
constexpr unsigned loop_flags = 0x18000;
constexpr unsigned immediate_do = 0x060080;
constexpr unsigned forever_do = 0x000203;
constexpr unsigned relative_do = 0x060090;
constexpr unsigned old_count = 7;
constexpr unsigned old_last = 0x321;
constexpr unsigned following_words = 12;
constexpr unsigned guards = 4;
constexpr unsigned preamble_steps = 3 + guards;
constexpr unsigned child_gap = 4;

DspLoopArchitecture control(const DspMachine &machine) {
  const auto cpu = machine.cpu().debug_state();
  return {.pc = cpu.cpu.pc,
          .omr = cpu.cpu.omr,
          .la = cpu.cpu.la,
          .lc = cpu.cpu.lc,
          .sp = cpu.cpu.sp,
          .sc = cpu.sc,
          .status = cpu.cpu.sr & control_mask};
}

void preamble(DspProgram &program) {
  program.immediate("sr", masked_status);
  program.immediate("la", old_last);
  program.immediate("lc", old_count);
  for (unsigned index = 0; index < guards; ++index) { program.word(0); }
}

DspLoopOpening native_open(DspMachine &machine, DspLoopFetch &fetch) {
  const auto before = control(machine);
  const DspProgramWord word{.address = before.pc,
                            .value = machine.bus().read({.address = before.pc})};
  const DspProgramWord extension{.address = before.pc + 1,
                                 .value = machine.bus().read({.address = before.pc + 1})};
  require(!fetch.fetched(word) && !fetch.fetched(extension),
          "A legal DO setup cannot itself complete the parent loop");
  machine.cpu().step();
  const DspLoopOpening opening{
      .word = word, .extension = extension, .before = before, .after = control(machine)};
  fetch.opened(opening);
  return opening;
}

struct PendingWord {
  DspProgramWord word;
  std::optional<DspLoopBoundary> end;
};

struct LoopCase {
  unsigned size;
  unsigned count;
  std::size_t ahead;
};

void one(const LoopCase sample) {
  const auto size = sample.size;
  const auto count = sample.count;
  const auto ahead = sample.ahead;
  DspProgram program;
  preamble(program);
  const auto setup = program.pc();
  const auto body = setup + 2;
  const auto after = body + size;
  program.word(immediate_do | (count << 8));
  program.word(after - 1);
  for (unsigned index = 0; index < size + following_words; ++index) { program.word(0); }
  DspMachine machine;
  program.install(machine.bus());
  checkpoint::run(machine, preamble_steps);
  const auto before = control(machine);
  require(before.pc == setup && before.lc == old_count && before.la == old_last && before.sp == 0 &&
              before.sc == 0 && (before.status & loop_flags) == 0,
          "Own native loop preconditions differ from the literal setup");
  DspLoopFetch fetch(before);
  const auto opening = native_open(machine, fetch);
  require(opening.after.pc == (count == 0 ? after : body) &&
              opening.after.lc == (count == 0 ? old_count : count),
          "Native DO count/PC differs from independent immediate semantics");
  std::deque<PendingWord> queue;
  const unsigned words = count * size;
  for (unsigned index = 0; index < words; ++index) {
    while (queue.size() < ahead && queue.size() < words - index) {
      const auto address = fetch.next_address();
      const DspProgramWord word{.address = address,
                                .value = machine.bus().read({.address = address})};
      queue.push_back({.word = word, .end = fetch.fetched(word)});
    }
    const auto pending = queue.front();
    queue.pop_front();
    const auto expected_pc = body + (index % size);
    require(pending.word.address == expected_pc && pending.word.value == 0 &&
                machine.cpu().state().pc == expected_pc,
            "Actual F2/native PC differs from the independent repeated body address");
    machine.cpu().step();
    if (pending.end) {
      const auto expected_next = index + 1 == words ? after : body;
      require(pending.end->expected.pc == expected_next,
              "Loop prediction differs from independent count exhaustion");
      fetch.confirm(*pending.end, control(machine));
    }
  }
  require(fetch.next_address() == after && fetch.depth() == 0 && fetch.pending() == 0 &&
              machine.cpu().state().pc == after && machine.cpu().state().lc == old_count &&
              machine.cpu().state().la == old_last && machine.cpu().state().sp == 0,
          "Loop end must retain actual native restoration and a sequential real F1 target");
}

void guards_and_snapshots() {
  DspProgram program;
  preamble(program);
  const auto setup = program.pc();
  const auto body = setup + 2;
  program.word(forever_do);
  program.word(body);
  for (unsigned index = 0; index < following_words; ++index) { program.word(0); }
  DspMachine machine;
  program.install(machine.bus());
  checkpoint::run(machine, preamble_steps);
  DspLoopFetch fetch(control(machine));
  static_cast<void>(native_open(machine, fetch));
  const DspProgramWord word{.address = body, .value = 0};
  std::array<DspLoopBoundary, dsp_program_fetch_capacity> ends{};
  for (auto &end : ends) {
    const auto captured = fetch.fetched(word);
    require(captured.has_value(), "Each single-word forever iteration needs its own F2 receipt");
    if (!captured) { throw blofeld::Fault("Loop F2 boundary missing"); }
    end = *captured;
  }
  const auto copied = fetch;
  static_cast<void>(checkpoint::rejects([&] { static_cast<void>(fetch.fetched(word)); },
                                        "Overfull loop event queue must reject before effects"));
  static_cast<void>(checkpoint::rejects([&] { fetch.confirm(ends.back(), ends.back().expected); },
                                        "Equal-PC future iterations cannot confirm an older F2"));
  auto wrong = ends.front().expected;
  ++wrong.lc;
  static_cast<void>(checkpoint::rejects([&] { fetch.confirm(ends.front(), wrong); },
                                        "A differing native LC must not confirm a prediction"));
  require(fetch.pending() == copied.pending() && fetch.next_address() == copied.next_address(),
          "Refused loop confirmation/admission must preserve pending observations");
  auto restored = copied;
  for (const auto &end : ends) {
    machine.cpu().step();
    const auto actual = control(machine);
    require(actual.pc == body && actual.lc == old_count,
            "Actual DO FOREVER must preserve LC and return to its one-word body");
    fetch.confirm(end, actual);
    restored.confirm(end, actual);
  }
  require(fetch.pending() == 0 && restored.pending() == 0 && fetch.depth() == 1 &&
              restored.next_address() == body,
          "Copied pending loop events must replay the same ordered native results");
  static_cast<void>(checkpoint::rejects([&] { fetch.confirm(ends.front(), control(machine)); },
                                        "Consumed loop confirmation must reject a duplicate"));
}

void two_word_tail() {
  DspProgram program;
  preamble(program);
  const auto setup = program.pc();
  const auto body = setup + 2;
  constexpr unsigned count = 3;
  constexpr unsigned immediate_x0 = 0x44f400; // FM Table12-5, also pinned by transfer tests.
  constexpr unsigned literal = 0x55;
  program.word(immediate_do | (count << 8));
  program.word(body + 1);
  program.word(immediate_x0);
  program.word(literal);
  program.word(0);
  DspMachine machine;
  program.install(machine.bus());
  checkpoint::run(machine, preamble_steps);
  DspLoopFetch fetch(control(machine));
  static_cast<void>(native_open(machine, fetch));
  std::array<DspLoopBoundary, count> boundaries{};
  for (auto &boundary : boundaries) {
    require(!fetch.fetched({.address = body, .value = immediate_x0}),
            "The opcode at LA-1 must not complete a two-word tail");
    const auto end = fetch.fetched({.address = body + 1, .value = literal});
    if (!end) { throw blofeld::Fault("Actual last extension must own the loop-end receipt"); }
    boundary = *end;
  }
  for (const auto &boundary : boundaries) {
    machine.cpu().step();
    require(machine.cpu().state().x == literal,
            "Actual native two-word immediate load must run on every loop iteration");
    fetch.confirm(boundary, control(machine));
  }
  require(fetch.next_address() == body + 2 && fetch.depth() == 0 && fetch.pending() == 0 &&
              machine.cpu().state().pc == body + 2,
          "Three two-word loop tails must preserve sequential native exit and all real extensions");
}

void invalid_openings() {
  DspProgram program;
  preamble(program);
  const auto setup = program.pc();
  program.word(immediate_do | (3U << 8));
  program.word(setup + 3);
  program.word(0);
  program.word(0);
  DspMachine machine;
  program.install(machine.bus());
  checkpoint::run(machine, preamble_steps);
  const auto initial = control(machine);
  DspLoopFetch original(initial);
  const auto opening = native_open(machine, original);
  constexpr unsigned cases = 7;
  for (unsigned index = 0; index < cases; ++index) {
    DspLoopFetch fetch(initial);
    static_cast<void>(fetch.fetched(opening.word));
    static_cast<void>(fetch.fetched(opening.extension));
    auto invalid = opening;
    switch (index) {
      case 0: invalid.after.lc = 0; break;
      case 1: ++invalid.after.sp; break;
      case 2: ++invalid.after.sc; break;
      case 3: invalid.after.status ^= loop_flags; break;
      case 4: invalid.after.omr ^= 1U; break;
      case 5: ++invalid.extension.address; break;
      case 6:
        invalid.word.value = relative_do;
        break; // Absolute extension/result do not match DOR.
      default: throw blofeld::Fault("Unknown invalid DO vector");
    }
    const auto diagnostic = checkpoint::rejects([&] { fetch.opened(invalid); },
                                                "Invalid actual DO state/form must fail visibly");
    require(diagnostic.contains("read P:") && diagnostic.contains("pc=") &&
                diagnostic.contains("omr=") && fetch.depth() == 0 && fetch.pending() == 0 &&
                fetch.next_address() == setup + 2,
            "DO refusal must preserve state and identify operation/address/native context");
    fetch.opened(opening);
    require(fetch.depth() == 1, "A rejected DO candidate cannot consume a valid native opening");
  }
}

void nested() {
  DspProgram program;
  preamble(program);
  const auto outer = program.pc();
  const auto child = outer + 2;
  constexpr unsigned child_size = 2;
  const auto child_after = child + 2 + child_size;
  const auto outer_after = child_after + child_gap;
  program.word(immediate_do | (2U << 8));
  program.word(outer_after - 1);
  program.word(immediate_do | (3U << 8));
  program.word(child_after - 1);
  for (unsigned index = 0; index < child_size + child_gap + following_words; ++index) {
    program.word(0);
  }
  DspMachine machine;
  program.install(machine.bus());
  checkpoint::run(machine, preamble_steps);
  DspLoopFetch fetch(control(machine));
  static_cast<void>(native_open(machine, fetch));
  for (unsigned outer_iteration = 0; outer_iteration < 2; ++outer_iteration) {
    require(machine.cpu().state().pc == child, "Outer native loop must reach its actual child DO");
    static_cast<void>(native_open(machine, fetch));
    for (unsigned index = 0; index < child_size * 3 + child_gap; ++index) {
      const auto address = fetch.next_address();
      const DspProgramWord word{.address = address,
                                .value = machine.bus().read({.address = address})};
      const auto end = fetch.fetched(word);
      require(machine.cpu().state().pc == address, "Nested native PC differs from actual F2 order");
      machine.cpu().step();
      if (end) { fetch.confirm(*end, control(machine)); }
    }
  }
  require(fetch.depth() == 0 && fetch.pending() == 0 && fetch.next_address() == outer_after &&
              machine.cpu().state().pc == outer_after && machine.cpu().state().sp == 0 &&
              machine.cpu().state().lc == old_count && machine.cpu().state().la == old_last,
          "Independent two-by-three loops must unwind both actual native frames");
}
}
}

int main() {
  try {
    unsigned vectors = 0;
    for (const unsigned size : {1U, 2U, 6U}) {
      for (const unsigned count : {0U, 1U, 2U, 3U, 17U}) {
        for (const std::size_t ahead :
             {std::size_t{1}, std::size_t{4}, blofeld::dsp_program_fetch_capacity}) {
          blofeld::test::loop_fetch::one({.size = size, .count = count, .ahead = ahead});
          ++vectors;
        }
      }
    }
    blofeld::test::loop_fetch::guards_and_snapshots();
    blofeld::test::loop_fetch::nested();
    blofeld::test::loop_fetch::two_word_tail();
    blofeld::test::loop_fetch::invalid_openings();
    std::cout << vectors
              << " directed native-loop/F2-ahead vectors plus nested/forever/guard/snapshot cases "
                 "passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
