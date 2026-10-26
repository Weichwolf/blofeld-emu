// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_loop_fetch.hpp"
#include "dsp/dsp_program_fetch.hpp"

#include <array>
#include <cstddef>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace blofeld::test::loop_barrier {
namespace {
constexpr unsigned setup = 0x100;
constexpr unsigned body = setup + 2;
constexpr unsigned omr = 0x305;
constexpr unsigned status = 0xc00300;
constexpr unsigned loop_flag = 0x8000;
constexpr unsigned old_la = 0x321;
constexpr unsigned old_lc = 7;
constexpr unsigned do_two = 0x060280;
constexpr unsigned changed = 0x13579b;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

void refuses(const auto &operation) {
  try {
    operation();
  } catch (const Fault &) { return; }
  throw Fault("Invalid actual fetch barrier unexpectedly succeeded");
}

DspFetchedWord queued(const DspProgramFetch &fetch, const std::size_t index) {
  const auto result = fetch.queued(index);
  if (!result) { throw Fault("Expected actual barrier F2 word is absent"); }
  return *result;
}

void read(DspProgramFetch &fetch, DspBus &bus) {
  fetch.issue();
  fetch.capture(bus);
}

void opening(DspProgramFetch &fetch,
             const DspFetchedInstruction &captured,
             const DspLoopArchitecture before,
             const DspLoopArchitecture after) {
  if (!captured.extension) { throw Fault("Own DO must have an actually captured extension"); }
  require(fetch.consume(captured.opcode.word.address) == captured.opcode.word.value &&
              fetch.consume(captured.extension->word.address) == captured.extension->word.value,
          "Own component opening must consume its actual F2 pair");
  fetch.open_loop({.word = captured.opcode.word,
                   .extension = captured.extension->word,
                   .before = before,
                   .after = after},
                  captured.opcode,
                  *captured.extension);
  require(fetch.confirm_opening(after, false), "Own component control fixture must confirm");
}

void initial_cut() {
  DspBus bus;
  constexpr unsigned leading = 3;
  constexpr unsigned instruction = setup + leading;
  constexpr unsigned child = instruction + 2;
  constexpr unsigned last = child + 2;
  for (unsigned index = 0; index < 16; ++index) {
    bus.write({.address = setup + index, .value = 0});
  }
  bus.write({.address = instruction, .value = do_two});
  bus.write({.address = instruction + 1, .value = last});
  DspProgramFetch fetch;
  fetch.start(setup, omr);
  for (unsigned index = 0; index < dsp_program_fetch_capacity - 1; ++index) { read(fetch, bus); }
  const auto before = fetch.view();
  std::array<DspFetchedWord, leading + 2> retained{};
  for (std::size_t index = 0; index < retained.size(); ++index) {
    retained.at(index) = queued(fetch, index);
  }
  const auto discard = queued(fetch, retained.size());
  const auto captured = fetch.instruction(queued(fetch, leading), 2);
  require(!fetch.loop_last(captured.opcode) && fetch.continuation_after(captured) == child,
          "An actual initial DO pair has no parent-loop scope before native opening");
  fetch.issue();
  require(fetch.view().pending_address.has_value(),
          "Own barrier must encounter a genuine pending F1");
  fetch.hold_after(captured);
  require(
      fetch.view().used == retained.size() && !fetch.view().pending_address &&
          fetch.view().next_address == child &&
          fetch.view().consume_address == before.consume_address &&
          !fetch.occurrence(discard.occurrence),
      "Actual barrier must retain older words, cancel younger F1 and stop exactly after the pair");
  for (std::size_t index = 0; index < retained.size(); ++index) {
    require(queued(fetch, index) == retained.at(index),
            "Barrier changed an actual older F2 sample");
  }
  for (unsigned index = 0; index < leading; ++index) {
    require(fetch.consume(setup + index) == 0, "Actual older NOP samples must remain consumable");
  }
  // Independently declared component controls; no native CPU state is injected.
  opening(fetch,
          captured,
          {.pc = instruction, .omr = omr, .la = old_la, .lc = old_lc, .status = status},
          {.pc = child,
           .omr = omr,
           .la = last,
           .lc = 2,
           .sp = 2,
           .sc = 2,
           .status = status | loop_flag});
  bus.write({.address = child, .value = changed});
  read(fetch, bus);
  require(queued(fetch, 0).word == DspProgramWord{.address = child, .value = changed} &&
              queued(fetch, 0).occurrence > discard.occurrence,
          "Only a new real F2 after the barrier may see changed body code, with a fresh identity");
}

void nested_cut() {
  DspBus bus;
  constexpr unsigned parent_size = 8;
  constexpr unsigned parent_last = body + parent_size - 1;
  constexpr unsigned nested = body + 2;
  constexpr unsigned child = nested + 2;
  for (unsigned index = 0; index < 16; ++index) {
    bus.write({.address = body + index, .value = 0});
  }
  bus.write({.address = setup, .value = do_two});
  bus.write({.address = setup + 1, .value = parent_last});
  bus.write({.address = nested, .value = do_two});
  bus.write({.address = nested + 1, .value = child});
  DspProgramFetch fetch;
  fetch.start(setup, omr);
  read(fetch, bus);
  read(fetch, bus);
  opening(fetch,
          fetch.instruction(queued(fetch, 0), 2),
          {.pc = setup, .omr = omr, .la = old_la, .lc = old_lc, .status = status},
          {.pc = body,
           .omr = omr,
           .la = parent_last,
           .lc = 2,
           .sp = 2,
           .sc = 2,
           .status = status | loop_flag});
  for (unsigned index = 0; index < dsp_program_fetch_capacity; ++index) { read(fetch, bus); }
  const auto discarded_end = queued(fetch, parent_size - 1);
  require(fetch.view().next_address == body,
          "Actual lookahead must already predict the parent backedge");
  const auto captured = fetch.instruction(queued(fetch, 2), 2);
  require(fetch.loop_last(captured.opcode) == parent_last,
          "The actual nested DO opcode must retain its captured parent-loop scope");
  fetch.hold_after(captured);
  require(!fetch.occurrence(discarded_end.occurrence) && fetch.view().next_address == child,
          "Nested barrier must remove the unconfirmed younger parent-end prediction");
  require(fetch.consume(body) == 0 && fetch.consume(body + 1) == 0,
          "Nested barrier must preserve the actual older parent prefix");
  opening(fetch,
          captured,
          {.pc = nested,
           .omr = omr,
           .la = parent_last,
           .lc = 2,
           .sp = 2,
           .sc = 2,
           .status = status | loop_flag},
          {.pc = child,
           .omr = omr,
           .la = child,
           .lc = 2,
           .sp = 4,
           .sc = 4,
           .status = status | loop_flag});
  read(fetch, bus);
  read(fetch, bus);
  require(queued(fetch, 0).word.address == child && queued(fetch, 1).word.address == child &&
              queued(fetch, 0).occurrence > discarded_end.occurrence,
          "Nested opening must forecast two actual child iterations without a stale parent end");
  const auto first_child = fetch.instruction(queued(fetch, 0), 1);
  const auto final_child = fetch.instruction(queued(fetch, 1), 1);
  require(fetch.loop_last(first_child.opcode) == child &&
              fetch.loop_last(final_child.opcode) == child &&
              fetch.continuation_after(first_child) == child &&
              fetch.continuation_after(final_child) == child + 1,
          "Each actual child-end occurrence keeps its own inner LA even after forecast unwind");
  read(fetch, bus);
  const auto parent = fetch.instruction(queued(fetch, 2), 1);
  require(fetch.loop_last(parent.opcode) == parent_last &&
              fetch.continuation_after(parent) == child + 2,
          "Younger actual parent code must carry outer LA while both inner ends remain unretired");
}

void invalid_receipts() {
  DspBus bus;
  bus.write({.address = setup, .value = do_two});
  bus.write({.address = setup + 1, .value = body});
  DspProgramFetch fetch;
  fetch.start(setup, omr);
  read(fetch, bus);
  const auto opcode = queued(fetch, 0);
  const auto before = fetch.view();
  refuses([&] { fetch.hold_after({.opcode = opcode, .extension = std::nullopt}); });
  require(fetch.view() == before, "Missing barrier extension changed the legitimate frontier");
  read(fetch, bus);
  auto captured = fetch.instruction(opcode, 2);
  ++captured.opcode.occurrence;
  const auto full = fetch.view();
  refuses([&] { fetch.hold_after(captured); });
  require(fetch.view() == full, "A forged occurrence changed actual retained F2 samples");
}
}
}

int main() {
  try {
    blofeld::test::loop_barrier::initial_cut();
    blofeld::test::loop_barrier::nested_cut();
    blofeld::test::loop_barrier::invalid_receipts();
    std::cout
        << "Actual F2 Decode-barrier cuts, nested forecast rollback and receipt guards passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
