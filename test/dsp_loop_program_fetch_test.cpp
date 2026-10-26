// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/dsp_branch_form.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_loop_fetch.hpp"
#include "dsp/dsp_program_fetch.hpp"

#include <array>
#include <cstddef>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace blofeld::test::loop_program {
namespace {
constexpr unsigned setup = 0x100;
constexpr unsigned body = setup + 2;
constexpr unsigned omr = 0x305;
constexpr unsigned status = 0xc00300;
constexpr unsigned loop_status = status | 0x8000;
constexpr unsigned old_lc = 7;
constexpr unsigned old_la = 0x321;
constexpr unsigned immediate_do = 0x060080;
constexpr unsigned jclr_omr3 = 0x0afa03;
constexpr unsigned jset_omr3 = 0x0afa23;
constexpr unsigned unknown_target = 0x400;
constexpr unsigned trailing_words = 16;
constexpr unsigned branch_size = 5;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

std::string refuses(const auto &operation) {
  try {
    operation();
  } catch (const Fault &fault) { return fault.what(); }
  throw Fault("Own negative F1/F2 case unexpectedly succeeded");
}

DspFetchedWord queued(const DspProgramFetch &fetch, const std::size_t index) {
  const auto word = fetch.queued(index);
  if (!word) { throw Fault("Own expected actual F2 occurrence is missing"); }
  return *word;
}

void read(DspProgramFetch &fetch, DspBus &bus) {
  const auto address = fetch.view().next_address;
  fetch.issue();
  require(fetch.view().pending_address == address, "F1 must actually latch the selected address");
  fetch.capture(bus);
}

DspLoopArchitecture initial() {
  return {.pc = setup, .omr = omr, .la = old_la, .lc = old_lc, .status = status};
}

DspLoopArchitecture opened_state(const unsigned count, const unsigned size) {
  if (count == 0) {
    auto result = initial();
    result.pc = body + size;
    return result;
  }
  return {.pc = body,
          .omr = omr,
          .la = body + size - 1,
          .lc = count,
          .sp = 2,
          .sc = 2,
          .status = loop_status};
}

struct LoopCase {
  unsigned count;
  unsigned size;
  std::size_t ahead;
};

void install(DspBus &bus, const LoopCase sample) {
  const auto count = sample.count;
  const auto size = sample.size;
  bus.write({.address = setup, .value = immediate_do | (count << 8)});
  bus.write({.address = setup + 1, .value = body + size - 1});
  for (unsigned index = 0; index < size + trailing_words; ++index) {
    bus.write({.address = body + index, .value = 0});
  }
}

void begin(DspProgramFetch &fetch, DspBus &bus, const unsigned count, const unsigned size) {
  fetch.start(setup, omr);
  read(fetch, bus);
  read(fetch, bus);
  const auto opcode = queued(fetch, 0);
  const auto extension = queued(fetch, 1);
  const DspLoopOpening opening{.word = opcode.word,
                               .extension = extension.word,
                               .before = initial(),
                               .after = opened_state(count, size)};
  static_cast<void>(refuses([&] { fetch.open_loop(opening, opcode, extension); }));
  require(fetch.consume(setup) == opcode.word.value &&
              fetch.consume(setup + 1) == extension.word.value,
          "Native opening must consume the actual opcode and extension");
  auto forged = opcode;
  ++forged.occurrence;
  static_cast<void>(refuses([&] { fetch.open_loop(opening, forged, extension); }));
  fetch.open_loop(opening, opcode, extension);
  auto bad = opening.after;
  ++bad.lc;
  static_cast<void>(refuses([&] { static_cast<void>(fetch.confirm_opening(bad, count == 0)); }));
  require(fetch.confirm_opening(opening.after, count == 0),
          "Actual native opening result must confirm once without a speculative state write");
}

void actual_reads(const LoopCase sample) {
  DspBus bus;
  install(bus, sample);
  DspProgramFetch fetch;
  begin(fetch, bus, sample.count, sample.size);
  const unsigned total = sample.count * sample.size;
  for (unsigned index = 0; index < total; ++index) {
    while (fetch.view().used < sample.ahead) { read(fetch, bus); }
    const auto word = queued(fetch, 0);
    require(word.word.address == body + (index % sample.size) && word.word.value == 0 &&
                fetch.occurrence(word.occurrence) == word.word,
            "Actual F2 FIFO must follow the independent repeated body address");
    require(fetch.consume(word.word.address) == 0 && !fetch.occurrence(word.occurrence),
            "Consumed F2 identity cannot select a younger same-PC word");
    if ((index + 1) % sample.size == 0) {
      const auto completed = (index + 1) / sample.size;
      auto expected = completed == sample.count
                          ? initial()
                          : opened_state(sample.count - completed, sample.size);
      expected.pc = completed == sample.count ? body + sample.size : body;
      if (fetch.view().used == 0) {
        static_cast<void>(refuses(
            [&] { static_cast<void>(fetch.confirm_loop(expected, completed < sample.count)); }));
        read(fetch, bus);
      }
      require(fetch.confirm_loop(expected, completed < sample.count),
              "Oldest consumed loop end must confirm the independently expected native control");
    }
  }
  require(fetch.view().consume_address == body + sample.size,
          "Counted or annulled DO must finish at its actual sequential successor");
}

void branch_rollback(const bool taken) {
  DspBus bus;
  install(bus, {.count = 2, .size = branch_size, .ahead = 0});
  bus.write({.address = body, .value = taken ? jclr_omr3 : jset_omr3});
  bus.write({.address = body + 1, .value = taken ? body + 2 : unknown_target});
  DspProgramFetch fetch;
  begin(fetch, bus, 2, branch_size);
  for (unsigned iteration = 0; iteration < 2; ++iteration) {
    while (fetch.view().used < dsp_program_fetch_capacity) { read(fetch, bus); }
    const auto opcode = queued(fetch, 0);
    const auto extension = queued(fetch, 1);
    const auto discarded = queued(fetch, fetch.view().used - 1);
    const DspBranchDecision decision{.word = opcode.word,
                                     .extension = extension.word,
                                     .omr = omr,
                                     .next_pc = body + 2,
                                     .taken = taken};
    if (iteration == 0) {
      static_cast<void>(refuses([&] { fetch.predict_branch(decision); }));
      require(fetch.view().used == dsp_program_fetch_capacity,
              "Ambiguous equal-PC branch pairs must reject before discarding real F2");
    }
    fetch.predict_branch(decision, opcode.occurrence);
    require(fetch.view().used == 2 && !fetch.occurrence(discarded.occurrence),
            "Discard must remove the wrong-path F2 identities and speculative loop ends");
    read(fetch, bus);
    require(queued(fetch, 2).occurrence > discarded.occurrence,
            "Fresh selected F2 cannot reuse discarded-future identity");
    require(fetch.consume(body) == opcode.word.value &&
                fetch.consume(body + 1) == extension.word.value,
            "Branch confirmation must refer to its actual consumed F2 pair");
    require(fetch.confirm_branch(body, body + 2, omr, taken),
            "Known OMR3 branch must preserve the selected real target");
    for (unsigned address = body + 2; address < body + branch_size; ++address) {
      if (fetch.view().used == 0) { read(fetch, bus); }
      require(queued(fetch, 0).word.address == address && fetch.consume(address) == 0,
              "Selected branch tail must execute its actual queued words");
    }
    if (fetch.view().used == 0) { read(fetch, bus); }
    auto expected = iteration == 0 ? opened_state(1, branch_size) : initial();
    expected.pc = iteration == 0 ? body : body + branch_size;
    require(fetch.confirm_loop(expected, iteration == 0),
            "Branch rewind must not decrement twice or resurrect an already confirmed end");
  }
}

void changed_words_and_copy() {
  DspBus bus;
  install(bus, {.count = 3, .size = 1, .ahead = 0});
  DspProgramFetch fetch;
  begin(fetch, bus, 3, 1);
  constexpr std::array values{0x123456U, 0x654321U, 0x123456U};
  std::array<DspFetchedWord, values.size()> captures{};
  for (std::size_t index = 0; index < values.size(); ++index) {
    bus.write({.address = body, .value = values.at(index)});
    read(fetch, bus);
    captures.at(index) = queued(fetch, index);
  }
  for (std::size_t index = 0; index < values.size(); ++index) {
    require(captures.at(index).word.address == body &&
                captures.at(index).word.value == values.at(index) &&
                fetch.occurrence(captures.at(index).occurrence) == captures.at(index).word,
            "Same-PC F2 occurrences must preserve the actual value sampled on each read");
    if (index != 0) {
      require(captures.at(index).occurrence > captures.at(index - 1).occurrence,
              "Equal words still require distinct actual F2 identity");
    }
  }
  auto copy = fetch;
  read(fetch, bus);
  read(copy, bus);
  require(queued(fetch, values.size()).occurrence != queued(copy, values.size()).occurrence,
          "Copied continuations cannot alias subsequently captured F2 identities");
}

void unknown_read() {
  DspBus bus;
  DspProgramFetch fetch;
  fetch.start(unknown_target, omr);
  fetch.issue();
  const auto before = fetch.view();
  const auto diagnostic = refuses([&] { fetch.capture(bus); });
  require(
      fetch.view() == before && diagnostic.contains("read P:0x000400") &&
          diagnostic.contains("pc=0x000400") && diagnostic.contains("omr=0x000305"),
      "Unknown actual P read must fail with operation/address/native context before F2 insertion");
}
}
}

int main() {
  try {
    using namespace blofeld::test::loop_program;
    for (const unsigned size : {1U, 2U, 6U}) {
      for (const unsigned count : {0U, 1U, 2U, 3U, 17U}) {
        for (const std::size_t ahead :
             {std::size_t{1}, std::size_t{4}, blofeld::dsp_program_fetch_capacity}) {
          actual_reads({.count = count, .size = size, .ahead = ahead});
        }
      }
    }
    branch_rollback(false);
    branch_rollback(true);
    changed_words_and_copy();
    unknown_read();
    std::cout << "45 actual F1/F2 loop cases plus branch-discard, changed-word, copy and "
                 "unknown-read cases passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
