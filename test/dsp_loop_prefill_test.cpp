// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_loop_fetch.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>

namespace {
using namespace blofeld;
using blofeld::test::esai::rejects;
using blofeld::test::esai::require;
constexpr unsigned setup = 0x100, body = setup + 2, outer_last = 0x444, outer_count = 7;
constexpr unsigned masked = 0x300, loop_flag = 0x8000, words = 32;
constexpr unsigned dor_immediate = 0x060090;

DspFetchedWord known_word(const DspProgramFetch &fetch, const std::size_t index) {
  const auto word = fetch.queued(index);
  if (!word) { throw Fault("Expected an actual known F2 occurrence"); }
  return *word;
}

struct Setup {
  unsigned count;
  unsigned width;
  unsigned prefill;
  bool known_body = true;
};

struct Fixture {
  explicit Fixture(const Setup spec) {
    const auto count = spec.count;
    const auto width = spec.width;
    const auto prefill = spec.prefill;
    std::array<std::uint32_t, words> code{};
    code[0] = dor_immediate | (count << 8);
    code[1] = width + 1;
    const auto known = spec.known_body ? words : 2U;
    for (unsigned index = 0; index < known; ++index) {
      bus.write({.address = setup + index, .value = code.at(index)});
    }
    fetch.start(setup, 0);
    for (unsigned index = 0; index < prefill; ++index) { capture(); }
    opcode = fetch.select(setup);
    extension = fetch.select(setup + 1);
    static_cast<void>(fetch.consume(setup));
    static_cast<void>(fetch.consume(setup + 1));
    before = {.pc = setup, .la = outer_last, .lc = outer_count, .status = masked};
    after = before;
    after.pc = body + width;
    if (count != 0) {
      after.pc = body;
      after.la = body + width - 1;
      after.lc = count;
      after.sp = 2;
      after.sc = 2;
      after.status |= loop_flag;
    }
    opening = {.word = opcode.word, .extension = extension.word, .before = before, .after = after};
  }

  void capture() {
    fetch.issue();
    fetch.capture(bus);
  }

  void open() { fetch.open_loop(opening, opcode, extension); }

  DspBus bus;
  DspProgramFetch fetch;
  DspFetchedWord opcode;
  DspFetchedWord extension;
  DspLoopArchitecture before;
  DspLoopArchitecture after;
  DspLoopOpening opening;
};

void retained_prefix() {
  constexpr unsigned count = 3;
  constexpr unsigned width = 12;
  for (const unsigned prefill : {2U, 3U, 8U}) {
    for (const bool issued : {false, true}) {
      Fixture f({.count = count, .width = width, .prefill = prefill});
      const auto old = f.fetch.view();
      std::array<DspFetchedWord, 6> captured{};
      std::array<DspFetchOrigin, 6> origins{};
      for (std::size_t index = 0; index < old.used; ++index) {
        const auto word = known_word(f.fetch, index);
        captured.at(index) = word;
        origins.at(index) = f.fetch.origin(word);
      }
      if (issued) { f.fetch.issue(); }
      const auto pending = f.fetch.view().pending_address;
      f.open();
      require(f.fetch.view().used == old.used && f.fetch.view().pending_address == pending &&
                  f.fetch.view().consume_address == body,
              "Opening retains the actual sequential F2 prefix and already-issued next read");
      for (std::size_t index = 0; index < old.used; ++index) {
        require(f.fetch.queued(index) == captured.at(index) &&
                    f.fetch.origin(captured.at(index)) == origins.at(index),
                "Opening changes neither known values nor occurrence IDs nor memory origins");
      }
      require(f.fetch.confirm_opening(f.after, false),
              "Only actual native opening confirms the frame");
    }
  }
}

void empty_continuation(const bool issued) {
  constexpr unsigned count = 2;
  Fixture f({.count = count, .width = 1, .prefill = 8});
  const auto first = known_word(f.fetch, 0);
  f.open();
  require(f.fetch.view().used == 1 && f.fetch.queued(0) == first &&
              f.fetch.view().next_address == body && !f.fetch.view().pending_address,
          "A new backedge retains its actual first word and discards only wrong-path suffixes");
  require(f.fetch.confirm_opening(f.after, false), "The real setup frame confirms before use");
  static_cast<void>(f.fetch.consume(body));
  require(f.fetch.view().used == 0, "The actual last word has been consumed with no future F2 yet");
  if (issued) { f.fetch.issue(); }
  auto native = f.after;
  native.lc = 1;
  auto wrong = native;
  wrong.lc = count;
  require(!rejects([&] { static_cast<void>(f.fetch.confirm_loop(wrong, true)); }).empty(),
          "An empty future queue cannot hide an incorrect native loop count");
  require(f.fetch.confirm_loop(native, true),
          "Native boundary confirmation is independent of its pending or unissued future read");
  if (!issued) { f.fetch.issue(); }
  f.fetch.capture(f.bus);
  const auto second = known_word(f.fetch, 0);
  require(second.word == first.word && second.occurrence != first.occurrence,
          "Equal-PC iterations require distinct genuine F2 occurrence IDs");
  static_cast<void>(f.fetch.consume(body));
  native = f.before;
  native.pc = body + 1;
  require(f.fetch.confirm_loop(native, false),
          "The final native boundary restores the outer control");
  f.capture();
  require(known_word(f.fetch, 0).word.address == body + 1,
          "After exhaustion the next actual F2 is the sequential outside instruction");
}

void annulled_and_unknown() {
  constexpr unsigned width = 4;
  Fixture annulled({.count = 0, .width = width, .prefill = 8});
  annulled.open();
  require(
      annulled.fetch.view().used == 0 && annulled.fetch.view().consume_address == body + width &&
          annulled.fetch.view().next_address == body + width &&
          annulled.fetch.confirm_opening(annulled.after, true),
      "A native zero count annuls the actual speculative body and preserves the restored frame");
  annulled.capture();
  require(known_word(annulled.fetch, 0).word.address == body + width,
          "The annulled continuation still needs a real outside F2 read");
  Fixture unknown({.count = 2, .width = width, .prefill = 8, .known_body = false});
  unknown.open();
  const auto retained = unknown.fetch.sampled(0);
  require(retained && !retained->sample.data,
          "Loop metadata retains the unknownness of a genuinely sampled absent body word");
  require(unknown.fetch.confirm_opening(unknown.after, false),
          "The original native DO must retire before its unknown body can be admitted");
  require(rejects([&] {
            static_cast<void>(unknown.fetch.consume(body));
          }).contains("Unknown DSP program sample"),
          "A loop forecast never manufactures a successful unknown program read");
}

void foreign_opening() {
  Fixture f({.count = 2, .width = 4, .prefill = 8});
  const auto before = f.fetch.view();
  auto forged = f.opcode;
  ++forged.occurrence;
  require(!rejects([&] { f.fetch.open_loop(f.opening, forged, f.extension); }).empty() &&
              f.fetch.view() == before,
          "A different equal-value F2 identity fails before changing the captured queue");
  auto wrong = f.opening;
  ++wrong.after.lc;
  // Native count is independently checked by the real-core test; this model
  // requires exact context and stack geometry rather than decoding its source.
  ++wrong.after.sp;
  require(!rejects([&] { f.fetch.open_loop(wrong, f.opcode, f.extension); }).empty() &&
              f.fetch.view() == before,
          "An impossible published native stack frame fails without altering the F2 prefix");
}

void native_control_guards() {
  Fixture f({.count = 3, .width = 4, .prefill = 6});
  f.open();
  require(f.fetch.confirm_opening(f.after, false),
          "The native frame precedes fallback guard checks");
  const auto foreground = f.fetch.instruction(known_word(f.fetch, 0), 1);
  const auto before = f.fetch.view();
  require(!rejects([&] { f.fetch.redirect(body + 1, 0); }).empty() && f.fetch.view() == before,
          "The ordinary redirect still rejects active native loop metadata");
  auto forged = foreground;
  ++forged.opcode.occurrence;
  require(!rejects([&] { static_cast<void>(f.fetch.suspend_loop_forecast(forged)); }).empty() &&
              f.fetch.view() == before,
          "Interrupt fallback requires the exact front occurrence rather than an equal opcode");
  static_cast<void>(f.fetch.consume(body));
  const auto consumed = f.fetch.view();
  require(!rejects([&] {
             static_cast<void>(f.fetch.retire_native_control(
                 foreground, {.pc = body + 1, .next_pc = body + 1, .omr = 0, .redirected = true}));
           }).empty() &&
              f.fetch.view() == consumed,
          "Retirement fallback cannot discard a different consumed instruction's forecast");
}
} // namespace

int main() {
  try {
    retained_prefix();
    empty_continuation(false);
    empty_continuation(true);
    annulled_and_unknown();
    foreign_opening();
    native_control_guards();
    std::cout
        << "Loop prefill values, origins, identities, empty continuations and refusals passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
