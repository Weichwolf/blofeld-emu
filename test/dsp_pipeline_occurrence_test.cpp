// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_loop_fetch.hpp"
#include "dsp/dsp_pipeline_words.hpp"
#include "dsp/dsp_program_fetch.hpp"

#include <array>
#include <cstddef>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <string>

namespace blofeld::test::pipeline_occurrence {
namespace {
constexpr unsigned setup = 0x100;
constexpr unsigned body = setup + 2;
constexpr unsigned omr = 0x305;
constexpr unsigned status = 0xc00300;
constexpr unsigned count = 8;
constexpr unsigned immediate_do = 0x060080;
constexpr unsigned immediate_x0 = 0x44f400;
constexpr unsigned old_lc = 7;
constexpr unsigned old_la = 0x321;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

void refuses(const auto &operation) {
  try {
    operation();
  } catch (const Fault &) { return; }
  throw Fault("Invalid actual pipeline occurrence unexpectedly accepted");
}

void read(DspProgramFetch &fetch, DspBus &bus) {
  fetch.issue();
  fetch.capture(bus);
}

DspFetchedWord queued(const DspProgramFetch &fetch, const std::size_t index) {
  const auto result = fetch.queued(index);
  if (!result) { throw Fault("Missing actual F2 word in own pipeline fixture"); }
  return *result;
}

void begin(DspProgramFetch &fetch, DspBus &bus, const unsigned size) {
  bus.write({.address = setup, .value = immediate_do | (count << 8)});
  bus.write({.address = setup + 1, .value = body + size - 1});
  fetch.start(setup, omr);
  read(fetch, bus);
  read(fetch, bus);
  const auto opcode = queued(fetch, 0);
  const auto extension = queued(fetch, 1);
  // Independently declared normal-mode control fixtures, not native CPU grants.
  const DspLoopOpening opening{
      .word = opcode.word,
      .extension = extension.word,
      .before = {.pc = setup, .omr = omr, .la = old_la, .lc = old_lc, .status = status},
      .after = {.pc = body,
                .omr = omr,
                .la = body + size - 1,
                .lc = count,
                .sp = 2,
                .sc = 2,
                .status = status | 0x8000}};
  static_cast<void>(fetch.consume(setup));
  static_cast<void>(fetch.consume(setup + 1));
  fetch.open_loop(opening, opcode, extension);
  require(fetch.confirm_opening(opening.after, false), "Own loop must confirm its opening");
}

void repeated_words(const bool two_words) {
  DspBus bus;
  bus.write({.address = body, .value = two_words ? immediate_x0 : 0});
  DspProgramFetch fetch;
  begin(fetch, bus, two_words ? 2U : 1U);
  std::array<DspFetchedWord, dsp_program_fetch_capacity> actual{};
  for (std::size_t index = 0; index < actual.size(); ++index) {
    if (two_words) { bus.write({.address = body + 1, .value = static_cast<unsigned>(index)}); }
    read(fetch, bus);
    actual.at(index) = queued(fetch, index);
    require(actual.at(index).word.address == body + (two_words ? index % 2 : 0),
            "Actual F2 addresses must follow the independent repeated instruction shape");
  }
  const std::size_t stride = two_words ? 2 : 1;
  const std::array<std::optional<DspFetchedWord>, 3> stages{
      actual.at(2 * stride), actual.at(stride), actual.front()};
  const std::array<std::optional<DspFetchedWord>, 2> extensions{
      two_words ? std::optional(actual.at(3)) : std::nullopt,
      two_words ? std::optional(actual.at(1)) : std::nullopt};
  const auto selected = select_pipeline_words(actual, stages, extensions);
  require(selected.next == actual.at(3 * stride),
          "Decode must select the fourth actual occurrence, despite equal PC and opcode");
  require(selected.ag1_extension == (two_words ? std::optional(actual.at(5)) : std::nullopt),
          "AG1 must retain its own adjacent extension rather than an older same-PC sample");
  if (two_words) {
    if (!selected.ag1_extension) { throw Fault("Expected actual AG1 extension"); }
    require(selected.ag1_extension->word.value == 5 && actual.at(1).word.value == 1 &&
                actual.at(3).word.value == 3,
            "Real F2 samples must preserve three independently changed literal values");
  }
  auto wrong = stages;
  wrong[2] = actual.at(stride);
  refuses([&] { static_cast<void>(select_pipeline_words(actual, wrong, extensions)); });
  wrong = stages;
  auto forged = actual.at(stride);
  ++forged.occurrence;
  wrong[1] = forged;
  refuses([&] { static_cast<void>(select_pipeline_words(actual, wrong, extensions)); });
  auto bad_extensions = extensions;
  bad_extensions[1] = actual.at(5);
  refuses([&] { static_cast<void>(select_pipeline_words(actual, stages, bad_extensions)); });
  const auto truncated = std::span(actual).first(two_words ? 5 : 2);
  refuses([&] { static_cast<void>(select_pipeline_words(truncated, stages, extensions)); });
  const auto drained = std::span(actual).first(3 * stride);
  require(!select_pipeline_words(drained, stages, extensions).next,
          "A genuine empty younger frontier must starve Decode without selecting an old PC");
}
}
}

int main() {
  try {
    blofeld::test::pipeline_occurrence::repeated_words(false);
    blofeld::test::pipeline_occurrence::repeated_words(true);
    std::cout << "Actual repeated F2 stage selection, changed extensions and stale guards passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
