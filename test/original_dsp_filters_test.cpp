// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/firmware.hpp"
#include "dsp_filter_fixture.hpp"
#include "dsp_filter_model.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <set>
#include <string>

namespace blofeld::test::filters {
namespace {
constexpr std::size_t program_record = 71;
constexpr std::size_t lookup_record = 41;
constexpr unsigned program_base = 0x126b;
constexpr unsigned program_words = 926;
constexpr unsigned calls_per_group = 79872;
constexpr unsigned cutoff_cases = 768;
constexpr std::array<std::int64_t, 13> resonances{-half,
                                                  -1,
                                                  0,
                                                  1,
                                                  0x200000,
                                                  0x400000,
                                                  0x600000,
                                                  0x783e0e,
                                                  0x783e0f,
                                                  0x783e10,
                                                  0x7ffffe,
                                                  0x7fffff,
                                                  0x7c0000};

std::set<std::int64_t> cutoffs() {
  constexpr std::array<std::int64_t, 6> offsets{0, 1, 65535, 65536, 131070, 131071};
  constexpr std::int64_t segment_count = 64;
  std::set<std::int64_t> result{-half, -1, 0, 1, half - 1};
  for (std::int64_t segment = -segment_count; segment < segment_count; ++segment) {
    for (const auto offset : offsets) { result.insert((segment * segment_width) + offset); }
  }
  require(result.size() == cutoff_cases, "Every signed cutoff segment has six selected offsets");
  return result;
}

void group(Fixture &fixture, const bool cascade, std::ostream &trace) {
  const auto before = fixture.counts();
  for (const auto kind : {Kind::low, Kind::band, Kind::high, Kind::notch}) {
    for (const auto cutoff : cutoffs()) {
      for (const auto resonance : resonances) {
        for (const bool twos : {false, true}) {
          fixture.run({.kind = kind,
                       .cutoff = cutoff,
                       .resonance = resonance,
                       .twos = twos,
                       .pattern = (fixture.counts().calls / 2) % 4,
                       .cascade = cascade});
        }
      }
    }
  }
  fixture.intact();
  const auto &after = fixture.counts();
  require(after.calls - before.calls == calls_per_group && after.wide > before.wide &&
              after.clipped > before.clipped && after.halved > before.halved &&
              after.capped > before.capped,
          "Complete filter group visits each declared branch");
  trace << (cascade ? "cascade" : "single") << ' ' << after.calls - before.calls << ' '
        << after.wide - before.wide << ' ' << after.clipped - before.clipped << ' '
        << after.halved - before.halved << ' ' << after.capped - before.capped << '\n';
}

void run(const Firmware &firmware, std::ostream &trace) {
  const auto &program = firmware.dsp.at(program_record);
  const auto &lookup = firmware.dsp.at(lookup_record);
  require(program.space == 0 && program.address == program_base &&
              program.words.size() == program_words && lookup.space == 2 &&
              lookup.address == table_base && lookup.words.size() == table_words,
          "Original 1.25 filter program and lookup records retain their exact layout");
  Fixture fixture({.program = program.words, .table = lookup.words});
  group(fixture, false, trace);
  group(fixture, true, trace);
  trace << "visited" << std::hex;
  for (const auto pc : fixture.visited()) { trace << ' ' << pc; }
  trace << std::dec << "\ninstructions " << fixture.counts().instructions << '\n';
  constexpr auto section_samples = 3ULL * calls_per_group * block_length;
  std::cout
      << fixture.counts().calls << " original non-FM filter calls; " << section_samples
      << " independently checked section-output samples. No full boot or board audio proof.\n";
}
}
}

int main(const int count, char **arguments) {
  try {
    blofeld::test::require(count == 3,
                           "Usage: original-dsp-filters-test firmware.zip private.trace");
    const auto firmware = blofeld::Firmware::load(arguments[1]);
    std::ofstream trace(arguments[2]);
    blofeld::test::require(trace.is_open(), "Cannot create private original-filter evidence");
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    blofeld::test::filters::run(firmware, trace);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
