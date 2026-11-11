// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/code_source.hpp"
#include "storage/dsp_records.hpp"
#include "support/dsp_records.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {
using blofeld::vi::dsp::CodeRange;
using blofeld::vi::dsp::CodeSource;
constexpr std::uint32_t address = 0x10;
constexpr std::uint32_t first = 0x123456;
constexpr std::uint32_t second = 0xabcdef;
constexpr std::uint32_t replaced = 0x765432;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

std::vector<std::uint8_t> fixture() {
  blofeld::test::DspRecords source;
  constexpr std::array initial{first, second};
  constexpr std::array replacement{replaced};
  constexpr std::array<std::uint32_t, 1> data{0xffff00};
  source.add(0, address, initial);
  source.add(1, address, data);
  source.add(2, address + 1, data);
  source.add(0, address + 1, replacement);
  return source.finish();
}

CodeSource owned_source() {
  const auto bytes = fixture();
  const auto input = blofeld::vi::DspRecords::read(bytes);
  constexpr std::array ranges{CodeRange{.address = address, .words = 2},
                              CodeRange{.address = address + 1, .words = 1}};
  return {input.records(), ranges};
}

void ordered_contexts() {
  const auto bytes = fixture();
  const auto input = blofeld::vi::DspRecords::read(bytes);
  constexpr std::array ranges{CodeRange{.address = address, .words = 2}};
  const CodeSource before(input.records().first(1), ranges);
  const CodeSource after_data(input.records().first(3), ranges);
  const auto check = [](const auto code) {
    return code.size() == 1 && code[0].address == address && code[0].words.size() == 2 &&
           code[0].words[0] == first && code[0].words[1] == second;
  };
  require(check(before.code()) && check(after_data.code()),
          "Selected prefixes retain their original P words; logical X/Y writes do not replace P");
  auto source = owned_source();
  const auto code = source.code();
  require(code.size() == 2 && code[0].words[0] == first && code[0].words[1] == replaced &&
              code[1].address == address + 1 && code[1].words[0] == replaced,
          "Later P writes apply in order while distinct overlapping instruction contexts survive");
  auto moved = std::move(source);
  require(moved.code()[0].words[1] == replaced && moved.code()[1].words[0] == replaced,
          "Code spans own their words after packed inputs, metadata and source objects are gone");
}

void missing_words() {
  const auto bytes = fixture();
  const auto input = blofeld::vi::DspRecords::read(bytes);
  constexpr std::array invalid{CodeRange{.address = address - 1, .words = 3},
                               CodeRange{.address = address, .words = 0},
                               CodeRange{.address = 0xffffff, .words = 2}};
  for (const auto range : invalid) {
    bool rejected = false;
    try {
      const CodeSource source(input.records(), {&range, 1});
    } catch (const std::invalid_argument &) { rejected = true; }
    require(rejected,
            "Missing words and invalid extents cannot become zero-filled executable code");
  }
  bool rejected = false;
  try {
    const CodeSource source(input.records(), {});
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "Absent instruction classification is not an executable-source catalogue");
}
}

int main() try {
  ordered_contexts();
  missing_words();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
