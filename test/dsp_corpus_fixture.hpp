// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp56kEmu/assembler.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <span>
#include <sstream>
#include <string>
#include <vector>

namespace blofeld::test {
inline constexpr unsigned corpus_boot = 0xff0000;
inline constexpr unsigned corpus_word_mask = 0xffffff;
inline constexpr unsigned program_words = 4096;
// Supplier bounds include zero-count loops with a conservative 70000-step budget.
// Program size and dynamic instruction count are independent limits.
inline constexpr unsigned maximum_case_steps = 100000;
inline constexpr unsigned x_words = 3072;
inline constexpr unsigned y_words = 2048;
inline constexpr unsigned snapshot_ram_begin = 0x400;
inline constexpr unsigned snapshot_ram_end = 0x500;
inline constexpr unsigned program_dump_begin = 0x7c0;

struct CorpusFill {
  bool enabled = false;
  // X multiplier/addend, Y multiplier/addend, SSH/SSL ramp bases.
  std::array<unsigned, 6> values{};

  [[nodiscard]] unsigned memory(const DspSpace area, const unsigned address) const {
    if (!enabled || area == DspSpace::p) { return 0; }
    const unsigned index = area == DspSpace::x ? 0 : 2;
    return static_cast<unsigned>(
        (static_cast<std::uint64_t>(values.at(index)) * address + values.at(index + 1)) &
        corpus_word_mask);
  }

  [[nodiscard]] unsigned stack(const bool high, const unsigned slot) const {
    return enabled ? values.at(high ? 4U : 5U) + slot - 1U : 0;
  }
};

struct CorpusCase {
  std::string name;
  unsigned start = 0;
  unsigned end = 0;
  unsigned limit = 0;
};

struct CorpusMetadata {
  CorpusFill fill;
  std::vector<CorpusCase> cases;
};

inline void corpus_require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

inline std::string corpus_number(const std::uint64_t value) {
  std::ostringstream text;
  text << '$' << std::hex << value;
  return text.str();
}

inline std::ifstream corpus_input(const std::filesystem::path &path) {
  constexpr unsigned maximum_bytes = 262144;
  corpus_require(std::filesystem::file_size(path) <= maximum_bytes, "Oversize DSP corpus input");
  std::ifstream input(path);
  corpus_require(input.is_open(), "Cannot open DSP corpus: " + path.string());
  return input;
}

inline void read_fill(std::istringstream &fields, CorpusFill &fill) {
  constexpr unsigned maximum_stack_base = 0xf1;
  corpus_require(!fill.enabled, "Duplicate DSP corpus fill");
  for (auto &value : fill.values) {
    corpus_require(static_cast<bool>(fields >> std::hex >> value) && value <= corpus_word_mask,
                   "Malformed DSP corpus fill");
  }
  fields >> std::ws;
  corpus_require(fields.eof() && (fill.values[0] & 1U) != 0 && (fill.values[2] & 1U) != 0 &&
                     fill.values[4] <= maximum_stack_base && fill.values[5] <= maximum_stack_base,
                 "Unsupported DSP corpus fill");
  fill.enabled = true;
}

inline CorpusMetadata read_cases(const std::filesystem::path &path) {
  constexpr unsigned maximum_name = 80;
  auto input = corpus_input(path);
  CorpusMetadata metadata;
  std::string line;
  while (std::getline(input, line)) {
    std::istringstream fields(line);
    std::string tag;
    corpus_require(static_cast<bool>(fields >> tag), "Malformed DSP corpus metadata");
    if (tag == "fill") {
      corpus_require(metadata.cases.empty(), "Misplaced DSP corpus fill");
      read_fill(fields, metadata.fill);
      continue;
    }
    CorpusCase item;
    corpus_require(static_cast<bool>(fields >> item.name >> std::hex >> item.start >> item.end >>
                                     std::dec >> item.limit),
                   "Malformed DSP corpus metadata");
    corpus_require(tag == "case" && item.start < item.end && item.end <= program_words &&
                       item.limit != 0 && item.limit <= maximum_case_steps &&
                       item.name.size() <= maximum_name,
                   "Unsupported DSP corpus case");
    fields >> std::ws;
    corpus_require(fields.eof(), "Trailing DSP corpus metadata");
    corpus_require(metadata.cases.empty() || metadata.cases.back().end <= item.start,
                   "Overlapping or reordered DSP corpus cases");
    metadata.cases.push_back(item);
  }
  corpus_require(input.eof() && !metadata.cases.empty(), "Incomplete DSP corpus metadata");
  return metadata;
}

inline void initialize_memory(DspBus &bus, const CorpusFill &fill) {
  for (const auto area : {DspSpace::p, DspSpace::x, DspSpace::y}) {
    unsigned end = program_words;
    if (area == DspSpace::x) { end = x_words; }
    if (area == DspSpace::y) { end = y_words; }
    for (unsigned address = 0; address < end; ++address) {
      bus.write({.space = area, .address = address, .value = fill.memory(area, address)});
    }
  }
}

inline void load_corpus(DspBus &bus, const std::filesystem::path &path) {
  auto input = corpus_input(path);
  std::string line;
  while (std::getline(input, line)) {
    std::istringstream fields(line);
    char space = ' ';
    unsigned address = 0;
    unsigned value = 0;
    corpus_require(static_cast<bool>(fields >> space >> std::hex >> address),
                   "Malformed DSP corpus address");
    if (space == 'I') {
      // The assembler appends address/symbol records, not executable words.
      std::string symbol;
      corpus_require(static_cast<bool>(fields >> symbol) && symbol.size() <= 128 &&
                         address < program_words,
                     "Malformed DSP corpus symbol");
      fields >> std::ws;
      corpus_require(fields.eof(), "Trailing DSP corpus symbol");
      continue;
    }
    corpus_require(static_cast<bool>(fields >> value), "Malformed DSP corpus word");
    DspSpace area = DspSpace::p;
    unsigned end = program_words;
    if (space == 'X') {
      area = DspSpace::x;
      end = x_words;
    }
    if (space == 'Y') {
      area = DspSpace::y;
      end = y_words;
    }
    corpus_require((space == 'P' || space == 'X' || space == 'Y') && address < end &&
                       value <= corpus_word_mask,
                   "Unsupported DSP corpus address or value");
    fields >> std::ws;
    corpus_require(fields.eof(), "Trailing DSP corpus word");
    bus.write({.space = area, .address = address, .value = value});
  }
  corpus_require(input.eof(), "Incomplete DSP corpus words");
}

class CorpusPreamble {
public:
  void emit(const std::string &text) {
    const auto result = assembler_.assemble(text.c_str());
    corpus_require(result.success(), "Cannot assemble own corpus preamble: " + text);
    for (unsigned index = 0; index < result.wordCount; ++index) {
      words_.push_back(result.word[index]);
    }
  }

  void immediate(const std::string &reg, const unsigned value) {
    emit("move #>" + corpus_number(value) + ',' + reg);
  }

  void install(DspBus &bus, const unsigned entry) {
    constexpr unsigned absolute_jump = 0x0af080;
    words_.push_back(absolute_jump);
    words_.push_back(entry);
    bus.install_rom({.start = corpus_boot, .words = words_});
  }

private:
  dsp56k::Assembler assembler_;
  std::vector<std::uint32_t> words_;
};

inline void install_preamble(DspBus &bus, const unsigned entry, const CorpusFill &fill) {
  CorpusPreamble preamble;
  for (const auto *name : {"a0", "a1", "a2", "b0", "b1", "b2", "x0", "x1", "y0", "y1"}) {
    preamble.immediate(name, 0);
  }
  for (unsigned index = 0; index < 8; ++index) {
    preamble.immediate("r" + std::to_string(index), 0);
    preamble.immediate("n" + std::to_string(index), 0);
    preamble.immediate("m" + std::to_string(index), corpus_word_mask);
  }
  preamble.immediate("sz", 0);
  preamble.immediate("ep", 0);
  preamble.immediate("sp", 0);
  for (unsigned index = 0; index < 15; ++index) {
    preamble.immediate("ssh", fill.stack(true, index + 1));
    preamble.immediate("ssl", fill.stack(false, index + 1));
  }
  preamble.immediate("sp", 0);
  preamble.immediate("sc", 0);
  preamble.immediate("la", corpus_word_mask);
  preamble.immediate("lc", 0);
  preamble.immediate("vba", corpus_boot);
  constexpr unsigned hardware_corpus_omr = 0x30f;
  constexpr unsigned hardware_corpus_sr = 0xc00300;
  preamble.immediate("omr", hardware_corpus_omr);
  preamble.immediate("sr", hardware_corpus_sr);
  preamble.install(bus, entry);
}
}
