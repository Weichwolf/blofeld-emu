// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/code_source.hpp"

#include "storage/dsp_records.hpp"
#include "storage/update_reader.hpp"

#include <algorithm>
#include <cstddef>
#include <dsp56300/core.hpp>
#include <dsp56300/memory.hpp>
#include <format>
#include <map>
#include <span>
#include <stdexcept>
#include <vector>

namespace blofeld::vi::dsp {
namespace {
using Words = std::map<dsp56300::Word, dsp56300::Word>;

bool valid_extent(const dsp56300::Word address, const std::size_t words) {
  return words != 0 && address < dsp56300::address_words &&
         words <= dsp56300::address_words - address;
}

Words program_words(const std::span<const DspRecord> records) {
  Words result;
  for (const auto &record : records) {
    const auto count = record.payload.size() / 3;
    if (record.space > 2 || record.payload.size() % 3 != 0 ||
        !valid_extent(record.address, count)) {
      throw std::invalid_argument("Invalid DSP source record");
    }
    if (record.space != 0) { continue; }
    update::Reader reader(record.payload);
    auto address = record.address;
    while (!reader.empty()) { result[address++] = reader.big(3); }
  }
  return result;
}

Words program_words(const std::span<const dsp56300::CodeSpan> writes) {
  Words result;
  for (const auto &write : writes) {
    if (!valid_extent(write.address, write.words.size()) ||
        std::ranges::any_of(write.words,
                            [](const auto word) { return word > dsp56300::word_mask; })) {
      throw std::invalid_argument("Invalid DSP stored program write");
    }
    auto address = write.address;
    for (const auto word : write.words) { result[address++] = word; }
  }
  return result;
}

std::vector<std::vector<dsp56300::Word>> classified_words(const Words &program,
                                                          const std::span<const CodeRange> ranges) {
  if (ranges.empty()) { throw std::invalid_argument("DSP code source has no instruction ranges"); }
  std::vector<std::vector<dsp56300::Word>> result;
  result.reserve(ranges.size());
  for (const auto range : ranges) {
    if (!valid_extent(range.address, range.words)) {
      throw std::invalid_argument("Invalid DSP executable source extent");
    }
    auto &words = result.emplace_back();
    words.reserve(range.words);
    for (std::size_t index = 0; index < range.words; ++index) {
      const auto address = range.address + static_cast<dsp56300::Word>(index);
      const auto found = program.find(address);
      if (found == program.end()) {
        throw std::invalid_argument(
            std::format("DSP executable source has no stored word at P:${:06x}", address));
      }
      words.push_back(found->second);
    }
  }
  return result;
}
}

CodeSource::CodeSource(const std::span<const DspRecord> records,
                       const std::span<const CodeRange> ranges)
    : words_(classified_words(program_words(records), ranges)) {
  index(ranges);
}

CodeSource::CodeSource(const std::span<const dsp56300::CodeSpan> writes,
                       const std::span<const CodeRange> ranges)
    : words_(classified_words(program_words(writes), ranges)) {
  index(ranges);
}

void CodeSource::index(const std::span<const CodeRange> ranges) {
  code_.reserve(ranges.size());
  for (std::size_t position = 0; position < ranges.size(); ++position) {
    code_.push_back({.address = ranges[position].address, .words = words_[position]});
  }
}
}
