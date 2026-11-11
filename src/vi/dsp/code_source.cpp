// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/code_source.hpp"

#include "storage/dsp_records.hpp"
#include "storage/update_reader.hpp"

#include <cstddef>
#include <dsp56300/memory.hpp>
#include <format>
#include <map>
#include <span>
#include <stdexcept>
#include <vector>

namespace blofeld::vi::dsp {
namespace {
std::map<dsp56300::Word, dsp56300::Word> program_words(const std::span<const DspRecord> records) {
  std::map<dsp56300::Word, dsp56300::Word> result;
  for (const auto &record : records) {
    const auto count = record.payload.size() / 3;
    if (record.space > 2 || record.payload.empty() || record.payload.size() % 3 != 0 ||
        record.address >= dsp56300::address_words ||
        count > dsp56300::address_words - record.address) {
      throw std::invalid_argument("Invalid DSP source record");
    }
    if (record.space != 0) { continue; }
    update::Reader reader(record.payload);
    auto address = record.address;
    while (!reader.empty()) { result[address++] = reader.big(3); }
  }
  return result;
}
}

CodeSource::CodeSource(const std::span<const DspRecord> records,
                       const std::span<const CodeRange> ranges) {
  if (ranges.empty()) { throw std::invalid_argument("DSP code source has no instruction ranges"); }
  const auto program = program_words(records);
  words_.reserve(ranges.size());
  code_.reserve(ranges.size());
  for (const auto range : ranges) {
    if (range.words == 0 || range.address >= dsp56300::address_words ||
        range.words > dsp56300::address_words - range.address) {
      throw std::invalid_argument("Invalid DSP executable source extent");
    }
    auto &words = words_.emplace_back();
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
    code_.push_back({.address = range.address, .words = words});
  }
}
}
