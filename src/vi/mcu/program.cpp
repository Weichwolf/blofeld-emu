// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/program.hpp"

#include "mcu/memory.hpp"

#include <algorithm>
#include <array>
#include <bitset>
#include <cpu12/core.hpp>
#include <cpu12/memory.hpp>
#include <cstddef>
#include <cstdint>
#include <format>
#include <set>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace blofeld::vi::mcu {
Program::Source::Source(const cpu12::CodeVariant &variant) : id(variant.id) {
  std::array<cpu12::Byte, cpu12::address_bytes> values{};
  std::bitset<cpu12::address_bytes> known;
  if (variant.code.empty()) { throw std::invalid_argument("MCU program requires source code"); }
  for (const auto span : variant.code) {
    if (span.words.empty() || span.words.size() > cpu12::address_bytes) {
      throw std::invalid_argument("Invalid MCU program source extent");
    }
    for (std::size_t offset = 0; offset < span.words.size(); ++offset) {
      const auto pc = static_cast<cpu12::Address>(span.address + offset);
      if (known[pc] && values[pc] != span.words[offset]) {
        throw std::invalid_argument("MCU source words disagree");
      }
      known[pc] = true;
      values[pc] = span.words[offset];
    }
  }
  bytes.reserve(known.count());
  for (std::size_t first = 0; first < values.size();) {
    if (!known[first]) {
      ++first;
      continue;
    }
    auto end = first + 1;
    while (end < values.size() && known[end]) { ++end; }
    const auto offset = bytes.size();
    bytes.insert(bytes.end(),
                 values.begin() + static_cast<std::ptrdiff_t>(first),
                 values.begin() + static_cast<std::ptrdiff_t>(end));
    code.push_back({.address = static_cast<cpu12::Address>(first),
                    .words = std::span(bytes).subspan(offset, end - first)});
    first = end;
  }
}

bool Program::Source::contains(const cpu12::Address pc) const {
  return std::ranges::any_of(code, [pc](const auto span) {
    return pc >= span.address && std::size_t{pc} - span.address < span.words.size();
  });
}

bool Program::Source::matches(const Memory &memory) const {
  for (const auto span : code) {
    for (std::size_t offset = 0; offset < span.words.size(); ++offset) {
      std::uint8_t actual = 0;
      if (!memory.read(static_cast<cpu12::Address>(span.address + offset), actual) ||
          actual != span.words[offset]) {
        return false;
      }
    }
  }
  return true;
}

Program::Program(cpu12::NativeImage image, const cpu12::CodeImage source)
    : image_(std::move(image)) {
  const auto prepared = image_.report();
  if (source.variants.empty() || prepared.prepared_entries == 0 ||
      prepared.unresolved_entries != 0 || prepared.failure.fault != cpu12::Fault::none) {
    throw std::invalid_argument("MCU program requires complete prepared code");
  }
  sources_.reserve(source.variants.size());
  std::set<std::uint64_t> ids;
  for (const auto &variant : source.variants) {
    if (!ids.insert(variant.id).second) {
      throw std::invalid_argument("Duplicate MCU program source");
    }
    sources_.emplace_back(variant);
  }
}

bool Program::contains(const std::span<const std::uint64_t> mounted,
                       const cpu12::Address pc) const {
  return std::ranges::any_of(sources_, [&](const auto &source) {
    return source.contains(pc) && std::ranges::find(mounted, source.id) != mounted.end();
  });
}

std::vector<std::uint64_t> Program::admit(Memory &memory, const cpu12::Address pc) const {
  std::vector<std::uint64_t> ids;
  std::vector<CodeBinding> bindings;
  for (std::size_t index = 0; index < sources_.size(); ++index) {
    const auto &source = sources_[index];
    if (source.matches(memory)) {
      ids.push_back(source.id);
      bindings.push_back({.identity = index + 1, .code = source.code});
    }
  }
  if (!contains(ids, pc) || !memory.bind_program(bindings)) {
    throw std::runtime_error(std::format("No prepared MCU source admits fetch at PC=${:04x}", pc));
  }
  return ids;
}

void Program::install(cpu12::Core &core, const std::span<const std::uint64_t> mounted) const {
  core.install(image_, mounted);
}
}
