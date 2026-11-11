// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <span>
#include <stdexcept>
#include <vector>

namespace blofeld::test {
class DspRecords {
public:
  void add(const std::uint32_t space,
           const std::uint32_t address,
           const std::span<const std::uint32_t> words) {
    word(space);
    word(static_cast<std::uint32_t>(words.size()));
    word(address);
    for (const auto value : words) { word(value); }
  }

  [[nodiscard]] std::vector<std::uint8_t> finish() {
    constexpr std::uint32_t end_word = 0x100;
    constexpr std::uint32_t mask = 0xffffff;
    word(end_word);
    word(end_word);
    word(end_word);
    constexpr std::uint64_t modulus = std::uint64_t{1} << 24U;
    word(static_cast<std::uint32_t>((modulus - (sum_ & mask)) & mask));
    return bytes_;
  }

private:
  void word(const std::uint32_t value) {
    constexpr std::uint32_t mask = 0xffffff;
    constexpr unsigned high = 16;
    constexpr unsigned middle = 8;
    constexpr std::uint32_t byte_mask = 0xff;
    if (value > mask) { throw std::invalid_argument("Test word exceeds packed 24-bit width"); }
    sum_ += value;
    bytes_.push_back(static_cast<std::uint8_t>(value >> high));
    bytes_.push_back(static_cast<std::uint8_t>((value >> middle) & byte_mask));
    bytes_.push_back(static_cast<std::uint8_t>(value & byte_mask));
  }

  std::vector<std::uint8_t> bytes_;
  std::uint64_t sum_ = 0;
};
}
