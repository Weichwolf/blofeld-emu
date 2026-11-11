// SPDX-License-Identifier: GPL-3.0-only
#include "storage/dsp_records.hpp"
#include "support/dsp_records.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <vector>

namespace {
void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

std::vector<std::uint8_t> fixture() {
  blofeld::test::DspRecords source;
  constexpr std::uint32_t address = 0x10;
  constexpr std::array<std::uint32_t, 2> first{0x123456, 0xabcdef};
  constexpr std::array<std::uint32_t, 1> data{0xffff00};
  constexpr std::array<std::uint32_t, 1> replacement{0x765432};
  source.add(0, address, first);
  source.add(1, address, data);
  source.add(0, address + 1, replacement);
  return source.finish();
}

void ordered_records() {
  auto bytes = fixture();
  constexpr std::size_t prefix = 51;
  require(bytes.size() == prefix, "The independent fixture has exact packed-word bounds");
  constexpr std::array<std::uint8_t, 3> checksum{0xcb, 0xa7, 0x53};
  require(std::ranges::equal(std::span(bytes).last(3), checksum),
          "The fixture checksum matches the independently summed original word list");
  constexpr std::array<std::uint8_t, 2> tail{0x3e, 0x7d};
  bytes.insert(bytes.end(), tail.begin(), tail.end());
  const auto input = bytes;
  const auto source = blofeld::vi::DspRecords::read(bytes);
  const auto records = source.records();
  require(records.size() == 3 && source.bytes() == prefix && bytes == input,
          "Parsing retains record multiplicity and leaves original input and trailing data intact");
  constexpr std::uint32_t address = 0x10;
  constexpr std::size_t second_offset = 15;
  constexpr std::size_t third_offset = 27;
  require(records[0].space == 0 && records[0].address == address && records[0].byte_offset == 0 &&
              records[0].payload.size() == 6 && records[0].payload.data() == bytes.data() + 9 &&
              records[1].space == 1 && records[1].address == address &&
              records[1].byte_offset == second_offset && records[2].space == 0 &&
              records[2].address == address + 1 && records[2].byte_offset == third_offset,
          "P/X separation, source offsets and the later overlapping P write remain explicit");
  constexpr std::array<std::uint8_t, 6> first{0x12, 0x34, 0x56, 0xab, 0xcd, 0xef};
  require(std::ranges::equal(records[0].payload, first),
          "Stored source bytes keep big-endian word order without flattening the later write");
}

void reject(const std::span<const std::uint8_t> input) {
  bool rejected = false;
  try {
    static_cast<void>(blofeld::vi::DspRecords::read(input));
  } catch (const std::runtime_error &) { rejected = true; }
  require(rejected, "Malformed record input must fail before it becomes executable source");
}

void malformed() {
  const auto bytes = fixture();
  constexpr std::array<std::size_t, 7> truncated{0, 1, 8, 9, 14, 39, 50};
  for (const auto length : truncated) { reject(std::span(bytes).first(length)); }
  auto invalid = bytes;
  invalid[2] = 3;
  reject(invalid);
  invalid = bytes;
  invalid[5] = 0;
  reject(invalid);
  invalid = bytes;
  constexpr std::uint8_t high = 0xff;
  constexpr std::uint8_t almost_high = 0xfe;
  invalid[6] = high;
  invalid[7] = high;
  invalid[8] = almost_high;
  invalid[5] = 3;
  reject(invalid);
  invalid = bytes;
  invalid[10] ^= 1;
  reject(invalid);
  invalid = bytes;
  invalid.back() ^= 1;
  reject(invalid);
}
}

int main() try {
  ordered_records();
  malformed();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
