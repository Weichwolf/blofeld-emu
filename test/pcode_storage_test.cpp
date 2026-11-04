// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/pcode_storage.hpp"
#include "space.hh"
#include "types.h"

#include <array>
#include <cstddef>
#include <exception>
#include <iostream>
#include <span>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
constexpr unsigned storage_bytes = 64;
constexpr unsigned word_bytes = 8;
constexpr unsigned byte_bits = 8;
constexpr unsigned byte_mask = 0xff;
using Bytes = std::array<ghidra::uint1, storage_bytes>;

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw Fault(std::string(message)); }
}

ghidra::uintb expected_value(const std::span<const ghidra::uint1> bytes, const bool big) {
  ghidra::uintb result = 0;
  for (std::size_t index = 0; index < bytes.size(); ++index) {
    const auto shift = byte_bits * (big ? bytes.size() - index - 1 : index);
    result |= static_cast<ghidra::uintb>(bytes[index]) << shift;
  }
  return result;
}

void check_reads(const PcodeStorage &storage, const Bytes &expected, const bool big) {
  for (unsigned size = 1; size <= word_bytes; ++size) {
    for (unsigned offset = 0; offset + size <= expected.size(); ++offset) {
      require(storage.getValue(offset, static_cast<ghidra::int4>(size)) ==
                  expected_value(std::span(expected).subspan(offset, size), big),
              "Independent byte expectations cover both sides of word/cache boundaries");
    }
  }
}

void check_writes(PcodeStorage &storage, Bytes &expected, const bool big) {
  constexpr std::array<unsigned, 8> offsets{0, 1, 7, 8, 29, 31, 32, 33};
  constexpr ghidra::uintb value = 0x0123456789abcdef;
  for (unsigned size = 1; size <= word_bytes; ++size) {
    for (const auto offset : offsets) {
      // P-code values are already restricted to the destination width.
      const auto operand = value >> (byte_bits * (word_bytes - size));
      storage.setValue(offset, static_cast<ghidra::int4>(size), operand);
      for (unsigned index = 0; index < size; ++index) {
        const auto shift = byte_bits * (big ? size - index - 1 : index);
        expected[offset + index] = static_cast<ghidra::uint1>((operand >> shift) & byte_mask);
      }
      for (unsigned index = 0; index < expected.size(); ++index) {
        require(storage.getValue(index, 1) == expected[index],
                "Narrow and unaligned writes preserve every neighboring byte");
      }
    }
  }
}

void check_storage(const bool big) {
  ghidra::AddrSpace space(
      nullptr, nullptr, ghidra::IPTR_PROCESSOR, "register", big, 8, 1, 1, 0, 0, 0);
  PcodeStorage storage(
      {.space = &space, .page_bytes = 256, .overflow_words = 1024, .dense_words = 4});
  Bytes expected{};
  check_reads(storage, expected, big);
  for (unsigned offset = 0; offset < expected.size(); ++offset) {
    expected[offset] = static_cast<ghidra::uint1>(offset + 1);
    storage.setValue(offset, 1, expected[offset]);
  }
  check_reads(storage, expected, big);
  check_writes(storage, expected, big);
  check_reads(storage, expected, big);
  Bytes saved{};
  storage.getChunk(0, static_cast<ghidra::int4>(saved.size()), saved.data());
  require(saved == expected, "Checkpoint bytes match the independent byte image");
  PcodeStorage restored(
      {.space = &space, .page_bytes = 256, .overflow_words = 1024, .dense_words = 4});
  restored.setChunk(0, static_cast<ghidra::int4>(saved.size()), saved.data());
  check_reads(restored, expected, big);
  constexpr ghidra::uintb sparse = 0x100000007;
  constexpr ghidra::uintb value = 0xfedcba9876543210;
  require(storage.getValue(sparse, 8) == 0, "Unwritten sparse scratch storage starts at zero");
  storage.setValue(sparse, 8, value);
  require(storage.getValue(sparse, 8) == value && storage.getValue(sparse - 1, 1) == 0 &&
              storage.getValue(sparse + 8, 1) == 0,
          "High sparse addresses retain full width and preserve neighboring bytes");
  check_reads(storage, expected, big);
}
}

int main() {
  try {
    check_storage(false);
    check_storage(true);
    std::cout
        << "Verified independent LE/BE scratch bytes, aliases, cache overflow and checkpoints.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
