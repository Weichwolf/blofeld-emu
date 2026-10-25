// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "mcu/hc12.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <span>

namespace blofeld::test {
inline constexpr unsigned minmax_entry = 0x8000;
inline constexpr unsigned minmax_source = 0x1202;
inline constexpr unsigned minmax_pointer = 0x1304;
inline constexpr unsigned minmax_byte_mask = 0xff;
inline constexpr unsigned minmax_word_mask = 0xffff;
inline constexpr unsigned minmax_low_canary = 0x5a;
inline constexpr unsigned minmax_initial_y = 0x2468;
inline constexpr unsigned minmax_initial_stack = 0x3579;

inline void require_minmax(const bool condition, const char *message) {
  if (!condition) { throw Fault(message); }
}

struct MinmaxAccess {
  bool write = false;
  unsigned address = 0;
  unsigned size = 0;
  std::uint64_t value = 0;
  bool operator==(const MinmaxAccess &) const = default;
};

// Explicit flat CPU test memory. Instruction/data revisions are independent;
// no native model RAM default, peripheral implementation or timing is supplied.
class MinmaxMemory final : public Bus {
public:
  std::array<std::uint8_t, 65536> bytes{};
  std::array<MinmaxAccess, 8> accesses{};
  unsigned access_count = 0;
  unsigned source = minmax_source;
  bool change_after_read = false;
  bool reject_read = false;
  bool reject_write = false;

  std::uint8_t read(const std::uint16_t address) override { return bytes.at(address); }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    bytes.at(address) = value;
    ++revision_;
  }

  std::uint64_t read_memory(const MemoryRead access) override {
    if (reject_read && access.address == source) { throw Fault("Own minmax read failure"); }
    const auto value = Bus::read_memory(access);
    accesses.at(access_count++) = {.address = access.address, .size = access.size, .value = value};
    if (change_after_read && access.address == source) {
      write(access.address, static_cast<std::uint8_t>(~bytes.at(access.address)));
    }
    return value;
  }

  void write_memory(const MemoryWrite access) override {
    if (reject_write && access.address == source) { throw Fault("Own minmax write failure"); }
    accesses.at(access_count++) = {
        .write = true, .address = access.address, .size = access.size, .value = access.value};
    Bus::write_memory(access);
  }

  [[nodiscard]] std::uint8_t peek(const std::uint16_t address) const override {
    return bytes.at(address);
  }

  [[nodiscard]] std::uint64_t revision() const override { return revision_; }

  [[nodiscard]] std::uint64_t code_revision() const override { return code_revision_; }

  void check_execute(InstructionFetch /*fetch*/) const override {}

  void install(const std::span<const std::uint8_t> code, const unsigned alignment = 0) {
    std::ranges::copy(code, bytes.begin() + minmax_entry + alignment);
    ++code_revision_;
    ++revision_;
  }

  void operand(const unsigned value, const bool word) {
    if (word) {
      bytes.at(source) = static_cast<std::uint8_t>(value >> 8U);
      bytes.at(source + 1) = static_cast<std::uint8_t>(value & minmax_byte_mask);
    } else {
      bytes.at(source) = static_cast<std::uint8_t>(value);
    }
    access_count = 0;
    ++revision_;
  }

  std::uint64_t revision_ = 0;
  std::uint64_t code_revision_ = 0;
};

struct MinmaxOperation {
  unsigned opcode;
  bool word;
  bool memory;
  bool maximum;
};

inline constexpr std::array minmax_operations{
    MinmaxOperation{.opcode = 0x18, .word = false, .memory = false, .maximum = true},
    MinmaxOperation{.opcode = 0x19, .word = false, .memory = false, .maximum = false},
    MinmaxOperation{.opcode = 0x1c, .word = false, .memory = true, .maximum = true},
    MinmaxOperation{.opcode = 0x1d, .word = false, .memory = true, .maximum = false},
    MinmaxOperation{.opcode = 0x1a, .word = true, .memory = false, .maximum = true},
    MinmaxOperation{.opcode = 0x1b, .word = true, .memory = false, .maximum = false},
    MinmaxOperation{.opcode = 0x1e, .word = true, .memory = true, .maximum = true},
    MinmaxOperation{.opcode = 0x1f, .word = true, .memory = true, .maximum = false}};

struct MinmaxValues {
  unsigned left;
  unsigned right;
  unsigned flags;
};

inline unsigned minmax_flags(const MinmaxValues values, const bool word) {
  const auto mask = word ? minmax_word_mask : minmax_byte_mask;
  const auto sign = (mask + 1) / 2;
  const auto result = (values.left + mask + 1 - values.right) & mask;
  constexpr unsigned preserved = 0xf0;
  constexpr unsigned negative = 8;
  constexpr unsigned zero = 4;
  constexpr unsigned overflow = 2;
  const bool signed_overflow = ((values.left ^ values.right) & (values.left ^ result) & sign) != 0;
  return (values.flags & preserved) | ((result & sign) != 0 ? negative : 0U) |
         (result == 0 ? zero : 0U) | (signed_overflow ? overflow : 0U) |
         (values.left < values.right ? 1U : 0U);
}

inline unsigned minmax_value(const MinmaxOperation operation, const MinmaxValues values) {
  return operation.maximum ? std::max(values.left, values.right)
                           : std::min(values.left, values.right);
}

inline unsigned minmax_accumulator(const MinmaxOperation operation, const unsigned value) {
  return operation.word ? value : (value << 8U) | minmax_low_canary;
}
}

namespace blofeld::test {
void check_minmax_result(const MinmaxMemory &memory,
                         const Hc12 &cpu,
                         MinmaxOperation operation,
                         MinmaxValues values,
                         unsigned leading_reads = 0);
unsigned minmax_word_cases();
unsigned minmax_timing_cases();
unsigned minmax_side_effect_cases();
unsigned minmax_machine_cases();
}
