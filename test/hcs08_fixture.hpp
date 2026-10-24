// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/hcs08.hpp"
#include "blofeld/hcs08_known_memory.hpp"

#include <array>
#include <bitset>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace blofeld::test::hcs08 {
constexpr unsigned space_bytes = 65536, entry = 0x8000, operand = 0x40;
constexpr unsigned flags_fixed = 0x60, flag_v = 0x80, flag_h = 0x10;
constexpr unsigned flag_i = 8, flag_n = 4, flag_z = 2, flag_c = 1;
constexpr unsigned byte_values = 256, byte_mask = 255, word_mask = 65535;

inline void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

struct Access {
  unsigned address;
  unsigned value;
  bool write;
  std::uint8_t known = byte_mask;
  bool operator==(const Access &) const = default;
};

class Memory : public Bus {
public:
  std::array<std::uint8_t, space_bytes> bytes{};
  std::bitset<space_bytes> known, writable;
  std::array<std::uint8_t, space_bytes> partial_known{};
  mutable std::bitset<space_bytes> decoded;
  std::vector<Access> accesses;
  std::optional<InterruptRequest> interrupt;
  ExecutionContext context;
  std::uint64_t generation = 1, code_generation = 1;
  unsigned retired = 0;

  std::uint8_t read(const std::uint16_t address) override {
    if (!known[address]) { fail(address, "read"); }
    accesses.push_back({.address = address, .value = bytes[address], .write = false});
    return bytes[address];
  }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    if (!writable[address]) { fail(address, "write"); }
    bytes[address] = value;
    known.set(address);
    partial_known[address] = 0;
    ++generation;
    if (decoded[address]) { ++code_generation; }
    accesses.push_back({.address = address, .value = value, .write = true});
  }

protected:
  Hcs08KnownByte read_known_byte(const std::uint16_t address) {
    if (known[address] || !writable[address]) {
      return {.value = static_cast<std::uint8_t>(read_memory({.address = address}))};
    }
    const auto mask = partial_known[address];
    const auto value = static_cast<std::uint8_t>(bytes[address] & mask);
    accesses.push_back({.address = address, .value = value, .write = false, .known = mask});
    return {.value = value, .known = mask};
  }

  void write_known_byte(const ByteWrite access, const std::uint8_t mask) {
    if (mask == byte_mask) {
      write(access.address, access.value);
      return;
    }
    if (!writable[access.address]) { fail(access.address, "masked RAM write"); }
    const auto value = static_cast<std::uint8_t>(access.value & mask);
    bytes[access.address] = value;
    known.reset(access.address);
    partial_known[access.address] = mask;
    ++generation;
    if (decoded[access.address]) { ++code_generation; }
    accesses.push_back({.address = access.address, .value = value, .write = true, .known = mask});
  }

public:
  [[nodiscard]] std::uint8_t peek(const std::uint16_t address) const override {
    return known[address] ? bytes[address] : 0;
  }

  [[nodiscard]] std::uint64_t revision() const override { return generation; }

  [[nodiscard]] std::uint64_t code_revision() const override { return code_generation; }

  void note_code_byte(const std::uint16_t address) const override { decoded.set(address); }

  void check_execute(const InstructionFetch fetch) const override {
    require(fetch.length <= space_bytes - fetch.address, "HCS08 instruction fetch wraps");
    for (unsigned n = 0; n < fetch.length; ++n) {
      if (!known[fetch.address + n]) { fail(fetch.address + n, "fetch"); }
    }
  }

  void begin_instruction(const ExecutionContext value) override { context = value; }

  [[nodiscard]] std::optional<InterruptRequest>
  interrupt_request(std::uint8_t /*ccr*/) const override {
    return interrupt;
  }

  void retire_instruction() override { ++retired; }

  void install(const unsigned address, const std::span<const std::uint8_t> data) {
    require(data.size() <= space_bytes - address, "Own HCS08 fixture input fits memory");
    for (unsigned n = 0; n < data.size(); ++n) {
      bytes.at(address + n) = data[n];
      known.set(address + n);
      partial_known[address + n] = 0;
    }
    ++generation;
    ++code_generation;
  }

  [[noreturn]] void fail(const unsigned address, const std::string &operation) const {
    throw Fault("Unknown HCS08 fixture " + operation + " address=" + hex(address) +
                " pc=" + hex(context.pc) + " ccr=" + hex(context.ccr));
  }
};

// Only a board whose partial accesses have been qualified opts into masked RAM.
// Inheriting ordinary RAM helpers must not bypass a derived owner's policies.
class MaskedMemory : public Memory, public Hcs08KnownMemory {
public:
  Hcs08KnownByte read_known_byte(const std::uint16_t address) override {
    return Memory::read_known_byte(address);
  }

  void write_known_byte(const ByteWrite access, const std::uint8_t mask) override {
    Memory::write_known_byte(access, mask);
  }
};

inline std::string rejected(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected HCS08 strict rejection");
}
} // namespace blofeld::test::hcs08
