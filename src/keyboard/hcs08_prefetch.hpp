// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "keyboard/hcs08.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>

namespace blofeld::detail::hcs08 {
struct PrefetchShape {
  unsigned bytes, periods;
  bool push = false;
  bool read = false, branch = false, write = false, returning = false, jump = false;
};

[[nodiscard]] inline std::optional<PrefetchShape> jump_shape(const unsigned opcode) {
  constexpr unsigned direct = 0xbc;
  constexpr unsigned extended = 0xcc;
  constexpr unsigned indexed16 = 0xdc;
  constexpr unsigned indexed8 = 0xec;
  constexpr unsigned indexed = 0xfc;
  switch (opcode) {
    case direct:
    case indexed8: return PrefetchShape{.bytes = 2, .periods = 3, .jump = true};
    case extended:
    case indexed16: return PrefetchShape{.bytes = 3, .periods = 4, .jump = true};
    case indexed: return PrefetchShape{.bytes = 1, .periods = 3, .jump = true};
    default: return std::nullopt;
  }
}

[[nodiscard]] inline std::optional<PrefetchShape> peripheral_shape(const unsigned opcode,
                                                                   const bool jumps) {
  if (jumps) {
    if (const auto shape = jump_shape(opcode)) { return shape; }
  }
  constexpr unsigned sta_direct = 0xb7;
  constexpr unsigned sta_extended = 0xc7;
  constexpr unsigned mov_immediate = 0x6e;
  constexpr unsigned lda_indexed8 = 0xe6;
  constexpr unsigned lda_indexed16 = 0xd6;
  constexpr unsigned pull_h = 0x8a;
  constexpr unsigned rti = 0x80;
  if (opcode == rti) { return PrefetchShape{.bytes = 1, .periods = 9, .returning = true}; }
  if (opcode == sta_direct) { return PrefetchShape{.bytes = 2, .periods = 3, .write = true}; }
  if (opcode == sta_extended || opcode == mov_immediate) {
    return PrefetchShape{.bytes = 3, .periods = 4, .write = true};
  }
  if (opcode == pull_h) { return PrefetchShape{.bytes = 1, .periods = 3, .read = true}; }
  if (opcode == lda_indexed8 || opcode == lda_indexed16) {
    const unsigned bytes = opcode == lda_indexed8 ? 2U : 3U;
    return PrefetchShape{.bytes = bytes, .periods = bytes + 1, .read = true};
  }
  return std::nullopt;
}

[[nodiscard]] inline std::optional<PrefetchShape> scanner_shape(const unsigned opcode,
                                                                const unsigned second) {
  constexpr unsigned aslx = 0x58;
  constexpr unsigned pull_x = 0x88;
  constexpr unsigned prefix = 0x9e;
  constexpr unsigned sta_stack = 0xe7;
  constexpr unsigned stx_stack = 0xef;
  if (opcode == aslx) { return PrefetchShape{.bytes = 1, .periods = 1}; }
  if (opcode == pull_x) { return PrefetchShape{.bytes = 1, .periods = 3, .read = true}; }
  if (opcode == prefix && (second == sta_stack || second == stx_stack)) {
    return PrefetchShape{.bytes = 3, .periods = 4, .write = true};
  }
  return std::nullopt;
}

[[nodiscard]] inline std::optional<PrefetchShape> reference_shape(
    const unsigned opcode, const unsigned second, const Hcs08PrefetchContract contract) {
  const bool scanner = contract == Hcs08PrefetchContract::reference_scanner_byte_cycles;
  if (scanner) {
    if (const auto shape = scanner_shape(opcode, second)) { return shape; }
  }
  const bool jumps = contract == Hcs08PrefetchContract::reference_jump_program_cycles || scanner;
  return peripheral_shape(opcode, jumps);
}

[[nodiscard]] inline PrefetchShape prefetch_shape(const unsigned opcode,
                                                  const ExecutionContext context,
                                                  const Hcs08PrefetchContract contract,
                                                  const unsigned second = 0) {
  constexpr std::array<unsigned, 10> single{
      0x9d, 0x98, 0x99, 0x4f, 0x5f, 0x8c, 0x97, 0x9f, 0x4c, 0x5c};
  constexpr std::array<unsigned, 5> immediate{0xa6, 0xae, 0xa4, 0xa5, 0xa1};
  constexpr std::array<unsigned, 3> push{0x87, 0x89, 0x8b};
  constexpr unsigned ldhx = 0x45;
  constexpr unsigned mul = 0x42;
  constexpr unsigned div = 0x52;
  if (std::ranges::find(single, opcode) != single.end()) { return {.bytes = 1, .periods = 1}; }
  if (std::ranges::find(immediate, opcode) != immediate.end()) {
    return {.bytes = 2, .periods = 2};
  }
  if (std::ranges::find(push, opcode) != push.end()) {
    return {.bytes = 1, .periods = 2, .push = true};
  }
  if (opcode == ldhx) { return {.bytes = 3, .periods = 3}; }
  if (opcode == mul || opcode == div) { return {.bytes = 1, .periods = opcode == mul ? 5U : 6U}; }
  const bool peripheral = contract == Hcs08PrefetchContract::reference_peripheral_program_cycles ||
                          contract == Hcs08PrefetchContract::reference_jump_program_cycles ||
                          contract == Hcs08PrefetchContract::reference_scanner_byte_cycles;
  if (peripheral) {
    if (const auto shape = reference_shape(opcode, second, contract)) { return *shape; }
  }
  if (contract == Hcs08PrefetchContract::reference_short_branch_refill || peripheral) {
    constexpr std::array<unsigned, 16> short_branches{0x20,
                                                      0x21,
                                                      0x22,
                                                      0x23,
                                                      0x24,
                                                      0x25,
                                                      0x26,
                                                      0x27,
                                                      0x28,
                                                      0x29,
                                                      0x2a,
                                                      0x2b,
                                                      0x90,
                                                      0x91,
                                                      0x92,
                                                      0x93};
    if (std::ranges::find(short_branches, opcode) != short_branches.end()) {
      return {.bytes = 2, .periods = 3, .branch = true};
    }
  }
  if (contract == Hcs08PrefetchContract::reference_data_branch_cycles ||
      contract == Hcs08PrefetchContract::reference_short_branch_refill || peripheral) {
    constexpr unsigned last_bit_branch = 0x0f;
    constexpr std::array<unsigned, 3> direct{0xb6, 0xbe, 0xb1};
    constexpr std::array<unsigned, 3> extended{0xc6, 0xce, 0xc1};
    if (opcode <= last_bit_branch) {
      return {.bytes = 3, .periods = 5, .read = true, .branch = true};
    }
    if (std::ranges::find(direct, opcode) != direct.end()) {
      return {.bytes = 2, .periods = 3, .read = true};
    }
    if (std::ranges::find(extended, opcode) != extended.end()) {
      return {.bytes = 3, .periods = 4, .read = true};
    }
  }
  throw Fault("HCS08 prefetch instruction address=" + hex(context.pc) + " pc=" + hex(context.pc) +
              " ccr=" + hex(context.ccr) + " opcode=" + hex(opcode) +
              ": unqualified program/data/flow cycle consumer");
}

// Only native instruction decoding/knowledge checks use this view. Actual
// operand transactions still belong to the original bus and native executor.
class PrefetchView final : public Bus {
public:
  PrefetchView(Bus &bus, const std::uint16_t start, const std::span<const std::uint8_t> bytes)
      : bus_(bus), start_(start), bytes_(bytes) {}

  std::uint8_t read(const std::uint16_t address) override {
    throw Fault("HCS08 prefetch view data read address=" + hex(address) + " pc=" + hex(start_));
  }

  void write(const std::uint16_t address, std::uint8_t /*value*/) override {
    throw Fault("HCS08 prefetch view data write address=" + hex(address) + " pc=" + hex(start_));
  }

  [[nodiscard]] std::uint64_t revision() const override { return bus_.revision(); }

  [[nodiscard]] std::uint8_t peek(const std::uint16_t address) const override {
    const auto offset = static_cast<std::uint16_t>(address - start_);
    return offset < bytes_.size() ? bytes_[offset] : bus_.peek(address);
  }

  void check_execute(const InstructionFetch fetch) const override {
    const auto offset = static_cast<std::uint16_t>(fetch.address - start_);
    constexpr unsigned address_space = 65536;
    if (fetch.length == 0 || fetch.length > address_space - fetch.address ||
        offset >= bytes_.size() || fetch.length > bytes_.size() - offset) {
      throw Fault("HCS08 prefetch execute address=" + hex(fetch.address) + " pc=" + hex(start_) +
                  ": instruction exceeds its genuinely captured program bytes or wraps");
    }
  }

private:
  Bus &bus_;
  std::uint16_t start_;
  std::span<const std::uint8_t> bytes_;
};
} // namespace blofeld::detail::hcs08
