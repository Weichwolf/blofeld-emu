// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_cycles.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace blofeld::test {
inline constexpr unsigned cycle_entry = 0x8000;
inline constexpr unsigned cycle_source = 0x1201;
inline constexpr unsigned cycle_destination = 0x1301;
inline constexpr unsigned cycle_external_destination = 0x1341;
inline constexpr unsigned cycle_pointer = 0x1402;
inline constexpr unsigned cycle_stack = 0x2201;
inline constexpr unsigned cycle_flags = 0xc0;

inline void require_cycle(const bool condition, const char *message) {
  if (!condition) { throw Fault(message); }
}

// Synthetic flat memory with unaligned word access; no board timing is inferred.
class CycleMemory final : public Bus {
public:
  std::array<std::uint8_t, 65536> bytes{};

  CycleMemory() {
    constexpr unsigned vector = 0xfffe;
    bytes[vector] = cycle_entry >> 8U;
  }

  std::uint8_t read(const std::uint16_t address) override { return bytes[address]; }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    bytes[address] = value;
    ++generation_;
  }

  [[nodiscard]] std::uint8_t peek(const std::uint16_t address) const override {
    return bytes[address];
  }

  [[nodiscard]] std::uint64_t revision() const override { return generation_; }

  void check_execute(InstructionFetch /*fetch*/) const override {}

  void install(const std::span<const std::uint8_t> code, const std::uint16_t address) {
    require_cycle(code.size() <= bytes.size() - address, "Test code exceeds its address range");
    std::ranges::copy(code, bytes.begin() + address);
    ++generation_;
  }

  std::uint64_t generation_ = 0;
};

struct CycleOperand {
  char kind;
  unsigned address;
};

struct CycleFixture {
  CycleMemory memory;
  std::vector<Hc12BusCycle> clocks;
  Hc12CycleBus bus;
  Hc12 cpu;

  CycleFixture()
      : bus(memory, [this](const auto cycle) { clocks.push_back(cycle); }),
        cpu(bus, BLOFELD_DEFAULT_SLA) {
    cpu.reset();
  }

  void install(const std::span<const std::uint8_t> code,
               CpuState state = {},
               const std::uint16_t address = cycle_entry) {
    memory.install(code, address);
    state.pc = address;
    cpu.set_state(state);
  }

  void check_operand(const std::size_t index, const CycleOperand expected) const {
    const auto cycle = clocks.at(index);
    require_cycle(
        cycle.kind == expected.kind && cycle.address == expected.address &&
            cycle.size ==
                (expected.kind == 'R' || expected.kind == 'W' || expected.kind == 'I' ? 2U : 1U),
        "Native operand address, width and manual cycle boundary must match");
  }
};
}
