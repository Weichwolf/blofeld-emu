// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/hc12.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>

namespace {
constexpr unsigned entry = 0x8000;
constexpr unsigned operand = 0x1000;
constexpr unsigned instructions = 2000000;
constexpr unsigned loop_length = 4;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned initial_flags = 0xd0;

class RamBus final : public blofeld::Bus {
public:
  std::array<std::uint8_t, 65536> bytes{};
  unsigned writes = 0;

  std::uint8_t read(const std::uint16_t address) override { return bytes[address]; }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    if (address != operand) { throw blofeld::Fault("Unexpected benchmark store"); }
    bytes[address] = value;
    ++writes;
  }

  [[nodiscard]] std::uint8_t peek(const std::uint16_t address) const override {
    return bytes[address];
  }

  [[nodiscard]] std::uint64_t revision() const override { return writes; }

  [[nodiscard]] std::uint64_t code_revision() const override { return 0; }

  void check_execute(const blofeld::InstructionFetch fetch) const override {
    if (fetch.address < entry || fetch.address + fetch.length > entry + 10) {
      throw blofeld::Fault("Unexpected benchmark fetch");
    }
  }
};
}

int main() {
  try {
    RamBus bus;
    // LDAA $1000; ADDA #1; STAA $1000; BRA back to LDAA.
    constexpr std::array<std::uint8_t, 10> program{
        0xb6, 0x10, 0x00, 0x8b, 0x01, 0x7a, 0x10, 0x00, 0x20, 0xf6};
    std::ranges::copy(program, bus.bytes.begin() + entry);
    blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
    cpu.set_state({.pc = entry, .sp = operand + 256, .ccr = initial_flags});
    const auto start = std::chrono::steady_clock::now();
    for (unsigned index = 0; index < instructions; ++index) {
      if (!cpu.step().instruction_executed) { throw blofeld::Fault("Unexpected idle CPU"); }
    }
    const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - start);
    const auto state = cpu.state();
    constexpr unsigned expected = (instructions / loop_length) & byte_mask;
    if (state.pc != entry || state.d != expected << 8U || bus.bytes[operand] != expected ||
        bus.writes != instructions / loop_length) {
      throw blofeld::Fault("Independent arithmetic/store/branch expectation failed");
    }
    std::cout << "{\"instructions\":" << instructions << ",\"host_nanoseconds\":" << elapsed.count()
              << ",\"d\":" << state.d << ",\"writes\":" << bus.writes << "}\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
