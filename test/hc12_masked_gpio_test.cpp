// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "common/masked_bus.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld;
constexpr unsigned byte_mask = 255;
constexpr unsigned entry = 0x8000;
constexpr unsigned stride = 8;
constexpr unsigned direct_address = 0x80;
constexpr unsigned extended_address = 0x0278;
constexpr unsigned port_u_input = 0x0279;
constexpr unsigned port_u_direction = 0x027a;
constexpr unsigned displacement = 2;
constexpr unsigned register_d = 0x1234;
constexpr unsigned register_y = 0x4567;
constexpr unsigned stack = 0x2200;

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw Fault(std::string(message)); }
}

void rejects(const auto &operation, const std::string_view reason) {
  try {
    operation();
  } catch (const Fault &error) {
    require(std::string_view(error.what()).contains(reason), error.what());
    return;
  }
  throw Fault("Expected strict selected-read failure is absent");
}

class ObservedMemory final : public MaskedByteBus {
public:
  std::array<std::uint8_t, 65536> bytes{};
  std::uint16_t target = extended_address;
  std::uint8_t known = byte_mask;
  unsigned reads = 0;
  unsigned last_mask = 0;

  std::uint8_t read(const std::uint16_t address) override {
    if (address == target) {
      ++reads;
      require(known == byte_mask, "Unqualified whole-byte observation");
    }
    return bytes[address];
  }

  std::uint8_t read_masked_byte(const MaskedByteRead access) override {
    const auto address = access.address;
    const auto mask = access.mask;
    if (address != target) {
      return MaskedByteBus::read_masked_byte({.address = address, .mask = mask});
    }
    ++reads;
    last_mask = mask;
    require((known & mask) == mask, "Unknown selected bit");
    return static_cast<std::uint8_t>(bytes[address] & mask);
  }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    bytes[address] = value;
  }

  [[nodiscard]] std::uint8_t peek(const std::uint16_t address) const override {
    return bytes[address];
  }

  [[nodiscard]] std::uint64_t revision() const override { return 0; }

  void check_execute(InstructionFetch /*fetch*/) const override {}
};

std::vector<std::uint8_t> code(const unsigned mode, const bool set, const std::uint8_t mask) {
  constexpr unsigned direct_set = 0x4e;
  constexpr unsigned extended_set = 0x1e;
  constexpr unsigned indexed_set = 0x0e;
  constexpr std::array opcodes{direct_set, extended_set, indexed_set};
  const auto branch = static_cast<std::uint8_t>(opcodes.at(mode) + (set ? 0U : 1U));
  if (mode == 0) { return {branch, direct_address, mask, displacement}; }
  if (mode == 1) {
    return {branch, extended_address >> 8U, extended_address & byte_mask, mask, displacement};
  }
  return {branch, 0, mask, displacement}; // Literal 0,X indexed operand.
}

void literal_matrix(const unsigned mode, const bool set) {
  ObservedMemory memory;
  memory.target = mode == 0 ? direct_address : extended_address;
  for (unsigned mask = 0; mask <= byte_mask; ++mask) {
    const auto bytes = code(mode, set, static_cast<std::uint8_t>(mask));
    const auto offset =
        static_cast<std::ptrdiff_t>(entry) + (static_cast<std::ptrdiff_t>(mask) * stride);
    std::ranges::copy(bytes, memory.bytes.begin() + offset);
  }
  std::vector<Hc12BusCycle> clocks;
  Hc12CycleBus bus(memory, [&](const auto cycle) { clocks.push_back(cycle); });
  Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  for (unsigned mask = 0; mask <= byte_mask; ++mask) {
    memory.known = static_cast<std::uint8_t>(mask);
    for (unsigned value = 0; value <= byte_mask; ++value) {
      const auto pc = static_cast<std::uint16_t>(entry + (mask * stride));
      const auto ccr = static_cast<std::uint8_t>(value ^ mask);
      memory.bytes[memory.target] = static_cast<std::uint8_t>(value);
      memory.reads = 0;
      clocks.clear();
      cpu.set_state({.pc = pc,
                     .d = register_d,
                     .x = extended_address,
                     .y = register_y,
                     .sp = stack,
                     .ccr = ccr});
      static_cast<void>(cpu.step());
      const auto result = cpu.state();
      const bool taken = set ? (value & mask) == mask : (value & mask) == 0;
      const unsigned length = mode == 1 ? 5U : 4U;
      require(result.pc == pc + length + (taken ? displacement : 0U) && result.d == register_d &&
                  result.x == extended_address && result.y == register_y && result.sp == stack &&
                  result.ccr == ccr,
              "Native bit branch must preserve literal complete CPU state and all flags");
      require(memory.reads == 1 && memory.last_mask == mask && clocks.size() == length &&
                  clocks[0].kind == 'r' && clocks[0].address == memory.target &&
                  clocks[0].size == 1,
              "Direct/extended/indexed bit branch retains one actual manual operand cycle");
    }
  }
}

void indexed_updates() {
  struct Case {
    std::vector<std::uint8_t> bytes;
    unsigned initial;
    unsigned result;
  };

  constexpr unsigned next_address = extended_address + 1;
  constexpr unsigned prior_address = extended_address - 1;
  const std::array cases{
      Case{.bytes = {0x0e, 0x30, 1, 2}, .initial = extended_address, .result = next_address},
      Case{.bytes = {0x0e, 0x20, 1, 2}, .initial = prior_address, .result = extended_address},
      Case{.bytes = {0x0e, 0xe0, 1, 1, 2}, .initial = prior_address, .result = prior_address},
      Case{.bytes = {0x0e, 0xe2, 0, 1, 1, 2}, .initial = prior_address, .result = prior_address}};
  for (const auto &input : cases) {
    ObservedMemory memory;
    memory.bytes[memory.target] = 1;
    memory.known = 1;
    std::ranges::copy(input.bytes, memory.bytes.begin() + entry);
    std::vector<Hc12BusCycle> clocks;
    Hc12CycleBus bus(memory, [&](const auto cycle) { clocks.push_back(cycle); });
    Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
    cpu.set_state({.pc = entry, .x = static_cast<std::uint16_t>(input.initial)});
    static_cast<void>(cpu.step());
    const auto state = cpu.state();
    require(state.pc == entry + input.bytes.size() + displacement && state.x == input.result &&
                state.d == 0 && state.y == 0 && state.sp == 0 && state.ccr == 0 &&
                memory.reads == 1 && memory.last_mask == 1 && clocks.size() == input.bytes.size(),
            "Native pre/post updates and long indexed addresses execute exactly once");
  }
}

void gpio_knowledge() {
  McuGpio gpio;
  gpio.set_drive(McuPort::u, {.mask = 1, .levels = 1});
  require(gpio.read_masked({.address = hc12::ptu, .mask = 1}) == 1 &&
              gpio.read_masked({.address = port_u_input, .mask = 1}) == 1,
          "PTU latch/input and PTIU pin reads may observe only the known PU0 bit");
  rejects([&] { static_cast<void>(gpio.read(hc12::ptu)); }, "Unknown MCU GPIO input");
  rejects([&] { static_cast<void>(gpio.read_masked({.address = hc12::ptu, .mask = 3})); },
          "Unknown MCU GPIO input");
  gpio.write({.address = hc12::ptu, .value = 1});
  gpio.write({.address = port_u_direction, .value = 1});
  gpio.set_drive(McuPort::u, {.mask = 1});
  require(gpio.read_masked({.address = hc12::ptu, .mask = 1}) == 1,
          "A selected PTU output read observes its native latch despite opposed pad drive");
  rejects([&] { static_cast<void>(gpio.read_masked({.address = port_u_input, .mask = 1})); },
          "contention");
  gpio.write({.address = port_u_direction, .value = 0});
  gpio.set_drive(McuPort::u, {});
  rejects([&] { static_cast<void>(gpio.read_masked({.address = hc12::ptu, .mask = 1})); },
          "Unknown MCU GPIO input");
}

void native_gpio(const bool whole_read) {
  std::array<std::uint8_t, 32768> flash{};
  const auto bytes = code(1, false, 1);
  std::ranges::copy(bytes, flash.begin());
  constexpr std::array<std::uint8_t, 3> load{0xf6, 2, 0x78}; // Native LDAB $0278.
  std::ranges::copy(load, flash.begin() + 5);
  McuBus memory(flash);
  memory.set_pin_drive(McuPort::u, {.mask = 1, .levels = 1});
  std::vector<Hc12BusCycle> clocks;
  Hc12CycleBus bus(memory, [&](const auto cycle) { clocks.push_back(cycle); });
  Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.set_state({.pc = entry, .sp = stack, .ccr = hc12::reset_ccr});
  static_cast<void>(cpu.step());
  require(cpu.state().pc == entry + 5 && clocks.size() == 5 && clocks.front().address == hc12::ptu,
          "Native original-form PU0 branch runs without inventing other PortU levels");
  rejects([&] { static_cast<void>(memory.read(hc12::ptu)); }, "Unknown MCU GPIO input");
  if (whole_read) {
    rejects([&] { static_cast<void>(cpu.step()); }, "Unknown MCU GPIO input");
  } else {
    memory.set_pin_drive(McuPort::u, {});
    cpu.set_state({.pc = entry, .sp = stack, .ccr = hc12::reset_ccr});
    rejects([&] { static_cast<void>(cpu.step()); }, "MCU masked read at 0x0278 mask=0x01");
  }
  require(bus.faulted(), "A missing selected PU0 level quarantines the real timed native path");
}

void default_read() {
  ObservedMemory memory;
  constexpr unsigned address = 0x123;
  constexpr unsigned value = 0xa5;
  constexpr unsigned mask = 0x0f;
  memory.target = address;
  memory.bytes[address] = value;
  require(blofeld::read_masked_byte(memory, {.address = address, .mask = mask}) == (value & mask),
          "Ordinary memories retain their real read through the strict default interface");
  require(memory.reads == 1 &&
              blofeld::read_masked_byte(memory, {.address = address, .mask = 0}) == 0 &&
              memory.reads == 2,
          "Even a zero mask preserves one actual backend read and its side effects");
}
}

int main(const int argc, const char *const *const argv) {
  try {
    require(argc == 1 || (argc == 2 && std::string_view(argv[1]) == "gpio"),
            "Expected optional selected-GPIO scope");
    if (argc == 1) {
      for (unsigned mode = 0; mode < 3; ++mode) {
        literal_matrix(mode, false);
        literal_matrix(mode, true);
      }
    }
    indexed_updates();
    gpio_knowledge();
    native_gpio(false);
    native_gpio(true);
    default_read();
    std::cout << (argc == 1 ? "393216 native bit-branch states and selected-GPIO controls passed\n"
                            : "Selected-GPIO strict controls passed\n");
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
