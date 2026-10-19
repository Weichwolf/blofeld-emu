// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_cycles.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace {
constexpr unsigned entry = 0x8000;
constexpr unsigned handler = 0x9000;
constexpr unsigned irq_vector = 0xfff2;
constexpr unsigned reset_vector = 0xfffe;
constexpr unsigned stack = 0x2000;
constexpr unsigned flags = 0xc0;
constexpr unsigned nop = 0xa7;
constexpr unsigned rts = 0x3d;
constexpr unsigned prefix = 0x18;
constexpr unsigned unsupported_mac = 0x12;
constexpr unsigned accumulator = 0xa500;

void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const blofeld::Fault &error) { return error.what(); }
  throw blofeld::Fault("Expected timed execution to reject");
}

class FlatBus final : public blofeld::Bus {
public:
  std::array<std::uint8_t, 65536> bytes{};
  std::vector<blofeld::MemoryRead> reads;
  std::optional<blofeld::InterruptRequest> pending;
  std::uint64_t generation = 0;
  bool acknowledge = true;

  FlatBus() {
    bytes[reset_vector] = entry >> 8U;
    bytes[irq_vector] = handler >> 8U;
    bytes[handler] = 0x0b;
  }

  std::uint8_t read(const std::uint16_t address) override { return bytes[address]; }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    bytes[address] = value;
    ++generation;
  }

  std::uint64_t read_memory(const blofeld::MemoryRead access) override {
    reads.push_back(access);
    return Bus::read_memory(access);
  }

  [[nodiscard]] std::uint8_t peek(const std::uint16_t address) const override {
    return bytes[address];
  }

  [[nodiscard]] std::uint64_t revision() const override { return generation; }

  void check_execute(blofeld::InstructionFetch /*fetch*/) const override {}

  [[nodiscard]] std::optional<blofeld::InterruptRequest>
  interrupt_request(std::uint8_t /*ccr*/) const override {
    return pending;
  }

  void acknowledge_interrupt(std::uint16_t /*vector*/) override {
    if (acknowledge) { pending.reset(); }
  }

  void program(const std::span<const std::uint8_t> code) {
    std::ranges::copy(code, bytes.begin() + entry);
  }
};

void test_native_operand_slots() {
  FlatBus memory;
  constexpr std::array<std::uint8_t, 16> program{
      0xcf, 0x20, 0, 0x86, 0xa5, 0x7a, 0x12, 0, 0xf6, 0x12, 0, 0x37, 0x32, 0x16, 0x80, 0x20};
  memory.program(program);
  constexpr unsigned subroutine = 0x8020;
  memory.bytes[subroutine] = rts;
  std::vector<blofeld::Hc12BusCycle> clocks;
  blofeld::Hc12CycleBus bus(memory, [&clocks](const auto cycle) { clocks.push_back(cycle); });
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.reset();
  for (unsigned instruction = 0; instruction < 8; ++instruction) { cpu.step(); }
  constexpr unsigned total_cycles = 28;
  constexpr unsigned final_pc = entry + program.size();
  constexpr unsigned result = 0xa5a5;
  require(bus.cycles() == total_cycles && clocks.size() == total_cycles &&
              cpu.state().pc == final_pc && cpu.state().sp == stack && cpu.state().d == result,
          "Native loads/stores/stack/call/return must retain literal CPU state and cycle counts");
  constexpr std::array<unsigned, 7> boundaries{1, 10, 12, 16, 17, 20, 24};
  constexpr std::array<unsigned, 7> addresses{
      reset_vector, 0x1200, 0x1200, stack - 1, stack - 1, stack - 2, stack - 2};
  constexpr std::array<char, 7> kinds{'V', 'w', 'r', 's', 'u', 'S', 'U'};
  for (std::size_t index = 0; index < boundaries.size(); ++index) {
    const auto cycle = clocks.at(boundaries.at(index) - 1);
    require(cycle.address == addresses.at(index) && cycle.kind == kinds.at(index),
            "Native operand transactions must occur at independent NXP bus-cycle boundaries");
  }
}

void test_constant_address_word_operands() {
  FlatBus memory;
  // LDD #1234; STD $80; LDD $80; ADDD $80; LDD $0080.
  constexpr std::array<std::uint8_t, 12> program{
      0xcc, 0x12, 0x34, 0x5c, 0x80, 0xdc, 0x80, 0xd3, 0x80, 0xfc, 0, 0x80};
  memory.program(program);
  constexpr unsigned location = 0x80;
  std::vector<blofeld::Hc12BusCycle> operands;
  blofeld::Hc12CycleBus bus(memory, [&operands, location](const auto cycle) {
    if (cycle.address == location) { operands.push_back(cycle); }
  });
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.reset();
  for (unsigned instruction = 0; instruction < 5; ++instruction) { cpu.step(); }
  constexpr unsigned total = 18;
  constexpr unsigned value = 0x1234;
  constexpr std::array<char, 4> kinds{'W', 'R', 'R', 'R'};
  require(
      bus.cycles() == total && cpu.state().d == value && operands.size() == kinds.size(),
      "Constant-address copy/arithmetic p-code must retain native word operands and literal data");
  for (std::size_t index = 0; index < operands.size(); ++index) {
    require(operands.at(index).size == 2 && operands.at(index).kind == kinds.at(index),
            "Direct-page word operands must not pass through byte-bank split accesses");
  }
}

void test_rti_order_and_chaining() {
  for (const bool chain : {false, true}) {
    FlatBus memory;
    memory.bytes[entry] = nop;
    memory.acknowledge = !chain;
    blofeld::Hc12CycleBus bus(memory, [](auto /*cycle*/) {});
    blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
    cpu.reset();
    constexpr blofeld::CpuState original{
        .pc = entry, .d = 0x1234, .x = 0x5678, .y = 0x9abc, .sp = stack, .ccr = flags};
    cpu.set_state(original);
    memory.pending = {.vector = irq_vector};
    require(cpu.step().interrupt_vector == irq_vector, "Native IRQ must fetch its real vector");
    constexpr unsigned after_entry = 14;
    require(bus.cycles() == after_entry && cpu.state().sp == stack - 9,
            "Reset plus vector-first native IRQ uses five plus nine cycles");
    memory.reads.clear();
    const auto returned = cpu.step();
    constexpr std::array<unsigned, 5> addresses{
        stack - 9, stack - 8, stack - 6, stack - 2, stack - 4};
    for (std::size_t index = 0; index < addresses.size(); ++index) {
      require(memory.reads.at(index).address == addresses.at(index) &&
                  memory.reads.at(index).size == (index == 0 ? 1U : 2U),
              "NXP RTI reads CCR, B:A, X, return PC, then Y with native operand widths");
    }
    constexpr unsigned normal_return = 8;
    constexpr unsigned chained_return = 11;
    require(bus.cycles() == after_entry + (chain ? chained_return : normal_return),
            "RTI normal/reused-frame latency must match the two independent NXP descriptions");
    if (chain) {
      require(returned.stack_reused && returned.interrupt_vector == irq_vector &&
                  cpu.state().pc == handler && cpu.state().sp == stack - 9,
              "RTI must reuse its actual restored frame for the next IRQ");
      memory.pending.reset();
      cpu.step();
    }
    const auto state = cpu.state();
    require(state.pc == original.pc && state.sp == original.sp && state.d == original.d &&
                state.x == original.x && state.y == original.y && state.ccr == original.ccr,
            "Native RTI must restore literal independent registers after its timed reads");
  }
}

void test_visible_rejection_and_precommit_clocks() {
  FlatBus memory;
  constexpr std::array<std::uint8_t, 3> program{0x7a, 0x12, 0};
  memory.program(program);
  constexpr unsigned rejected_address = 0x1200;
  blofeld::Hc12CycleBus bus(memory, [rejected_address](const auto cycle) {
    if (cycle.address == rejected_address) { throw blofeld::Fault("Unqualified memory wait"); }
  });
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.reset();
  cpu.set_state({.pc = entry, .d = accumulator, .sp = stack, .ccr = flags});
  const auto error = rejects([&cpu] { cpu.step(); });
  require(bus.faulted() && error.contains("pc=0x8000") && error.contains("address=0x1200") &&
              memory.bytes[rejected_address] == 0,
          "A rejected operand clock must preserve address/context and precede the native store");
  static_cast<void>(rejects([&bus] { bus.advance_idle(1); }));
  FlatBus unsupported;
  unsupported.bytes[entry] = prefix;
  unsupported.bytes[entry + 1] = unsupported_mac;
  blofeld::Hc12CycleBus other(unsupported, [](auto /*cycle*/) {});
  blofeld::Hc12 second(other, BLOFELD_DEFAULT_SLA);
  second.reset();
  static_cast<void>(rejects([&second] { second.step(); }));
  require(other.cycles() == 5 && second.state().pc == entry,
          "Unsupported timing must reject before instruction effects or speculative clocks");
}
}

int main() {
  try {
    test_native_operand_slots();
    test_constant_address_word_operands();
    test_rti_order_and_chaining();
    test_visible_rejection_and_precommit_clocks();
    std::cout
        << "Native HC12 cycle boundaries, IRQ/RTI order, chaining and failure context passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
