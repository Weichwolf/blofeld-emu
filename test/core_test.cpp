// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/mcu_bus.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <string>

// Independent instruction/flag expectations: NXP HCS12COREUG, section 3.1 and Appendix A.
namespace {
constexpr unsigned reset_high = 0xfffe;
constexpr unsigned reset_low = 0xffff;
constexpr unsigned reset_stack = 0x00ff;
constexpr unsigned reset_flags = 0xd0;
constexpr unsigned positive_accumulator = 0x7f00;
constexpr unsigned after_load = 0x8002;
constexpr unsigned pushed_stack = 0x1ffe;
constexpr unsigned stack_low = 0x1fff;
constexpr unsigned index_high = 0x12;
constexpr unsigned index_low = 0x34;
constexpr unsigned index_value = 0x1234;
constexpr unsigned subroutine = 0x8010;
constexpr unsigned return_address = 0x800b;
constexpr unsigned clock_flags = 0x37;

class TestBus final : public blofeld::Bus {
public:
  std::uint64_t generation = 0;
  unsigned read_count = 0;
  mutable unsigned last_fetch_length = 0;
  std::array<std::uint8_t, 65536> bytes{};

  std::uint8_t read(const std::uint16_t address) override {
    ++read_count;
    return bytes[address];
  }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    ++generation;
    bytes[address] = value;
  }

  [[nodiscard]] std::uint8_t peek(const std::uint16_t address) const override {
    return bytes[address];
  }

  [[nodiscard]] std::uint64_t revision() const override { return generation; }

  void check_execute(const blofeld::InstructionFetch fetch) const override {
    last_fetch_length = fetch.length;
  }

  void code(const std::span<const std::uint8_t> program) {
    std::ranges::copy(program, bytes.begin() + 0x8000);
    bytes[reset_high] = 0x80;
    bytes[reset_low] = 0;
  }
};

void require(const bool condition, const std::string &message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void test_load_and_flags() {
  TestBus bus;
  constexpr std::array<std::uint8_t, 6> code{0x86, 0x7f, 0x8b, 0x01, 0x84, 0x00};
  bus.code(code);
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.reset();
  require(cpu.state().pc == 0x8000 && cpu.state().ccr == reset_flags &&
              cpu.state().sp == reset_stack,
          "Reset state");
  cpu.step();
  require(cpu.state().d == positive_accumulator && cpu.state().pc == after_load, "LDAA immediate");
  cpu.step();
  require(cpu.state().d == 0x8000, "ADDA result");
  require((cpu.state().ccr & 0x0f) == 0x0a, "ADDA overflow and negative flags");
  cpu.step();
  require(cpu.state().d == 0 && (cpu.state().ccr & 0x0f) == 0x04, "ANDA zero flags");
}

void test_index_and_stack() {
  TestBus bus;
  // LDS #0x2000; LDX #0x1234; PSHX; PULY; JSR $8010; NOP; NOP; RTS.
  constexpr std::array<std::uint8_t, 18> code{0xcf,
                                              0x20,
                                              0x00,
                                              0xce,
                                              0x12,
                                              0x34,
                                              0x34,
                                              0x31,
                                              0x16,
                                              0x80,
                                              0x10,
                                              0xa7,
                                              0xa7,
                                              0xa7,
                                              0xa7,
                                              0xa7,
                                              0xa7,
                                              0x3d};
  bus.code(code);
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.reset();
  cpu.step();
  cpu.step();
  cpu.step();
  require(cpu.state().sp == pushed_stack && bus.bytes[pushed_stack] == index_high &&
              bus.bytes[stack_low] == index_low,
          "PSHX byte order");
  cpu.step();
  require(cpu.state().y == index_value && cpu.state().sp == 0x2000, "PULY stack restore");
  cpu.step();
  require(cpu.state().pc == subroutine && cpu.state().sp == pushed_stack, "JSR call state");
  cpu.step();
  cpu.step();
  require(cpu.state().pc == return_address && cpu.state().sp == 0x2000,
          "RTS return state: pc=" + blofeld::hex(cpu.state().pc) +
              " sp=" + blofeld::hex(cpu.state().sp));
}

void test_code_modification_and_branch() {
  TestBus bus;
  // LDAA #1; BRA $8000. Rewriting the immediate operand must invalidate SLEIGH's decoder cache.
  constexpr std::array<std::uint8_t, 4> code{0x86, 0x01, 0x20, 0xfc};
  bus.code(code);
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.reset();
  cpu.step();
  require(cpu.state().d == 0x100, "Initial immediate operand");
  cpu.step();
  require(cpu.state().pc == 0x8000, "Backward branch wraps its signed displacement");
  constexpr unsigned operand_address = 0x8001;
  bus.write(operand_address, 2);
  cpu.step();
  require(cpu.state().d == 0x200, "Modified instruction operand must not execute from stale cache");
}

void test_conditional_branch() {
  TestBus bus;
  // LDAA #0; BEQ +2; LDAA #7; LDAA #9.
  constexpr std::array<std::uint8_t, 8> code{0x86, 0x00, 0x27, 0x02, 0x86, 0x07, 0x86, 0x09};
  bus.code(code);
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.reset();
  cpu.step();
  cpu.step();
  constexpr unsigned taken_address = 0x8006;
  require(cpu.state().pc == taken_address, "Taken BEQ target");
  cpu.step();
  constexpr unsigned expected_accumulator = 0x900;
  require(cpu.state().d == expected_accumulator, "Branch must skip LDAA #7");
  require(cpu.state().pc == 0x8000 + code.size(),
          "No eager execution or decoding of next instruction");
  bool rejected = false;
  try {
    cpu.step();
  } catch (const blofeld::Fault &) { rejected = true; }
  require(rejected, "Unsupported BGND must fail visibly on its own step");
}

void test_word_bus_transfers() {
  TestBus bus;
  // LDD #$1234; STD $0030; LDD $0030.
  constexpr std::array<std::uint8_t, 9> code{0xcc, 0x12, 0x34, 0x7c, 0x00, 0x30, 0xfc, 0x00, 0x30};
  constexpr unsigned data_address = 0x30;
  constexpr unsigned previous_byte = 0xff;
  bus.code(code);
  bus.bytes[data_address] = previous_byte;
  bus.bytes[data_address + 1] = previous_byte;
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.reset();
  bus.read_count = 0;
  cpu.step();
  cpu.step();
  require(bus.bytes[data_address] == index_high && bus.bytes[data_address + 1] == index_low,
          "Word store must replace both bytes including prior nonzero bits");
  require(bus.read_count == 0, "Word store must not synthesize reads of IO registers");
  constexpr unsigned changed_high = 0x56;
  constexpr unsigned changed_low = 0x78;
  bus.write(data_address, changed_high);
  bus.write(data_address + 1, changed_low);
  cpu.step();
  constexpr unsigned expected_word = 0x5678;
  require(cpu.state().d == expected_word && bus.read_count == 2,
          "Word read must fetch exactly two bytes in big-endian order");
}

void test_bit_branch(const bool branch_if_set, const unsigned form, const std::uint8_t mask) {
  TestBus bus;
  constexpr std::uint16_t indexed_operand = 0x2000;
  constexpr std::uint16_t direct_operand = 0x40;
  constexpr std::uint8_t displacement = 7;
  constexpr std::array<std::uint8_t, 3> set_opcodes{0x4e, 0x1e, 0x0e};
  constexpr std::array<std::uint8_t, 3> clear_opcodes{0x4f, 0x1f, 0x0f};
  const auto opcode = branch_if_set ? set_opcodes.at(form) : clear_opcodes.at(form);
  const auto operand = form == 0 ? direct_operand : indexed_operand;
  const std::array<std::uint8_t, 4> direct{opcode, direct_operand, mask, displacement};
  const std::array<std::uint8_t, 5> extended{opcode, 0x20, 0, mask, displacement};
  const std::array<std::uint8_t, 4> indexed{opcode, 0, mask, displacement};
  unsigned length = 0;
  if (form == 0) {
    bus.code(direct);
    length = direct.size();
  } else if (form == 1) {
    bus.code(extended);
    length = extended.size();
  } else {
    bus.code(indexed);
    length = indexed.size();
  }
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  constexpr std::uint8_t unchanged_ccr = 0xff;
  for (unsigned value = 0; value < 256; ++value) {
    bus.bytes[operand] = static_cast<std::uint8_t>(value);
    bus.read_count = 0;
    cpu.set_state({.pc = 0x8000, .x = indexed_operand, .ccr = unchanged_ccr});
    cpu.step();
    const bool taken = branch_if_set ? (value & mask) == mask : (value & mask) == 0;
    const auto expected = 0x8000 + length + (taken ? displacement : 0);
    if (cpu.state().pc != expected || cpu.state().ccr != unchanged_ccr || bus.read_count != 1) {
      throw blofeld::Fault(std::string(branch_if_set ? "BRSET" : "BRCLR") +
                           " form=" + std::to_string(form) + " value=" + blofeld::hex(value, 2) +
                           " mask=" + blofeld::hex(mask, 2));
    }
  }
}

void test_bit_branches_and_branch_never() {
  // S12CPUV2 instruction definitions: all selected bits, not any selected bit.
  constexpr std::array<std::uint8_t, 8> masks{0, 1, 8, 0x80, 0x0f, 0xf0, 0xaa, 0xff};
  for (unsigned form = 0; form < 3; ++form) {
    for (const auto mask : masks) {
      test_bit_branch(true, form, mask);
      test_bit_branch(false, form, mask);
    }
  }
  TestBus bus;
  constexpr std::array<std::uint8_t, 4> code{0x21, 0x7f, 0x86, 0x09};
  bus.code(code);
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.reset();
  cpu.step();
  require(cpu.state().pc == after_load && bus.last_fetch_length == 2,
          "BRN consumes its displacement byte as a two-byte NOP");
  cpu.step();
  constexpr unsigned expected_accumulator = 0x900;
  require(cpu.state().d == expected_accumulator, "BRN ignores the displacement value");
}

void test_strict_bus() {
  const std::array<std::uint8_t, 32768> flash{};
  blofeld::McuBus bus(flash);
  bool rejected = false;
  try {
    static_cast<void>(bus.read(clock_flags));
  } catch (const blofeld::Fault &) { rejected = true; }
  require(rejected, "Unimplemented clock status must fail");
  rejected = false;
  try {
    bus.write(0x8000, 1);
  } catch (const blofeld::Fault &) { rejected = true; }
  require(rejected, "Direct flash writes must fail");
}

} // namespace

int main() {
  try {
    test_load_and_flags();
    test_index_and_stack();
    test_strict_bus();
    test_code_modification_and_branch();
    test_conditional_branch();
    test_word_bus_transfers();
    test_bit_branches_and_branch_never();
    std::cout << "HC12 arithmetic/flags, stack/calls and strict bus checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
