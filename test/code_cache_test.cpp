// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/mcu_bus.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
constexpr unsigned ram_code = 0x1200;
constexpr unsigned second_code = 0x1400;
constexpr unsigned data_address = 0x1800;
constexpr unsigned flash_code = 0x8000;
constexpr unsigned qram_code = 0x2000;
constexpr unsigned qram_mapping = 0x12;
constexpr unsigned qram_at_2000 = 0x21;
constexpr unsigned qram_at_8000 = 0x81;
constexpr unsigned byte_value = 0x55;
constexpr unsigned first_accumulator = 0x0100;
constexpr unsigned second_accumulator = 0x0200;
constexpr unsigned qram_accumulator = 0x0700;
constexpr std::array<std::uint8_t, 4> loop{0x86, 1, 0x20, 0xfc};
constexpr blofeld::CpuState ram_state{.pc = ram_code, .sp = 0x2000, .ccr = 0xd0};

void require(const bool condition, const std::string &message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void reject(const auto &operation, const std::string &message) {
  try {
    operation();
  } catch (const blofeld::Fault &) { return; }
  throw blofeld::Fault(message);
}

std::array<std::uint8_t, 32768> flash() {
  std::array<std::uint8_t, 32768> bytes{};
  std::ranges::copy(loop, bytes.begin());
  constexpr unsigned nop = 0xa7;
  bytes[8] = nop;
  return bytes;
}

void write_code(blofeld::McuBus &bus,
                const std::uint16_t destination,
                const std::array<std::uint8_t, 4> &bytes = loop) {
  for (unsigned index = 0; index < bytes.size(); ++index) {
    bus.write(static_cast<std::uint16_t>(destination + index), bytes[index]);
  }
}

void test_data_writes_and_instruction_mutation() {
  const auto image = flash();
  blofeld::McuBus bus(image);
  write_code(bus, ram_code);
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.set_state(ram_state);
  static_cast<void>(cpu.disassemble());
  const auto decoded = bus.code_revision();
  const auto memory = bus.revision();
  bus.write(data_address, byte_value);
  require(bus.code_revision() == decoded && bus.revision() > memory,
          "Unrelated data writes retain the decoder while the general bus revision advances");
  cpu.step();
  require(cpu.state().d == first_accumulator, "The original RAM immediate instruction executes");
  cpu.step();
  bus.write(ram_code + 1, 2);
  require(bus.code_revision() > decoded, "Writing an observed operand invalidates decoded code");
  cpu.step();
  require(cpu.state().d == second_accumulator,
          "Changed RAM operands cannot execute from stale translation");
  cpu.step();
  constexpr unsigned load_b = 0xc6;
  bus.write(ram_code, load_b);
  bus.write(ram_code + 1, 3);
  cpu.step();
  constexpr unsigned changed_register = 0x0203;
  require(cpu.state().d == changed_register, "Changing an opcode changes its destination register");
  constexpr unsigned branch_to_self = 0xfe;
  bus.write(ram_code + 3, branch_to_self);
  cpu.step();
  require(cpu.state().pc == ram_code + 2,
          "A rewritten branch displacement changes the real control flow");
  const auto before_failure = bus.code_revision();
  bus.write(flash_code + 1, 2);
  constexpr unsigned flash_status = 0x105;
  constexpr unsigned access_error = 0x10;
  require(bus.code_revision() == before_failure && bus.read(flash_code + 1) == 1 &&
              (bus.read(flash_status) & access_error) != 0,
          "An illegal flash byte write sets ACCERR without mutating code or invalidating a "
          "successful store");
}

void test_self_modifying_instruction_and_wide_store() {
  const auto image = flash();
  blofeld::McuBus bus(image);
  // LDAA #2; STAA $1206; LDAA #1; BRA $1205.
  constexpr std::array<std::uint8_t, 9> code{0x86, 2, 0x7a, 0x12, 6, 0x86, 1, 0x20, 0xfc};
  for (unsigned index = 0; index < code.size(); ++index) {
    bus.write(static_cast<std::uint16_t>(ram_code + index), code[index]);
  }
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  auto state = ram_state;
  state.pc = ram_code + 5;
  cpu.set_state(state);
  static_cast<void>(cpu.disassemble()); // Cache the future operand before the CPU rewrites it.
  const auto before = bus.code_revision();
  cpu.set_state(ram_state);
  cpu.step();
  cpu.step();
  require(bus.code_revision() > before && bus.read(ram_code + 6) == 2,
          "An executed store invalidates a previously decoded future instruction");
  cpu.step();
  require(cpu.state().d == second_accumulator,
          "The self-modified immediate is re-decoded on execution");
  // Rewriting a complete word updates both its opcode and operand.
  // The independently expected instruction becomes LDAB #7.
  constexpr unsigned rewritten = 0xc607;
  const auto observed = bus.code_revision();
  bus.write_memory({.address = ram_code + 5, .size = 2, .value = rewritten});
  require(bus.code_revision() > observed, "Word stores invalidate every affected observed byte");
  state = ram_state;
  state.pc = ram_code + 5;
  state.d = second_accumulator;
  cpu.set_state(state);
  cpu.step();
  constexpr unsigned after_word_store = 0x0207;
  require(cpu.state().d == after_word_store, "A word rewrite updates both opcode and operand");
}

void test_remapping_and_instruction_permissions() {
  const auto image = flash();
  blofeld::McuBus bus(image);
  bus.write(qram_mapping, qram_at_2000);
  bus.retire_instruction();
  constexpr std::array<std::uint8_t, 4> overlay{0x86, 7, 0x20, 0xfc};
  write_code(bus, qram_code, overlay);
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  auto state = ram_state;
  state.pc = flash_code;
  cpu.set_state(state);
  cpu.step();
  cpu.step();
  require(cpu.state().d == first_accumulator && cpu.state().pc == flash_code,
          "The initial ROM loop supplies the independent accumulator value");
  const auto saved = cpu.state();
  state = saved;
  state.pc = flash_code + 8;
  cpu.set_state(state);
  static_cast<void>(cpu.disassemble()); // Cache a known ROM NOP before overlaying unknown RAM.
  cpu.set_state(saved);
  const auto before = bus.code_revision();
  bus.write(qram_mapping, qram_at_8000);
  require(bus.code_revision() == before && bus.peek(flash_code + 1) == 1,
          "A pending map write does not change the current instruction image");
  bus.retire_instruction();
  require(bus.code_revision() > before,
          "Committed remapping invalidates all decoded address interpretations");
  cpu.step();
  require(cpu.state().d == qram_accumulator,
          "QRAM overlay replaces a previously decoded ROM instruction");
  bus.write(qram_mapping, qram_at_2000);
  bus.retire_instruction();
  cpu.step();
  cpu.step();
  require(cpu.state().d == first_accumulator && cpu.state().pc == flash_code + 2,
          "Removing the overlay restores the ROM interpretation");
  bus.write(qram_mapping, qram_at_8000);
  bus.retire_instruction();
  state.pc = flash_code + 8; // This QRAM byte has never been initialized.
  cpu.set_state(state);
  reject([&cpu] { cpu.step(); },
         "Decoder lookahead never grants execution permission to unknown RAM");
}

void test_reset_and_shared_bus_observations() {
  const auto image = flash();
  blofeld::McuBus bus(image);
  write_code(bus, ram_code);
  constexpr std::array<std::uint8_t, 4> other_loop{0x86, 3, 0x20, 0xfc};
  write_code(bus, second_code, other_loop);
  blofeld::Hc12 first(bus, BLOFELD_DEFAULT_SLA);
  blofeld::Hc12 second(bus, BLOFELD_DEFAULT_SLA);
  first.set_state(ram_state);
  auto state = ram_state;
  state.pc = second_code;
  second.set_state(state);
  static_cast<void>(first.disassemble());
  static_cast<void>(second.disassemble());
  bus.write(ram_code + 1, 2);
  first.step();
  static_cast<void>(second.disassemble()); // Must not erase the first decoder's observed bytes.
  const auto before = bus.code_revision();
  bus.write(ram_code + 1, 4);
  require(bus.code_revision() > before,
          "Decoder refreshes preserve the union of observed code bytes");
  first.set_state(ram_state);
  first.step();
  constexpr unsigned updated = 0x0400;
  require(first.state().d == updated,
          "Interleaved decoders cannot miss another observed-code write");
  const auto before_reset = bus.code_revision();
  bus.reset();
  require(bus.code_revision() > before_reset, "External reset invalidates every existing decoder");
  const auto reset_revision = bus.code_revision();
  bus.write(ram_code + 1, 5);
  require(bus.code_revision() == reset_revision,
          "Reset releases old observation history until code is decoded again");
  first.set_state(ram_state);
  first.step();
  constexpr unsigned after_reset = 0x0500;
  require(first.state().d == after_reset,
          "Reset generation still prevents reuse of a pre-reset instruction");
}

void test_observed_second_byte_of_word_transfers() {
  const auto image = flash();
  blofeld::McuBus bus(image);
  bus.note_code_byte(data_address + 1);
  const auto before = bus.code_revision();
  constexpr unsigned pattern = 0x1234;
  bus.write_memory({.address = data_address, .size = 2, .value = pattern});
  require(bus.code_revision() > before &&
              bus.read_memory({.address = data_address, .size = 2}) == pattern,
          "An observed second byte is sufficient to invalidate a normal word store");
  constexpr unsigned usb_control = 0x300;
  constexpr unsigned usb_selector = 0x30e;
  constexpr unsigned usb_high_word = 0x310;
  bus.write(usb_control, 0x40);
  bus.write(usb_selector, 8);
  bus.note_code_byte(usb_high_word + 1);
  const auto before_usb = bus.code_revision();
  bus.write_memory({.address = usb_high_word, .size = 2});
  require(bus.code_revision() > before_usb,
          "The USB word-only path also honors a decoder-observed second byte");
}

void test_cached_operations_use_live_registers_flags_and_data() {
  auto image = flash();
  // INCA; BNE $8000; NOP. Both the operation and conditional branch repeat.
  constexpr std::array<std::uint8_t, 4> counted{0x42, 0x26, 0xfd, 0xa7};
  std::ranges::copy(counted, image.begin());
  blofeld::McuBus bus(image);
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  auto state = ram_state;
  state.pc = flash_code;
  constexpr unsigned starting_value = 0xfe55;
  constexpr unsigned carry_flags = 0xd1;
  state.d = starting_value;
  state.ccr = carry_flags;
  cpu.set_state(state);
  const auto revision = bus.code_revision();
  for (unsigned index = 0; index < 4; ++index) { cpu.step(); }
  constexpr unsigned wrapped = 0x0055;
  constexpr unsigned zero_and_carry = 0xd5;
  require(cpu.state().pc == flash_code + 3 && cpu.state().d == wrapped &&
              cpu.state().ccr == zero_and_carry && bus.code_revision() == revision,
          "Cached operations and conditional branches use current registers/flags on every visit");
  constexpr unsigned before_overflow = 0x7f55;
  state.d = before_overflow;
  cpu.set_state(state);
  cpu.step();
  constexpr unsigned overflowed = 0x8055;
  constexpr unsigned negative_overflow_carry = 0xdb;
  require(cpu.state().d == overflowed && cpu.state().ccr == negative_overflow_carry,
          "A cached INCA observes externally restored register state and computes independent "
          "overflow flags");

  // A second CPU reads changing data through the same immutable instruction.
  constexpr std::array<std::uint8_t, 5> loads{0xfc, 0x18, 0, 0x20, 0xfb};
  std::ranges::copy(loads, image.begin());
  blofeld::McuBus data_bus(image);
  blofeld::Hc12 data_cpu(data_bus, BLOFELD_DEFAULT_SLA);
  state = ram_state;
  state.pc = flash_code;
  data_cpu.set_state(state);
  constexpr unsigned first_word = 0x1234;
  constexpr unsigned next_word = 0x5678;
  data_bus.write_memory({.address = data_address, .size = 2, .value = first_word});
  data_cpu.step();
  data_cpu.step();
  const auto before_data = data_bus.code_revision();
  data_bus.write_memory({.address = data_address, .size = 2, .value = next_word});
  data_cpu.step();
  require(data_cpu.state().d == next_word && data_bus.code_revision() == before_data,
          "Cached LDD still performs a real bus load and cannot retain a previous data value");
}

void test_translation_capacity_and_reentrant_invalidation() {
  auto image = flash();
  constexpr unsigned instruction_count = 160;
  constexpr unsigned load_a = 0x86;
  for (std::size_t index = 0; index < instruction_count; ++index) {
    image[index * 2] = load_a;
    image[(index * 2) + 1] = static_cast<std::uint8_t>(index);
  }
  blofeld::McuBus bus(image);
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  auto state = ram_state;
  state.pc = flash_code;
  cpu.set_state(state);
  for (unsigned index = 0; index < instruction_count; ++index) { cpu.step(); }
  constexpr unsigned last_literal = 0x9f00;
  constexpr unsigned end = 0x8140;
  require(cpu.state().pc == end && cpu.state().d == last_literal,
          "Execution remains correct beyond the bounded translation working set");
  cpu.set_state(state);
  cpu.step();
  require(cpu.state().d == 0 && cpu.state().pc == flash_code + 2,
          "Revisiting an evicted translation reconstructs the original instruction");

  // An IO observer must not free an instruction that is currently executing.
  constexpr std::array<std::uint8_t, 6> stores{0x86, 1, 0x7a, 0, 0, 0xa7};
  for (unsigned index = 0; index < stores.size(); ++index) {
    bus.write(static_cast<std::uint16_t>(ram_code + index), stores[index]);
  }
  cpu.set_state(ram_state);
  cpu.step();
  bus.set_observer([&bus, &cpu](const blofeld::BusEvent & /*event*/) {
    bus.write(ram_code + 1, 2);
    static_cast<void>(cpu.disassemble());
  });
  bool rejected = false;
  try {
    cpu.step();
  } catch (const blofeld::Fault &fault) {
    const std::string message = fault.what();
    require(message.find("active HC12") != std::string::npos,
            "Reentrant invalidation reports its exact lifetime conflict");
    rejected = true;
  }
  require(rejected,
          "A decoder refresh cannot invalidate an instruction from inside its own bus callback");
  bus.set_observer({});
  cpu.set_state(ram_state);
  cpu.step();
  require(cpu.state().d == second_accumulator,
          "Exception unwinding releases the execution guard for a later safe decode");
}
}

int main() {
  try {
    test_data_writes_and_instruction_mutation();
    test_self_modifying_instruction_and_wide_store();
    test_remapping_and_instruction_permissions();
    test_reset_and_shared_bus_observations();
    test_observed_second_byte_of_word_transfers();
    test_cached_operations_use_live_registers_flags_and_data();
    test_translation_capacity_and_reentrant_invalidation();
    std::cout
        << "Observed-code invalidation, mutation, map, reset and shared-decoder checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
