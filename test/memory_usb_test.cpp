// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_usb.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {
// Independent references: S12CPUV2 operand sizes and exception sequence;
// S12USB20D6E2FV1 sections 3.1.7/3.1.8, 9S12UF32DGV1 integration map.
constexpr unsigned entry = 0x8000;
constexpr unsigned handler = 0x9000;
constexpr unsigned stack = 0x5000;
constexpr unsigned vector = 0xffd6;
constexpr unsigned select_high = 0x30e;
constexpr unsigned select_low = 0x30f;
constexpr unsigned data_high = 0x310;
constexpr unsigned data_low = 0x312;
constexpr unsigned clock_control = 0x300;
constexpr unsigned module_clock = 0x40;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned word_mask = 0xffff;
constexpr unsigned buffer_mask = 0x77;
constexpr unsigned selector_reserved_bits = 0xf0;
constexpr unsigned buffer_fixture = 0x23;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void reject(const auto &operation, const std::string &message) {
  try {
    operation();
  } catch (const blofeld::Fault &) { return; }
  throw blofeld::Fault(message);
}

struct Operand {
  bool write = false;
  std::uint16_t address = 0;
  unsigned size = 1;
  std::uint64_t value = 0;
};

class WidthBus final : public blofeld::Bus {
public:
  std::array<std::uint8_t, 65536> bytes{};
  std::vector<Operand> operands;
  std::vector<Operand> byte_accesses;
  std::optional<std::uint16_t> failed_write;
  std::optional<blofeld::InterruptRequest> request;
  std::uint64_t generation = 0;

  std::uint8_t read(const std::uint16_t address) override {
    const auto value = bytes[address];
    byte_accesses.push_back({.address = address, .value = value});
    return value;
  }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    if (failed_write == address) { throw blofeld::Fault("Injected byte-write fault"); }
    byte_accesses.push_back({.write = true, .address = address, .value = value});
    bytes[address] = value;
    ++generation;
  }

  std::uint64_t read_memory(const blofeld::MemoryRead access) override {
    operands.push_back({.address = access.address, .size = access.size});
    return Bus::read_memory(access);
  }

  void write_memory(const blofeld::MemoryWrite access) override {
    operands.push_back(
        {.write = true, .address = access.address, .size = access.size, .value = access.value});
    Bus::write_memory(access);
  }

  [[nodiscard]] std::uint8_t peek(const std::uint16_t address) const override {
    return bytes[address];
  }

  [[nodiscard]] std::uint64_t revision() const override { return generation; }

  void check_execute(blofeld::InstructionFetch /*fetch*/) const override {}

  [[nodiscard]] std::optional<blofeld::InterruptRequest>
  interrupt_request(std::uint8_t /*ccr*/) const override {
    return request;
  }

  void acknowledge_interrupt(std::uint16_t /*address*/) override { request.reset(); }
};

void test_byte_order_widths_and_failures() {
  WidthBus bus;
  constexpr unsigned location = 0x4000;
  constexpr std::uint64_t literal = 0x0123456789abcdef;
  constexpr std::array<std::uint8_t, 8> golden{0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef};
  bus.write_memory({.address = location, .size = 8, .value = literal});
  require(bus.byte_accesses.size() == 8, "A wide store writes only its eight requested bytes");
  for (unsigned index = 0; index < golden.size(); ++index) {
    const auto access = bus.byte_accesses[index];
    require(access.write && access.address == location + index && access.value == golden[index],
            "Operand stores preserve literal high-to-low byte order without prereads");
  }
  bus.byte_accesses.clear();
  require(bus.read_memory({.address = location, .size = 8}) == literal &&
              bus.byte_accesses.size() == 8,
          "An eight-byte load retains all bits without shifts overflowing their type");
  constexpr unsigned low_word = 0xcdef;
  bus.write_memory({.address = location, .size = 2, .value = literal});
  require(bus.read_memory({.address = location, .size = 2}) == low_word,
          "Narrow stores retain the low operand bits");
  bus.byte_accesses.clear();
  reject([&bus] { bus.write_memory({.address = word_mask, .size = 2}); },
         "Unsupported address-space-spanning operands fail visibly");
  reject([&bus] { static_cast<void>(bus.read_memory({.size = 0})); }, "Zero-size reads fail");
  reject([&bus] { static_cast<void>(bus.read_memory({.size = 9})); }, "Oversized reads fail");
  require(bus.byte_accesses.empty(), "Invalid operand ranges do not access physical bytes");
  bus.failed_write = location + 1;
  constexpr unsigned interrupted_word = 0x7654;
  reject([&bus] { bus.write_memory({.address = location, .size = 2, .value = interrupted_word}); },
         "Partial byte-store failures propagate");
  constexpr unsigned partial_high = 0x76;
  require(bus.bytes[location] == partial_high && bus.bytes[location + 1] == golden.back(),
          "A second-byte fault retains the first byte already written, without fake rollback");
}

void test_native_operands_preserve_width() {
  WidthBus bus;
  // LDD #$1234; STD $4000; LDD $4000; STAA $4002; LDAA $4003.
  constexpr std::array<std::uint8_t, 15> code{
      0xcc, 0x12, 0x34, 0x7c, 0x40, 0, 0xfc, 0x40, 0, 0x7a, 0x40, 2, 0xb6, 0x40, 3};
  std::ranges::copy(code, bus.bytes.begin() + entry);
  constexpr unsigned loaded_byte = 0x56;
  constexpr unsigned byte_location = 0x4003;
  bus.bytes[byte_location] = loaded_byte;
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  constexpr blofeld::CpuState initial{.pc = entry, .sp = stack, .ccr = 0xd0};
  cpu.set_state(initial);
  for (unsigned index = 0; index < 5; ++index) { cpu.step(); }
  constexpr std::array<Operand, 4> golden{
      {{.write = true, .address = 0x4000, .size = 2, .value = 0x1234},
       {.address = 0x4000, .size = 2},
       {.write = true, .address = 0x4002, .value = 0x12},
       {.address = 0x4003}}};
  require(bus.operands.size() == golden.size(), "Immediate operands cause no data bus transfers");
  for (unsigned index = 0; index < golden.size(); ++index) {
    const auto expected = golden[index];
    const auto actual = bus.operands[index];
    require(actual.write == expected.write && actual.address == expected.address &&
                actual.size == expected.size && (!actual.write || actual.value == expected.value),
            "Native LDD/STD and byte loads/stores expose their original operand widths");
  }
  constexpr unsigned final_accumulator = 0x5634;
  require(cpu.state().d == final_accumulator && cpu.state().pc == entry + code.size(),
          "Width propagation preserves independently expected instruction effects");
}

std::uint32_t configuration(const blofeld::McuUsb &usb) {
  std::uint32_t result = 0;
  for (unsigned index = 0; index < 4; ++index) {
    result = (result << 8U) | usb.read(static_cast<std::uint16_t>(data_high + index));
  }
  return result;
}

void test_configuration_banks_and_staging() {
  blofeld::McuUsb usb;
  require(usb.read(select_high) == 0 && usb.read(select_low) == 0, "Selector reset is zero");
  reject([&usb] { static_cast<void>(configuration(usb)); },
         "Selection zero cannot alias a valid endpoint");
  constexpr std::array<unsigned, 7> selectors{1, 2, 3, 4, 5, 8, 9};
  constexpr std::array<std::uint32_t, 7> values{
      0x02000000, 0x020000b1, 0x020000a2, 0x100000a3, 0x100000b3, 0x00000442, 0x12345678};
  for (unsigned index = 0; index < selectors.size(); ++index) {
    usb.write({.address = select_high, .value = static_cast<std::uint8_t>(selectors[index])});
    require(configuration(usb) == 0,
            "Each endpoint/configuration register has separate reset storage");
    usb.write_memory({.address = data_high, .size = 2, .value = values[index] >> 16U});
    require(configuration(usb) == 0,
            "The upper word stages without committing any of the four bytes");
    usb.write({.address = select_low, .value = byte_mask});
    require(usb.read(select_low) == buffer_mask,
            "Buffer selection masks reserved bits without changing UCRSEL");
    usb.write_memory({.address = data_low, .size = 2, .value = values[index] & word_mask});
    require(configuration(usb) == values[index],
            "Low-word completion commits the entire selected 32-bit value");
  }
  for (unsigned index = 0; index < selectors.size(); ++index) {
    usb.write({.address = select_high,
               .value = static_cast<std::uint8_t>(selectors[index] | selector_reserved_bits)});
    require(usb.read(select_high) == selectors[index] && configuration(usb) == values[index],
            "UCRSEL masks high reserved bits and preserves every distinct register bank");
  }
  usb.reset();
  usb.write({.address = select_high, .value = 9});
  require(configuration(usb) == 0, "External reset clears all configuration and staging state");
  reject([&usb] { usb.write_memory({.address = data_low, .size = 2}); },
         "Reset does not invent a known high-word staging latch");
}

void test_reserved_selections_and_incomplete_writes() {
  blofeld::McuUsb usb;
  constexpr std::array<std::uint8_t, 9> reserved{0, 6, 7, 10, 11, 12, 13, 14, 15};
  for (const auto selector : reserved) {
    usb.write({.address = select_high, .value = selector});
    reject([&usb] { static_cast<void>(configuration(usb)); }, "Reserved configuration reads fail");
    reject([&usb] { usb.write_memory({.address = data_high, .size = 2}); },
           "Reserved configuration writes cannot corrupt another bank");
  }
  usb.write({.address = select_high, .value = 1});
  reject([&usb] { usb.write({.address = data_high, .value = 1}); },
         "Byte writes cannot impersonate word transfers");
  reject([&usb] { usb.write_memory({.address = data_high, .size = 4}); },
         "A single 32-bit operand is not the documented pair of 16-bit operations");
  reject([&usb] { usb.write_memory({.address = data_high + 1, .size = 2}); },
         "Misaligned UUCFGR word writes fail");
  constexpr unsigned first_high = 0xabcd;
  constexpr unsigned second_high = 0x1234;
  usb.write_memory({.address = data_high, .size = 2, .value = first_high});
  reject([&usb] { usb.write({.address = select_high, .value = 2}); },
         "Undefined selection changes between staged words cannot silently choose a bank");
  require(usb.read(select_high) == 1 && configuration(usb) == 0,
          "Rejected selection changes preserve the old bank");
  usb.write_memory({.address = data_high, .size = 2, .value = second_high});
  constexpr unsigned low = 0x5678;
  usb.write_memory({.address = data_low, .size = 2, .value = low});
  constexpr unsigned final_value = 0x12345678;
  require(configuration(usb) == final_value,
          "A newer high word replaces the previous uncommitted latch");
  usb.write({.address = select_high, .value = 8});
  usb.write_memory({.address = data_high, .size = 2, .value = word_mask});
  constexpr unsigned counts_with_reserved_bits = 0x88c0;
  usb.write_memory({.address = data_low, .size = 2, .value = counts_with_reserved_bits});
  constexpr unsigned masked_counts = 0x40;
  require(configuration(usb) == masked_counts, "UNCIR reserved bits remain zero");
  usb.write_memory({.address = data_high, .size = 2});
  reject([&usb] { usb.write_memory({.address = data_low, .size = 2, .value = 3}); },
         "Reserved number-of-configurations value is not a valid USB configuration");
  require(configuration(usb) == masked_counts,
          "Invalid configuration counts do not commit partial state");
}

void test_real_cpu_configures_usb_window() {
  // Own program selects UNCIR, stores 0x00000442 with two STD instructions,
  // loads back each word, then stores a byte to the high data word (must fail).
  constexpr std::array<std::uint8_t, 33> code{0xcc, 0x08, 0x23, 0x7c, 0x03, 0x0e, 0xcc, 0,    0,
                                              0x7c, 0x03, 0x10, 0xcc, 0x04, 0x42, 0x7c, 0x03, 0x12,
                                              0xfc, 0x03, 0x10, 0x7c, 0x12, 0,    0xfc, 0x03, 0x12,
                                              0x7c, 0x12, 2,    0x7a, 0x03, 0x10};
  std::array<std::uint8_t, 32768> flash{};
  std::ranges::copy(code, flash.begin());
  blofeld::McuBus bus(flash);
  reject([&bus] { bus.write(select_high, 8); },
         "Clock-disabled configuration writes are unverified and fail");
  bus.write(clock_control, module_clock);
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  constexpr blofeld::CpuState initial{.pc = entry, .sp = 0x2000, .ccr = 0xd0};
  cpu.set_state(initial);
  constexpr unsigned configuration_instructions = 10;
  for (unsigned index = 0; index < configuration_instructions; ++index) { cpu.step(); }
  constexpr unsigned mailbox = 0x1200;
  constexpr unsigned low_word = 0x442;
  require(
      bus.read_memory({.address = mailbox, .size = 2}) == 0 &&
          bus.read_memory({.address = mailbox + 2, .size = 2}) == low_word &&
          bus.read(select_high) == 8 && bus.read(select_low) == buffer_fixture,
      "Original HC12 operand semantics select, stage, commit and read the USB configuration banks");
  reject([&cpu] { cpu.step(); },
         "A byte store through the native CPU cannot commit a USB word-only register");
  constexpr unsigned initrg = 0x11;
  constexpr unsigned relocated = 0x1000;
  bus.write(initrg, 0x10);
  bus.retire_instruction();
  require(bus.read_memory({.address = relocated + data_low, .size = 2}) == low_word,
          "Register relocation preserves the selected USB configuration value");
  bus.reset();
  bus.write(clock_control, module_clock);
  bus.write(select_high, 8);
  require(bus.read_memory({.address = data_low, .size = 2}) == 0,
          "Board reset clears USB configuration and restores its map");
  constexpr unsigned enable_controller = 0xc0;
  bus.write(clock_control, enable_controller);
  require(bus.read(clock_control) == enable_controller,
          "MEN enables the modeled USB register/data path");
  constexpr unsigned usb_status = 0x302;
  reject([&bus] { static_cast<void>(bus.read(usb_status)); },
         "Controller enable cannot assume unknown USB protocol-decoder inputs");
}

void test_interrupt_and_return_word_transfers() {
  WidthBus bus;
  bus.bytes[vector] = handler >> 8U;
  constexpr unsigned rti = 0x0b;
  bus.bytes[handler] = rti;
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  constexpr blofeld::CpuState initial{
      .pc = entry, .d = 0x1234, .x = 0x5678, .y = 0x9abc, .sp = stack, .ccr = 0x40};
  cpu.set_state(initial);
  bus.request = blofeld::InterruptRequest{.vector = vector};
  cpu.step();
  require(bus.operands.size() == 6 && !bus.operands.front().write && bus.operands.front().size == 2,
          "Interrupt entry fetches one vector word and saves four words plus CCR");
  for (unsigned index = 1; index < 5; ++index) {
    require(bus.operands[index].write && bus.operands[index].size == 2,
            "PC, Y, X and B:A save their CPU word widths");
  }
  require(bus.operands.back().write && bus.operands.back().size == 1, "CCR saves one byte");
  bus.operands.clear();
  cpu.step();
  constexpr std::array<unsigned, 5> sizes{1, 2, 2, 2, 2};
  require(bus.operands.size() == sizes.size(), "RTI restores CCR and four CPU words");
  for (unsigned index = 0; index < sizes.size(); ++index) {
    require(!bus.operands[index].write && bus.operands[index].size == sizes[index],
            "RTI preserves the documented byte/word stack-transfer sizes");
  }
  require(cpu.state().pc == initial.pc && cpu.state().sp == initial.sp &&
              cpu.state().d == initial.d,
          "Grouped transfers retain the independently expected restored registers");
}
}

int main() {
  try {
    test_byte_order_widths_and_failures();
    test_native_operands_preserve_width();
    test_configuration_banks_and_staging();
    test_reserved_selections_and_incomplete_writes();
    test_real_cpu_configures_usb_window();
    test_interrupt_and_return_word_transfers();
    std::cout << "Native memory operand widths and USB configuration checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
