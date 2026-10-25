// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/mcu_bus.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string_view>

namespace {
constexpr unsigned control = 0x300;
constexpr unsigned out_control = 0x31c;
constexpr unsigned in_control = 0x320;
constexpr unsigned select = 0x30e;
constexpr unsigned data = 0x310;
constexpr unsigned high_mask = 0xff00;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned word_mask = 0xffff;
constexpr unsigned normal_fields = 0xdfff;
constexpr unsigned baseline = 0xd357;
constexpr unsigned enabled = 0xc0;
constexpr unsigned clock_enabled = 0x40;
constexpr unsigned full_speed = 8;
constexpr unsigned phy_edges = 15;
constexpr unsigned reference_pc = 0x9123;
constexpr unsigned reference_ccr = 0xd5;
constexpr unsigned size_high = 0x0200;
constexpr unsigned bulk_configuration = 0xc1;
constexpr unsigned flash_bytes = 32768;

void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void put(blofeld::McuBus &bus, const blofeld::MemoryWrite value) {
  bus.write_memory(value);
}

unsigned get(blofeld::McuBus &bus, const unsigned at) {
  return static_cast<unsigned>(
      bus.read_memory({.address = static_cast<std::uint16_t>(at), .size = 2}));
}

void prepare(blofeld::McuBus &bus) {
  bus.write(control, clock_enabled);
  bus.write(control + 1, full_speed);
  bus.observe_phy({.edges = phy_edges});
  bus.observe_usb_decoder({});
  bus.write(control, enabled);
}

void test_words(blofeld::McuBus &bus) {
  for (const auto at : {out_control, in_control}) {
    require(get(bus, at) == 0, "Both unconfigured IQUE A controls reset to zero");
    for (unsigned value = 0; value <= word_mask; ++value) {
      put(bus, {.address = static_cast<std::uint16_t>(at), .size = 2, .value = value});
      require(get(bus, at) == (value & normal_fields),
              "All 16-bit writes retain STALL/SNAK/TCIE/CTERR/size without creating TFRC");
    }
    put(bus, {.address = static_cast<std::uint16_t>(at), .size = 2, .value = 0});
  }
}

void test_bytes(blofeld::McuBus &bus) {
  for (const auto at : {out_control, in_control}) {
    for (unsigned value = 0; value <= byte_mask; ++value) {
      put(bus, {.address = static_cast<std::uint16_t>(at), .size = 2, .value = baseline});
      bus.write(static_cast<std::uint16_t>(at), static_cast<std::uint8_t>(value));
      const auto expected_high = (value << 8U) & normal_fields;
      require(get(bus, at) == (expected_high | (baseline & byte_mask)),
              "High-byte updates preserve all eight low size bits");
      put(bus, {.address = static_cast<std::uint16_t>(at), .size = 2, .value = baseline});
      bus.write(static_cast<std::uint16_t>(at + 1), static_cast<std::uint8_t>(value));
      require(get(bus, at) == ((baseline & high_mask) | value),
              "Low-byte size updates preserve every high-byte control field");
    }
  }
}

void test_copy_and_resets(blofeld::McuBus &bus) {
  put(bus, {.address = out_control, .size = 2, .value = baseline});
  put(bus, {.address = in_control, .size = 2, .value = normal_fields});
  auto copy = bus.clone();
  put(bus, {.address = out_control, .size = 2, .value = 0});
  require(get(*copy, out_control) == baseline && get(*copy, in_control) == normal_fields,
          "Owner copies independently retain both IQUE control words");
  bus.observe_usb_decoder({.reset = true});
  require(get(bus, out_control) == 0 && get(bus, in_control) == 0,
          "A decoded bus reset clears both endpoint controls and sizes");
  copy->reset();
  require(get(*copy, out_control) == 0 && get(*copy, in_control) == 0,
          "External MCU reset clears both IQUE control words");
  bus.observe_usb_decoder({});
}

void reject_access(const auto &operation, const std::string_view reason) {
  bool rejected = false;
  try {
    operation();
  } catch (const blofeld::Fault &fault) {
    const std::string_view text(fault.what());
    require(text.contains(reason) && text.contains("pc=0x9123") && text.contains("ccr=0xd5"),
            "Unsupported IQUE accesses retain operation/address and native context");
    rejected = true;
  }
  require(rejected, "Unsupported IQUE state must fail, never report an empty successful queue");
}

void test_unsupported(blofeld::McuBus &bus) {
  bus.begin_instruction({.pc = reference_pc, .ccr = reference_ccr});
  constexpr std::array<unsigned, 4> b_registers{0x31e, 0x31f, 0x322, 0x323};
  for (const auto at : b_registers) {
    reject_access([&] { static_cast<void>(bus.read(static_cast<std::uint16_t>(at))); },
                  "Unimplemented MCU peripheral read");
    reject_access([&] { bus.write(static_cast<std::uint16_t>(at), 0); },
                  "Unimplemented MCU peripheral write");
  }
  for (const auto bank : {4U, 5U}) {
    const auto at = bank == 4 ? out_control : in_control;
    put(bus, {.address = static_cast<std::uint16_t>(at), .size = 2, .value = baseline});
    bus.write(select, static_cast<std::uint8_t>(bank));
    put(bus, {.address = data, .size = 2, .value = size_high});
    put(bus, {.address = data + 2, .size = 2, .value = bulk_configuration});
    reject_access([&] { static_cast<void>(get(bus, at)); }, "Unimplemented configured USB IQUE");
    reject_access([&] { bus.write(static_cast<std::uint16_t>(at), 0); },
                  "Unimplemented configured USB IQUE");
    put(bus, {.address = data, .size = 2, .value = 0});
    put(bus, {.address = data + 2, .size = 2, .value = 0});
    require(get(bus, at) == baseline, "Rejected configured accesses preserve the control word");
  }
  constexpr unsigned local_control = 0x314;
  require(get(bus, local_control) == 0, "IQUE controls do not alias the local endpoint registers");
}

void run() {
  const std::array<std::uint8_t, flash_bytes> flash{};
  blofeld::McuBus bus(flash);
  prepare(bus);
  test_words(bus);
  test_bytes(bus);
  test_copy_and_resets(bus);
  test_unsupported(bus);
  std::cout << "Verified 131072 full-word writes, byte independence, owner copies/reset and "
               "strict unconfigured-ONLY IQUE controls. No IQUE data/count progress or IRQ "
               "completion is fabricated.\n";
}
}

int main() {
  try {
    run();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
