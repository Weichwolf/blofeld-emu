// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_flash_config.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace {
// Independent literal expectations from S12FTS32KV1 V01.03 sections 3.3.1/2/5,
// tables 3-4..3-9 and UF32 device integration section 10.1.
constexpr unsigned divider = 0x100;
constexpr unsigned security = 0x101;
constexpr unsigned protection = 0x104;
constexpr unsigned byte_values = 256;
constexpr unsigned flash_bytes = 32768;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned options_offset = 0x7f0d;
constexpr unsigned security_offset = 0x7f0f;
constexpr unsigned unsupported_status = 0x105;
constexpr unsigned unsupported_command = 0x106;
constexpr unsigned remap = 0x11;
constexpr unsigned size_fields = 0x1b;
constexpr unsigned helper_divider = 0x48;
constexpr unsigned loaded_helper_divider = 0xc8;
constexpr unsigned options_address = 0xff0d;
constexpr unsigned security_address = 0xff0f;
constexpr unsigned application_address = 0x8200;

void require(const bool value, const char *message) {
  if (!value) { throw blofeld::Fault(message); }
}

void reject(const auto &operation, const char *message) {
  try {
    operation();
  } catch (const blofeld::Fault &) { return; }
  throw blofeld::Fault(message);
}

std::uint8_t byte(const unsigned value) {
  return static_cast<std::uint8_t>(value & byte_mask);
}

void test_divider_and_security() {
  for (unsigned value = 0; value < byte_values; ++value) {
    blofeld::McuFlashConfig config({.protection = byte(value), .security = byte(value)});
    require(config.read(divider) == 0 && !config.oscillator_divisor(),
            "Reset divider is unwritten, not an invented valid flash clock");
    require(config.read(security) == value && config.read(protection) == value,
            "Every nonvolatile option bit is sampled exactly on reset");
    require(config.secured() == (value % 4 != 2), "Only SEC=10 is unsecured");
    require(config.backdoor_enabled() == (value / 64 == 2),
            "Only KEYEN=10 enables backdoor access");
    config.write({.address = security, .value = byte(~value)});
    require(config.read(security) == value, "FSEC writes cannot alter the reset security state");
    config.write({.address = divider, .value = byte(value)});
    const auto expected_divisor = ((value / 64) % 2 != 0 ? 8U : 1U) * ((value % 64) + 1);
    require(config.read(divider) == (value | 128U) &&
                config.oscillator_divisor() == expected_divisor,
            "First FCLKDIV write latches FDIVLD, PRDIV8 and divide-by-FDIV-plus-one");
    config.write({.address = divider, .value = byte(~value)});
    require(config.read(divider) == (value | 128U), "Divider is write-once until reset");
    config.reset({.protection = byte(~value), .security = byte(~value)});
    require(config.read(divider) == 0 && !config.oscillator_divisor() &&
                config.read(security) == byte(~value) && config.read(protection) == byte(~value),
            "Reset reloads nonvolatile fields and releases the divider write lock");
  }
}

constexpr std::array<unsigned, 8> scenarios{0, 4, 32, 36, 128, 132, 160, 164};
// Row masks transcribed from table 3-9; bit n permits destination scenario n.
constexpr std::array<unsigned, 8> transitions{0x0f, 0x0a, 0x0c, 0x08, 0x18, 0x3c, 0x5a, 0xff};

void test_scenario_transitions() {
  for (unsigned from = 0; from < scenarios.size(); ++from) {
    for (unsigned to = 0; to < scenarios.size(); ++to) {
      blofeld::McuFlashConfig config({.protection = byte(scenarios[from])});
      config.write({.address = protection, .value = byte(scenarios[to])});
      const bool allowed = (transitions[from] & (1U << to)) != 0;
      require(config.read(protection) == (allowed ? scenarios[to] : scenarios[from]),
              "All 64 normal-mode protection scenario transitions match the independent table");
    }
  }
}

unsigned scenario_index(const unsigned value) {
  return ((value / 128) * 4) + (((value / 32) % 2) * 2) + ((value / 4) % 2);
}

void test_all_protection_writes() {
  constexpr unsigned disabled_fields = 0x24;
  constexpr unsigned high_fields = 0x18;
  for (unsigned original = 0; original < byte_values; ++original) {
    for (unsigned requested = 0; requested < byte_values; ++requested) {
      blofeld::McuFlashConfig config({.protection = byte(original)});
      config.write({.address = protection, .value = byte(requested)});
      const auto high = (original & 32U) == 0 ? original & high_fields : requested & high_fields;
      const auto low = (original & 4U) == 0 ? original % 4 : requested % 4;
      const bool allowed =
          (transitions[scenario_index(original)] & (1U << scenario_index(requested))) != 0;
      const auto expected = allowed ? (requested & ~size_fields) | high | low : original;
      require(config.read(protection) == expected,
              "All 65536 full-byte writes match the independent scenario table and size locks");
    }
  }
  blofeld::McuFlashConfig config({.protection = disabled_fields});
  config.write({.address = protection, .value = 0});
  config.write({.address = protection, .value = disabled_fields});
  require(config.read(protection) == disabled_fields,
          "An invalid scenario ignores the entire write rather than partly locking its fields");
  config.reset({.protection = byte_mask});
  constexpr unsigned enable_both_with_sizes = 0xdb;
  config.write({.address = protection, .value = enable_both_with_sizes});
  require(config.read(protection) == enable_both_with_sizes,
          "Size and enable bits can be selected together on the first protecting write");
}

void test_protected_ranges() {
  constexpr std::array<unsigned, 4> low_ends{512, 1024, 2048, 4096};
  constexpr std::array<unsigned, 4> high_starts{30720, 28672, 24576, 16384};
  for (unsigned value = 0; value < byte_values; ++value) {
    const blofeld::McuFlashConfig config({.protection = byte(value)});
    for (unsigned offset = 0; offset < flash_bytes; ++offset) {
      const bool low = (value / 4) % 2 == 0 && offset < low_ends[value % 4];
      const bool high = (value / 32) % 2 == 0 && offset >= high_starts[(value / 8) % 4];
      const bool expected = value / 128 != 0 ? low || high : !low && !high;
      require(config.protected_byte(static_cast<std::uint16_t>(offset)) == expected,
              "Every byte in every protection encoding matches independent literal window bounds");
    }
    require(config.whole_array_unprotected() ==
                (value / 128 == 1 && (value / 32) % 2 == 1 && (value / 4) % 2 == 1),
            "Whole-array unprotected requires FPOPEN, FPHDIS and FPLDIS all set");
    reject([&config] { static_cast<void>(config.protected_byte(flash_bytes)); },
           "Queries outside the physical 32 KiB array cannot alias a protected byte");
  }
}

void test_bus_reset_remap_and_unsupported_operations() {
  std::array<std::uint8_t, flash_bytes> image{};
  constexpr unsigned original_protection = 0xff;
  constexpr unsigned original_security = 0xbe;
  image[options_offset] = original_protection;
  image[security_offset] = original_security;
  blofeld::McuBus bus(image);
  require(bus.read(protection) == original_protection && bus.read(security) == original_security,
          "MCU bus constructor samples its own immutable flash option bytes");
  bus.write(protection, 0);
  bus.write(divider, helper_divider);
  require(bus.read(protection) == 0 && bus.read(divider) == loaded_helper_divider,
          "Normal bus accesses reach the configuration model");
  require(bus.read(options_address) == original_protection &&
              bus.read(security_address) == original_security,
          "Volatile configuration changes do not program nonvolatile flash fields");
  constexpr unsigned new_base = 0x4000;
  bus.write(remap, new_base >> 8U);
  bus.retire_instruction();
  require(bus.read(new_base + protection) == 0 &&
              bus.read(new_base + divider) == loaded_helper_divider,
          "Register relocation preserves configuration state at the new base");
  reject([&bus] { static_cast<void>(bus.read(new_base + unsupported_status)); },
         "Configuration support does not fabricate command-ready flags");
  reject([&bus] { bus.write(new_base + unsupported_command, 0x20); },
         "Configuration support cannot silently accept a programming command");
  reject([&bus] { bus.write(application_address, 0); },
         "Flash array writes still require a command-state model");
  bus.reset();
  require(bus.read(protection) == original_protection && bus.read(security) == original_security &&
              bus.read(divider) == 0,
          "External reset reloads flash options, resets mapping and clears the divider lock");
}
}

int main() {
  try {
    test_divider_and_security();
    test_scenario_transitions();
    test_all_protection_writes();
    test_protected_ranges();
    test_bus_reset_remap_and_unsupported_operations();
    std::cout << "FTS32K reset/security/divider and protection checks passed, including all "
              << "64 scenario transitions and 8388608 byte/range combinations; "
              << "no program/erase/recovery or physical security proof.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
