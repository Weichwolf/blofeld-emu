// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/flash_image.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_flash.hpp"
#include "flash_backdoor_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>

namespace blofeld::test::flash_key {
McuFlashImage image(const std::uint8_t options) {
  McuFlashImage result;
  result.bytes.fill(nop); // Own HC12 NOP bytes; no original firmware is embedded.
  result.known.set();
  result.bytes.at(protection_offset) = 0; // All protected; key matching must retain protection.
  result.bytes.at(security_offset) = options;
  for (unsigned i = 0; i < keys.size(); ++i) {
    result.bytes.at(key_offset + (i * 2)) = static_cast<std::uint8_t>(keys.at(i) >> 8U);
    result.bytes.at(key_offset + 1 + (i * 2)) = static_cast<std::uint8_t>(keys.at(i) & erased);
  }
  return result;
}

void put(McuBus &bus, const unsigned index, const unsigned word) {
  bus.write_memory(
      {.address = static_cast<std::uint16_t>(first_key + (index * 2)), .size = 2, .value = word});
}

void feed(McuBus &bus) {
  for (unsigned i = 0; i < keys.size(); ++i) { put(bus, i, keys.at(i)); }
}

void unchanged(const McuBus &bus, const McuFlashImage &before) {
  const auto now = bus.power_off_flash_image();
  require(now.bytes == before.bytes && now.known == before.known && now.fresh == before.fresh,
          "Key comparison cannot modify any flash bytes, knownness or erase history");
}
}

namespace {
using namespace blofeld::test::flash_key;

void test_success_reset_and_all_security_fields() {
  for (unsigned value = keyen_enabled; value < keyen_enabled_end; ++value) {
    const auto before = image(static_cast<std::uint8_t>(value));
    blofeld::McuBus bus(before);
    bus.write(control, key_access);
    feed(bus);
    reject([&] { static_cast<void>(bus.read(security)); }, "after four key matches before KEYACC");
    require(bus.read(control) == key_access,
            "An unqualified pre-close security read retains the complete pending key transaction");
    bus.write(control, 0);
    require(bus.read(security) == ((value & retained_security_mask) | 2U) &&
                bus.read(protection) == 0 && bus.read(status) == idle_status &&
                bus.read(control) == 0,
            "Only SEC becomes10; reserved/KEYEN/protection/command flags remain independent");
    unchanged(bus, before);
    bus.reset();
    require(bus.read(security) == value, "Reset samples the unchanged NVM security byte");
  }
}

void test_permutations_and_key_mismatches() {
  constexpr std::array<unsigned, 4> ordered{0, 1, 2, 3};
  auto order = ordered;
  do {
    const auto before = image();
    blofeld::McuBus bus(before);
    bus.write(control, key_access);
    for (const auto i : order) { put(bus, i, keys.at(i)); }
    bus.write(control, 0);
    require(bus.read(security) == (order == ordered ? unsecured : secured),
            "Only the literal sequential four-word order can unsecure");
    unchanged(bus, before);
  } while (std::ranges::next_permutation(order).found);
  for (unsigned target = 0; target < keys.size(); ++target) {
    for (unsigned bit = 0; bit < 16; ++bit) {
      blofeld::McuBus bus(image());
      bus.write(control, key_access);
      for (unsigned i = 0; i < keys.size(); ++i) {
        put(bus, i, keys.at(i) ^ (i == target ? 1U << bit : 0U));
      }
      bus.write(control, 0);
      require(bus.read(security) == secured, "Every single-bit mismatch locks out the sequence");
      bus.write(control, key_access);
      feed(bus);
      bus.write(control, 0);
      require(bus.read(security) == secured, "A new KEYACC attempt cannot release a mismatch lock");
      bus.reset();
      bus.write(control, key_access);
      feed(bus);
      bus.write(control, 0);
      require(bus.read(security) == unsecured,
              "Only reset releases the security state-machine lock");
    }
  }
}

void test_partial_extra_forbidden_and_power_cuts() {
  const auto before = image();
  for (unsigned count = 0; count <= keys.size(); ++count) {
    blofeld::McuBus bus(before);
    bus.write(control, key_access);
    for (unsigned i = 0; i < count; ++i) { put(bus, i, keys.at(i)); }
    auto interrupted = bus.clone();
    if (count == 0) {
      reject([&] { bus.write(control, 0); }, "empty MCU flash key sequence");
      require(bus.read(security) == secured,
              "An unqualified empty close cannot fabricate an unlock");
      feed(bus); // A rejected empty close cannot consume or lock a comparison.
    }
    const bool matched = count == 0 || count == keys.size();
    bus.write(control, 0);
    require(bus.read(security) == (matched ? unsecured : secured),
            "Clearing KEYACC after only one to three matching words cannot unsecure");
    bus.write(control, key_access);
    feed(bus);
    bus.write(control, 0);
    require(bus.read(security) == (matched ? unsecured : secured),
            "Partial completion remains locked until reset");
    interrupted->power_cycle();
    require(interrupted->read(security) == secured && interrupted->read(control) == 0,
            "Every pre-clear power cut retains secure NVM and discards partial key access");
    interrupted->write(control, key_access);
    feed(*interrupted);
    interrupted->write(control, 0);
    require(interrupted->read(security) == unsecured, "A new power epoch accepts a correct key");
    unchanged(*interrupted, before);
    interrupted->power_cycle();
    require(interrupted->read(security) == secured,
            "Even completed temporary unlock is not durable");
  }
  blofeld::McuBus extra(before);
  extra.write(control, key_access);
  feed(extra);
  put(extra, 0, keys.front());
  extra.write(control, 0);
  require(extra.read(security) == secured,
          "A fifth word invalidates a complete pending comparison");
  for (unsigned target = 0; target < keys.size(); ++target) {
    for (const unsigned forbidden : {0U, word_mask}) {
      auto forbidden_image = before;
      forbidden_image.bytes.at(key_offset + (target * 2)) =
          static_cast<std::uint8_t>(forbidden >> 8U);
      forbidden_image.bytes.at(key_offset + 1 + (target * 2)) =
          static_cast<std::uint8_t>(forbidden & erased);
      blofeld::McuBus bus(forbidden_image);
      bus.write(control, key_access);
      for (unsigned i = 0; i < keys.size(); ++i) {
        put(bus, i, i == target ? forbidden : keys.at(i));
      }
      bus.write(control, 0);
      require(bus.read(security) == secured,
              "Zero/FFFF cannot unlock even when stored and matched");
      unchanged(bus, forbidden_image);
    }
  }
}

void test_command_ownership_and_repeated_enable() {
  auto before = image();
  before.bytes.at(protection_offset) = erased;
  before.bytes.at(test_offset) = erased;
  before.bytes.at(test_offset + 1) = erased;
  before.fresh.set(test_offset / 2);
  blofeld::McuBus bus(before);
  bus.observe_flash_clock({.oscillator_hz = oscillator_hz, .bus_hz = bus_hz});
  bus.write(divider, divider_value);
  bus.write_memory({.address = test_target, .size = 2, .value = test_data});
  reject([&] { bus.write(control, key_access); }, "staged or executing command");
  bus.write(command, word_program);
  reject([&] { bus.write(control, key_access); }, "staged or executing command");
  bus.write(status, buffer_empty);
  reject([&] { bus.write(control, key_access); }, "staged or executing command");
  bus.advance_flash(blofeld::FlashMilestone::transfer_buffer);
  reject([&] { bus.write(control, key_access); }, "staged or executing command");
  bus.advance_flash(blofeld::FlashMilestone::complete_command);
  require(bus.read_memory({.address = test_target, .size = 2}) == test_data &&
              bus.read(status) == idle_status && bus.read(control) == 0,
          "Unqualified key entry leaves every real command owner and result intact");
  bus.write(control, key_access);
  for (unsigned i = 0; i < keys.size(); ++i) {
    put(bus, i, keys.at(i));
    bus.write(control, key_access);
  }
  bus.write(control, 0);
  require(bus.read(security) == unsecured,
          "Rewriting an already set KEYACC does not restart a partly matched key");
  blofeld::McuBus protected_bus(image());
  protected_bus.write(control, key_access);
  feed(protected_bus);
  protected_bus.write(control, 0);
  protected_bus.write(divider, divider_value);
  protected_bus.write_memory({.address = test_target, .size = 2, .value = test_data});
  require(protected_bus.read(status) == protected_status &&
              protected_bus.read(security) == unsecured,
          "A valid temporary unlock cannot bypass physical flash write protection");
  unchanged(protected_bus, image());
}

void test_unknown_keys_read_guards_and_unqualified_interleavings() {
  auto before = image();
  before.known.reset(key_offset + 3);
  before.bytes.at(key_offset + 3) = 0;
  blofeld::McuBus unknown(before);
  unknown.write(control, key_access);
  put(unknown, 0, keys.front());
  reject([&] { put(unknown, 1, keys.at(1)); }, "Unknown physical MCU flash byte");
  unchanged(unknown, before);
  blofeld::McuBus bus(image());
  bus.begin_instruction({.pc = diagnostic_pc, .ccr = masked_flags});
  bus.write(control, all_control);
  require(bus.read(control) == all_control,
          "KEYACC retains both independently writable interrupt enables");
  reject([&] { static_cast<void>(bus.read(first_key)); }, "pc=0x1802 ccr=0xd0");
  reject([&] { static_cast<void>(bus.read(flash_base)); }, "KEYACC is set");
  reject([&] { bus.write_memory({.address = first_key, .size = 1, .value = first_key_high_byte}); },
         "outside aligned key words");
  reject([&] { put(bus, 4, 1); }, "outside aligned key words");
  reject([&] { bus.write(command, word_program); }, "register write during key access");
  feed(bus);
  bus.write(control, interrupt_enable);
  require(bus.read(security) == unsecured && bus.read(control) == interrupt_enable,
          "Rejected unsupported operations do not fabricate or consume a comparison");
  for (const unsigned value : {0U, 0x40U, 0xc0U}) {
    blofeld::McuBus disabled(image(static_cast<std::uint8_t>(value)));
    disabled.write(control, key_access);
    reject([&] { put(disabled, 0, keys.front()); }, "KEYEN disabled");
    require(disabled.read(security) == value, "Unsupported disabled-KEYEN writes cannot unsecure");
  }
}
}

int main() {
  try {
    test_success_reset_and_all_security_fields();
    test_permutations_and_key_mismatches();
    test_partial_extra_forbidden_and_power_cuts();
    test_unknown_keys_read_guards_and_unqualified_interleavings();
    test_command_ownership_and_repeated_enable();
    blofeld::test::flash_key::test_native_keys_and_partial_owner_restore();
    std::cout << "Nominal native key-access, security locks, reset/power cuts and joint replay "
                 "passed; no BDM, original update/recovery or hardware safety proof.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
