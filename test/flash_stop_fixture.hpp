// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_flash.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace blofeld::test::flash_stop {
inline constexpr unsigned array_bytes = 32768, base = 0x8000, sector_bytes = 512;
inline constexpr unsigned target = 0x8434, sector = 0x8400, successor = 0x9002;
inline constexpr unsigned status = 0x105, command = 0x106, configuration = 0x103;
inline constexpr unsigned program = 0x20, erase = 0x40, mass = 0x41, verify = 5;
inline constexpr unsigned idle = 0xc0, launch_bit = 0x80, access_error = 0x10;
inline constexpr unsigned word_data = 0x1234, next_data = 0x5678, byte_mask = 0xff;
inline constexpr unsigned pattern = 0xa5, unsecured = 0xfe, divider_value = 0x48;
inline constexpr unsigned oscillator_hz = 12000000, bus_hz = 24000000;
inline constexpr auto nominal = FlashStopContract::nominal_active_abort;

inline void require(const bool condition, const char *message) {
  if (!condition) { throw Fault(message); }
}

inline std::string rejected(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected flash STOP qualification/data fault");
}

inline McuFlashImage image() {
  std::array<std::uint8_t, array_bytes> data{};
  data.fill(pattern);
  data[hc12::flash_protection_field - base] = byte_mask;
  data[hc12::flash_security_field - base] = unsecured;
  return McuFlashImage::from_known(data);
}

inline void configure(McuBus &bus) {
  bus.write(hc12::flash_clock_divider, divider_value);
  bus.observe_flash_clock({.oscillator_hz = oscillator_hz, .bus_hz = bus_hz});
}

struct Command {
  unsigned opcode = program;
  unsigned address = target;
  unsigned value = word_data;
};

inline void launch(McuBus &bus, const Command request = {}) {
  bus.write_memory(
      {.address = static_cast<std::uint16_t>(request.address), .size = 2, .value = request.value});
  bus.write(command, static_cast<std::uint8_t>(request.opcode));
  bus.write(status, launch_bit);
}

inline void finish(McuBus &bus) {
  bus.advance_flash(FlashMilestone::transfer_buffer);
  bus.advance_flash(FlashMilestone::complete_command);
}

inline void erase_sector(McuBus &bus, const unsigned address) {
  launch(bus, {.opcode = erase, .address = address});
  finish(bus);
}

inline void enter(McuBus &bus) {
  bus.observe_flash_stop(FlashStopTransition::enter, nominal);
}

inline void resume(McuBus &bus) {
  bus.observe_flash_stop(FlashStopTransition::resume, nominal);
}

inline void same_image(const McuFlashImage &a, const McuFlashImage &b) {
  require(a.bytes == b.bytes && a.known == b.known && a.fresh == b.fresh,
          "Every NVM byte, known bit and erase-history bit retains its independent expectation");
}
}
