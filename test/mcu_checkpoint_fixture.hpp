// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <span>

namespace blofeld::test {
inline constexpr unsigned entry = 0x8000;
inline constexpr unsigned handler = 0x9000;
inline constexpr unsigned stack = 0x2000;
inline constexpr unsigned ram = 0x1200;
inline constexpr unsigned irq_vector = 0xfff2;
inline constexpr unsigned xirq_vector = 0xfff4;
inline constexpr unsigned word_mask = 0xffff;
inline constexpr unsigned byte_mask = 0xff;

inline void require(const bool condition, const char *message) {
  if (!condition) { throw Fault(message); }
}

inline void reject(const auto &operation, const char *message) {
  try {
    operation();
  } catch (const std::exception &) { return; }
  throw Fault(message);
}

inline McuFlashImage checkpoint_image(const std::span<const std::uint8_t> code = {}) {
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  constexpr unsigned nop = 0xa7;
  constexpr unsigned return_interrupt = 0x0b;
  constexpr unsigned protection = 0x7f0d;
  constexpr unsigned security = 0x7f0f;
  constexpr unsigned unsecured = 0xfe;
  bytes.fill(nop);
  for (unsigned index = 0; index < code.size(); ++index) { bytes.at(index) = code[index]; }
  bytes[handler - entry] = return_interrupt;
  for (const unsigned vector : {irq_vector, xirq_vector}) {
    bytes[vector - entry] = handler >> 8U;
    bytes[vector - entry + 1] = 0;
  }
  bytes[word_mask - entry - 1] = entry >> 8U;
  bytes[word_mask - entry] = 0;
  bytes[protection] = byte_mask;
  bytes[security] = unsecured;
  return McuFlashImage::from_known(bytes);
}

inline void drive_interrupts(McuBus &bus) {
  bus.set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
}

void test_checkpoint_peripherals();
void test_checkpoint_flash();
}
