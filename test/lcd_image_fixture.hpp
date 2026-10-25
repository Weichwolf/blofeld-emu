// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "display/st7565.hpp"

#include <cstdint>
#include <string_view>

namespace blofeld::test {
constexpr unsigned lcd_page_command = 0xb0;
constexpr unsigned lcd_display_on = 0xaf;
constexpr unsigned lcd_display_off = 0xae;
constexpr unsigned lcd_adc_reverse = 0xa1;
constexpr unsigned lcd_common_reverse = 0xc8;
constexpr unsigned lcd_reverse_on = 0xa7;
constexpr unsigned lcd_all_points = 0xa5;
constexpr unsigned lcd_rmw = 0xe0;
constexpr unsigned lcd_end = 0xee;
constexpr unsigned lcd_byte_mask = 255;

inline void require(const bool condition, const char *message) {
  if (!condition) { throw Fault(message); }
}

inline void reject(const auto &operation, const std::string_view reason) {
  try {
    operation();
  } catch (const Fault &fault) {
    require(std::string_view(fault.what()).contains(reason), "LCD failure must identify its cause");
    return;
  }
  throw Fault("Expected an explicit LCD failure");
}

struct LcdAddress {
  unsigned page;
  unsigned column;
};

inline void select(St7565 &lcd, const LcdAddress address) {
  lcd.command(static_cast<std::uint8_t>(lcd_page_command | address.page));
  lcd.command(static_cast<std::uint8_t>(16U | (address.column >> 4U)));
  lcd.command(static_cast<std::uint8_t>(address.column & 15U));
}

inline St7565 blank() {
  St7565 lcd;
  lcd.hardware_reset();
  for (unsigned page = 0; page < 8; ++page) {
    select(lcd, {.page = page, .column = 0});
    for (unsigned column = 0; column < St7565::columns; ++column) { lcd.write_data(0); }
  }
  lcd.command(lcd_display_on);
  return lcd;
}
}
