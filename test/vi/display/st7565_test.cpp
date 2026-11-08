// SPDX-License-Identifier: GPL-3.0-only
#include "display/st7565.hpp"

#include <exception>
#include <iostream>
#include <stdexcept>

namespace {
void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}
}

int main() try {
  blofeld::vi::St7565 lcd;
  bool rejected = false;
  try {
    lcd.write_data(0);
  } catch (const std::runtime_error &) { rejected = true; }
  require(rejected, "Access requires reset");
  lcd.hardware_reset();
  require(!lcd.state().display_on && lcd.ram(0, 0).known == 0, "Reset does not invent RAM");
  constexpr unsigned page_two = 0xb2;
  constexpr unsigned column_high = 0x10;
  constexpr unsigned column_low = 0x03;
  constexpr unsigned pattern = 0xa5;
  constexpr unsigned full_byte = 0xff;
  lcd.command(page_two);
  lcd.command(column_high);
  lcd.command(column_low);
  lcd.write_data(pattern);
  require(lcd.ram(2, 3).value == pattern && lcd.ram(2, 3).known == full_byte, "RAM write");
  require(lcd.state().column == 4, "Writes advance the column");
  lcd.command(column_low);
  require(lcd.read_data().value == column_low, "Dummy read returns the command holder");
  require(lcd.read_data().value == pattern && lcd.state().column == 4, "RAM data follows dummy");
  constexpr unsigned icon_page = 0xb8;
  lcd.command(icon_page);
  lcd.command(0);
  lcd.write_data(full_byte);
  require(lcd.ram(8, 0).value == 1 && lcd.ram(8, 0).known == 1, "Icon page has one bit");
  constexpr unsigned display_on = 0xaf;
  lcd.command(display_on);
  require(lcd.state().display_on, "Display mode follows commands");
  lcd.hardware_reset();
  require(!lcd.state().display_on && lcd.ram(2, 3).value == pattern, "Reset preserves RAM");
} catch (const std::exception &error) {

  std::cerr << error.what() << "\n";
  return 1;
}
