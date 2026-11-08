// SPDX-License-Identifier: GPL-3.0-only
#include "display/lcd_image.hpp"
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
  using blofeld::vi::LcdPixel;
  blofeld::vi::St7565 lcd;
  lcd.hardware_reset();
  constexpr blofeld::vi::LcdPanel panel{.width = 128, .first_segment = 4};
  require(capture_lcd(lcd, panel).pixel(0, 0) == LcdPixel::off, "Display off masks unknown RAM");
  constexpr unsigned display_on = 0xaf;
  lcd.command(display_on);
  require(capture_lcd(lcd, panel).pixel(0, 0) == LcdPixel::unknown, "Unknown RAM is explicit");
  constexpr unsigned page_zero = 0xb0;
  constexpr unsigned first_column = 0x04;
  lcd.command(page_zero);
  lcd.command(first_column);
  constexpr unsigned pattern = 0x81;
  lcd.write_data(pattern);
  const auto image = capture_lcd(lcd, panel);
  require(image.pixel(0, 0) == LcdPixel::on && image.pixel(0, 7) == LcdPixel::on,
          "RAM byte maps to vertical page bits");
  require(image.pixel(0, 1) == LcdPixel::off && image.pixel(1, 0) == LcdPixel::unknown,
          "Capture preserves blank and unknown neighbors");
  lcd.command(first_column);
  lcd.write_data(0);
  require(image.pixel(0, 0) == LcdPixel::on, "Captured images own their pixels");
  constexpr unsigned all_points = 0xa5;
  lcd.command(all_points);
  const auto forced = capture_lcd(lcd, panel);
  constexpr unsigned final_column = 127;
  constexpr unsigned final_row = 63;
  require(forced.pixel(0, 0) == LcdPixel::on &&
              forced.pixel(final_column, final_row) == LcdPixel::on,
          "All-points mode covers the visible panel");
  bool rejected = false;
  try {
    static_cast<void>(capture_lcd(lcd, {.width = 128, .first_segment = 5}));
  } catch (const std::runtime_error &) { rejected = true; }
  require(rejected, "Invalid panel ranges fail explicitly");
} catch (const std::exception &error) {

  std::cerr << error.what() << "\n";
  return 1;
}
