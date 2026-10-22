// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/lcd_image.hpp"
#include "blofeld/st7565.hpp"
#include "lcd_image_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <string>
#include <vector>

namespace {
using blofeld::test::reject;
using blofeld::test::require;
using blofeld::test::select;
constexpr unsigned last_segment = 131;
constexpr unsigned last_visible_column = 127;

struct Point {
  unsigned x;
  unsigned y;
};

void check(const blofeld::LcdImage &image, const std::span<const Point> lit) {
  for (unsigned y = 0; y < blofeld::LcdImage::height; ++y) {
    for (unsigned x = 0; x < image.width(); ++x) {
      const bool on =
          std::ranges::any_of(lit, [&](const Point point) { return point.x == x && point.y == y; });
      require(image.pixel(x, y) == (on ? blofeld::LcdPixel::on : blofeld::LcdPixel::off),
              "Every pixel must match the independent literal-coordinate expectation");
    }
  }
}

blofeld::St7565 pattern() {
  auto lcd = blofeld::test::blank();
  select(lcd, {.page = 0, .column = 0});
  lcd.write_data(1);
  constexpr unsigned edge_pattern = 0x80;
  select(lcd, {.page = 7, .column = last_visible_column});
  lcd.write_data(edge_pattern);
  constexpr unsigned middle_pattern = 0x24;
  select(lcd, {.page = 3, .column = 9});
  lcd.write_data(middle_pattern);
  return lcd;
}

void test_mapping() {
  auto lcd = pattern();
  constexpr std::array<Point, 4> normal{
      {{.x = 0, .y = 0}, {.x = 127, .y = 63}, {.x = 9, .y = 26}, {.x = 9, .y = 29}}};
  check(capture_lcd(lcd, {.width = 128}), normal);
  lcd.command(blofeld::test::lcd_adc_reverse);
  constexpr std::array<Point, 4> adc{
      {{.x = 131, .y = 0}, {.x = 4, .y = 63}, {.x = 122, .y = 26}, {.x = 122, .y = 29}}};
  check(capture_lcd(lcd, {.width = blofeld::St7565::columns}), adc);
  lcd = pattern();
  lcd.command(blofeld::test::lcd_common_reverse);
  constexpr std::array<Point, 4> com{
      {{.x = 0, .y = 63}, {.x = 127, .y = 0}, {.x = 9, .y = 37}, {.x = 9, .y = 34}}};
  check(capture_lcd(lcd, {.width = 128}), com);
  check(capture_lcd(lcd, {.width = 128, .reverse_commons = true}), normal);
  lcd = pattern();
  constexpr unsigned start_line_four = 0x44;
  lcd.command(start_line_four);
  constexpr std::array<Point, 4> scrolled{
      {{.x = 0, .y = 60}, {.x = 127, .y = 59}, {.x = 9, .y = 22}, {.x = 9, .y = 25}}};
  check(capture_lcd(lcd, {.width = 128}), scrolled);
  constexpr std::array<Point, 4> flipped{
      {{.x = 0, .y = 3}, {.x = 127, .y = 4}, {.x = 9, .y = 41}, {.x = 9, .y = 38}}};
  check(capture_lcd(lcd, {.width = 128, .reverse_commons = true}), flipped);
  constexpr std::array<Point, 2> crop{{{.x = 0, .y = 22}, {.x = 0, .y = 25}}};
  check(capture_lcd(lcd, {.width = 1, .first_segment = 9}), crop);
  lcd = pattern();
  constexpr std::array<Point, 4> wired{
      {{.x = 127, .y = 0}, {.x = 0, .y = 63}, {.x = 118, .y = 26}, {.x = 118, .y = 29}}};
  check(capture_lcd(lcd,
                    {.width = 128, .first_segment = last_visible_column, .reverse_segments = true}),
        wired);
}

void test_unknown_and_modes() {
  blofeld::St7565 lcd;
  reject([&] { static_cast<void>(capture_lcd(lcd, {.width = 128})); }, "observed hardware reset");
  lcd.hardware_reset();
  check(capture_lcd(lcd, {.width = 128}), {});
  lcd.command(blofeld::test::lcd_display_on);
  const auto unknown = capture_lcd(lcd, {.width = 128});
  std::string expected;
  for (unsigned row = 0; row < 64; ++row) { expected += std::string(128, '?') + '\n'; }
  require(encode_lcd_text(unknown) == expected, "Text preserves unknown pixels explicitly");
  reject([&] { static_cast<void>(encode_lcd_pbm(unknown)); },
         "Unknown LCD screenshot pixel x=0 y=0");
  reject([&] { static_cast<void>(encode_lcd_png(unknown)); }, "Unknown LCD screenshot pixel");
  lcd.command(blofeld::test::lcd_reverse_on);
  lcd.command(blofeld::test::lcd_all_points);
  const auto forced = capture_lcd(lcd, {.width = 128});
  for (unsigned y = 0; y < 64; ++y) {
    for (unsigned x = 0; x < 128; ++x) {
      require(forced.pixel(x, y) == blofeld::LcdPixel::on,
              "All-points ON overrides reverse without relying on unknown RAM");
    }
  }
  require(lcd.ram(0, 0).known == 0, "Display modes never establish RAM contents");
  lcd.command(blofeld::test::lcd_display_off);
  check(capture_lcd(lcd, {.width = 128}), {});
  lcd = blofeld::test::blank();
  lcd.command(blofeld::test::lcd_reverse_on);
  require(capture_lcd(lcd, {.width = 128}) == forced, "Reverse inverts a known blank frame");
}

void test_read_only() {
  auto lcd = pattern();
  select(lcd, {.page = 3, .column = 9});
  lcd.command(blofeld::test::lcd_rmw);
  auto untouched = lcd;
  const auto before = capture_lcd(lcd, {.width = 128});
  static_cast<void>(encode_lcd_png(before));
  constexpr unsigned original_byte = 0x24;
  constexpr unsigned replacement = 0x53;
  for (auto *controller : {&lcd, &untouched}) {
    require(controller->read_data().value == blofeld::test::lcd_rmw,
            "Capture preserves the dummy holder value");
    require(controller->read_data().value == original_byte && controller->state().column == 9,
            "Capture preserves the RMW dummy-read pipeline and selected address");
    controller->write_data(replacement);
    controller->command(blofeld::test::lcd_end);
    require(controller->state().column == 9 && !controller->state().read_modify_write,
            "Capture preserves the RMW saved column and END behavior");
  }
  require(capture_lcd(lcd, {.width = blofeld::St7565::columns}) ==
              capture_lcd(untouched, {.width = blofeld::St7565::columns}),
          "Capture and encoding preserve all independently exercised controller RAM");
  reject([&] { static_cast<void>(before.pixel(128, 0)); }, "outside the captured frame");
  for (const auto panel :
       {blofeld::LcdPanel{},
        blofeld::LcdPanel{.width = blofeld::St7565::columns + 1},
        blofeld::LcdPanel{.width = 2, .first_segment = last_segment},
        blofeld::LcdPanel{.width = 2, .reverse_segments = true},
        blofeld::LcdPanel{.width = 1, .first_segment = blofeld::St7565::columns}}) {
    reject([&] { static_cast<void>(capture_lcd(lcd, panel)); }, "LCD panel");
  }
}

void test_pbm() {
  auto lcd = blofeld::test::blank();
  select(lcd, {.page = 0, .column = 0});
  lcd.write_data(1);
  select(lcd, {.page = 0, .column = 8});
  lcd.write_data(1);
  select(lcd, {.page = 7, .column = 7});
  constexpr unsigned top_bit = 0x80;
  lcd.write_data(top_bit);
  const auto image = capture_lcd(lcd, {.width = 9});
  const std::string header = "P4\n9 64\n";
  std::vector<std::uint8_t> expected(header.begin(), header.end());
  constexpr std::size_t row_bytes = 2;
  expected.resize(header.size() + (row_bytes * blofeld::LcdImage::height));
  expected.at(header.size()) = top_bit;
  expected.at(header.size() + 1) = top_bit;
  constexpr unsigned last_row_offset = 126;
  expected.at(header.size() + last_row_offset) = 1;
  require(encode_lcd_pbm(image) == expected,
          "P4 header, MSB-first pixels, row order and zero padding");
  const auto text = encode_lcd_text(image);
  constexpr unsigned text_size = 640;
  require(text.size() == text_size && text.substr(0, 10) == "#       #\n" &&
              text.substr(text.size() - 10) == "       # \n",
          "Text coordinates agree with literal top/bottom row expectations");
}
}

int main() {
  try {
    test_mapping();
    test_unknown_and_modes();
    test_read_only();
    test_pbm();
    std::cout << "LCD literal pixel/mode/unknownness, read-only pipeline and PBM checks passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
