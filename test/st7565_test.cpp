// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "display/st7565.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string_view>

namespace {
constexpr unsigned byte_mask = 255;
constexpr unsigned page_command = 0xb0;
constexpr unsigned reset_command = 0xe2;
constexpr unsigned rmw_command = 0xe0;
constexpr unsigned end_command = 0xee;
constexpr unsigned nop_command = 0xe3;
constexpr unsigned last_column = 131;
constexpr unsigned display_on = 0xaf;
constexpr unsigned display_off = 0xae;
constexpr unsigned reverse_on = 0xa7;
constexpr unsigned adc_reverse = 0xa1;
constexpr unsigned all_points_on = 0xa5;
constexpr unsigned bias_on = 0xa3;
constexpr unsigned common_reverse = 0xc8;
constexpr unsigned power_all = 0x2f;
constexpr unsigned resistor_five = 0x25;
constexpr unsigned indicator_on = 0xad;
constexpr unsigned indicator_off = 0xac;
constexpr unsigned volume_command = 0x81;
constexpr unsigned booster_command = 0xf8;
constexpr unsigned max_volume = 63;
constexpr unsigned volume_from_reset_parameter = 34;
constexpr unsigned invalid_page = 0xb9;
constexpr unsigned invalid_column_high = 0x1f;

void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void reject(const auto &operation, const std::string_view message) {
  try {
    operation();
  } catch (const blofeld::Fault &fault) {
    require(std::string_view(fault.what()).contains(message), "LCD fault identifies its cause");
    return;
  }
  throw blofeld::Fault("Expected an explicit LCD qualification fault");
}

void equal(const blofeld::LcdByte byte, const unsigned value, const unsigned known = byte_mask) {
  require(byte.value == value && byte.known == known, "Independent LCD data and known-bit oracle");
}

struct Position {
  unsigned page = 0;
  unsigned column = 0;
};

void select(blofeld::St7565 &lcd, const Position position) {
  const auto [page, column] = position;
  lcd.command(static_cast<std::uint8_t>(page_command | page));
  lcd.command(static_cast<std::uint8_t>(16U | (column >> 4U)));
  lcd.command(static_cast<std::uint8_t>(column & 15U));
}

void test_reset_and_unknowns() {
  blofeld::St7565 lcd;
  reject([&] { lcd.command(nop_command); }, "before an observed hardware reset");
  reject([&] { lcd.write_data(1); }, "before an observed hardware reset");
  reject([&] { static_cast<void>(lcd.read_data()); }, "before an observed hardware reset");
  reject([&] { static_cast<void>(lcd.ram(0, 0)); }, "before an observed hardware reset");
  reject([&] { static_cast<void>(lcd.state()); }, "before an observed hardware reset");
  lcd.hardware_reset();
  const auto state = lcd.state();
  constexpr unsigned initial_volume = 32;
  require(!state.display_on && !state.reverse && !state.adc_reverse && !state.all_points &&
              !state.bias && !state.common_reverse && !state.static_on &&
              !state.read_modify_write && state.page == 0 && state.column == 0 &&
              state.start_line == 0 && state.volume == initial_volume && state.power == 0 &&
              state.static_mode == 0 && !state.resistor && !state.booster,
          "Documented reset registers are established without guessing undocumented values");
  equal(lcd.read_data(), 0, 0);
  equal(lcd.read_data(), 0, 0);
  equal(lcd.ram(0, 0), 0, 0);
  lcd.write_data(byte_mask);
  const auto column = lcd.state().column - 1U;
  lcd.hardware_reset();
  equal(lcd.ram(0, column), byte_mask);
  equal(lcd.read_data(), 0, 0);
  require(lcd.state().column == 0, "Hardware reset preserves RAM but invalidates read history");
}

void test_all_cells_and_saturation() {
  blofeld::St7565 lcd;
  lcd.hardware_reset();
  constexpr unsigned stride = 37;
  for (unsigned page = 0; page < blofeld::St7565::pages; ++page) {
    select(lcd, {.page = page, .column = 0});
    for (unsigned column = 0; column < blofeld::St7565::columns; ++column) {
      lcd.write_data(static_cast<std::uint8_t>((page ^ (column * stride)) & byte_mask));
    }
    require(lcd.state().column == last_column && lcd.state().page == page,
            "Columns saturate at 83H independently of page, never wrapping");
  }
  for (unsigned page = 0; page < blofeld::St7565::pages; ++page) {
    const unsigned mask = page == 8 ? 1 : byte_mask;
    for (unsigned column = 0; column < blofeld::St7565::columns; ++column) {
      equal(lcd.ram(page, column), (page ^ (column * stride)) & mask, mask);
    }
  }
  constexpr unsigned icon_value = 0x5a;
  lcd.write_data(icon_value);
  equal(lcd.ram(8, last_column), 0, 1);
  require(lcd.state().column == last_column && lcd.state().page == 8,
          "Further writes replace the saturated last icon cell");
  reject([&] { static_cast<void>(lcd.ram(9, 0)); }, "page=0x0009 column=0x0000");
  reject([&] { static_cast<void>(lcd.ram(0, blofeld::St7565::columns)); },
         "page=0x0000 column=0x0084");
}

void test_read_pipeline() {
  blofeld::St7565 lcd;
  lcd.hardware_reset();
  constexpr std::array<std::uint8_t, 3> contents{0x12, 0x34, 0x6c};
  for (const auto value : contents) { lcd.write_data(value); }
  select(lcd, {.page = 0, .column = 0});
  equal(lcd.read_data(), 0);
  require(lcd.state().column == 0, "The dummy read loads the bus holder without incrementing");
  equal(lcd.read_data(), contents[0]);
  require(lcd.state().column == 1, "The first valid read increments after returning column zero");
  lcd.command(nop_command);
  equal(lcd.read_data(), nop_command);
  require(lcd.state().column == 1, "An intervening MPU command write requires a fresh dummy read");
  equal(lcd.read_data(), contents[1]);
  equal(lcd.read_data(), contents[2]);
  equal(lcd.read_data(), 0, 0);
  constexpr unsigned replacement = 0xa5;
  select(lcd, {.page = 0, .column = 2});
  lcd.write_data(replacement);
  select(lcd, {.page = 0, .column = 0});
  equal(lcd.read_data(), 0);
  equal(lcd.read_data(), contents[0]);
  select(lcd, {.page = 0, .column = last_column});
  lcd.write_data(replacement);
  equal(lcd.read_data(), replacement);
  equal(lcd.read_data(), replacement);
  require(lcd.state().column == last_column, "Read increments also saturate at 83H");
}

void test_read_modify_write() {
  blofeld::St7565 lcd;
  lcd.hardware_reset();
  constexpr unsigned first = 0x55;
  constexpr unsigned second = 0x66;
  constexpr unsigned changed = 0xaa;
  select(lcd, {.page = 3, .column = 5});
  lcd.write_data(first);
  lcd.write_data(second);
  select(lcd, {.page = 3, .column = 5});
  lcd.command(rmw_command);
  equal(lcd.read_data(), rmw_command);
  equal(lcd.read_data(), first);
  equal(lcd.read_data(), first);
  require(lcd.state().column == 5, "RMW reads preserve the selected column");
  lcd.write_data(changed);
  require(lcd.state().column == 6, "Only an RMW write increments the selected column");
  equal(lcd.read_data(), changed);
  equal(lcd.read_data(), second);
  lcd.command(end_command);
  require(lcd.state().column == 5 && !lcd.state().read_modify_write,
          "END restores the column saved by RMW entry");
  equal(lcd.ram(3, 5), changed);
  equal(lcd.ram(3, 6), second);
  equal(lcd.ram(2, 5), 0, 0);
}

void test_modes_and_two_byte_commands() {
  blofeld::St7565 lcd;
  lcd.hardware_reset();
  constexpr std::array<std::uint8_t, 9> enabled{
      0xaf, 0xa7, 0xa1, 0xa5, 0xa3, 0xcf, 0x2f, 0x27, 0x7f};
  for (const auto command : enabled) { lcd.command(command); }
  lcd.command(indicator_on);
  reject([&] { lcd.write_data(1); }, "during a two-byte command");
  reject([&] { static_cast<void>(lcd.read_data()); }, "during a two-byte command");
  lcd.command(3);
  lcd.command(volume_command);
  lcd.command(byte_mask);
  lcd.command(booster_command);
  lcd.command(1);
  const auto state = lcd.state();
  require(state.display_on && state.reverse && state.adc_reverse && state.all_points &&
              state.bias && state.common_reverse && state.power == 7 && state.resistor == 7 &&
              state.static_on && state.static_mode == 3 && state.volume == max_volume &&
              state.start_line == max_volume && state.booster == 1,
          "Independent command table decodes modes, ignored bits and consecutive parameters");
  lcd.command(indicator_off);
  lcd.command(display_off);
  require(!lcd.state().static_on && !lcd.state().display_on,
          "Static indicator OFF is a single-byte command; the following opcode remains an opcode");
  lcd.command(volume_command);
  lcd.command(reset_command);
  require(lcd.state().volume == volume_from_reset_parameter && lcd.state().start_line == max_volume,
          "A reset-looking parameter remains volume data while the double-byte mode is active");
  lcd.command(nop_command);
  require(lcd.state().volume == volume_from_reset_parameter, "NOP preserves mode state");
}

void test_reset_differences_and_address_isolation() {
  blofeld::St7565 lcd;
  lcd.hardware_reset();
  lcd.command(display_on);
  lcd.command(reverse_on);
  lcd.command(adc_reverse);
  lcd.command(all_points_on);
  lcd.command(bias_on);
  lcd.command(common_reverse);
  lcd.command(power_all);
  lcd.command(resistor_five);
  select(lcd, {.page = 4, .column = 12});
  constexpr unsigned value = 0x96;
  lcd.write_data(value);
  equal(lcd.ram(4, 12), value);
  equal(lcd.ram(0, 12), 0, 0);
  lcd.command(rmw_command);
  lcd.command(reset_command);
  const auto state = lcd.state();
  require(state.display_on && state.reverse && state.adc_reverse && state.all_points &&
              state.bias && state.power == 7 && state.resistor == 5 && !state.common_reverse &&
              !state.read_modify_write && state.page == 0 && state.column == 0 &&
              state.start_line == 0 && state.volume == 32,
          "Software reset applies only reset-list entries 11..19, retaining prior display modes");
  equal(lcd.ram(4, 12), value);
  lcd.hardware_reset();
  require(!lcd.state().display_on && !lcd.state().adc_reverse && !lcd.state().resistor,
          "Hardware reset also resets display modes without inventing a resistor reset value");
  equal(lcd.ram(4, 12), value);
}

void test_rejected_accesses() {
  blofeld::St7565 lcd;
  lcd.hardware_reset();
  for (const auto command : {0x30, 0x80, 0xe1, 0xf0, 0xff}) {
    reject([&] { lcd.command(static_cast<std::uint8_t>(command)); }, "Unsupported ST7565 command");
  }
  reject([&] { lcd.command(invalid_page); }, "Unqualified ST7565 page command 0xb9");
  require(lcd.state().page == 0, "A rejected page command cannot change the previous page");
  reject([&] { lcd.command(end_command); }, "END without RMW");
  lcd.command(invalid_column_high);
  reject([&] { lcd.write_data(1); }, "page=0x0000 column=0x00f0");
  reject([&] { static_cast<void>(lcd.read_data()); }, "page=0x0000 column=0x00f0");
  lcd.command(reset_command);
  equal(lcd.ram(0, 0), 0, 0);
  lcd.command(booster_command);
  reject([&] { lcd.command(2); }, "Unqualified ST7565 booster ratio 0x02");
  lcd.command(3);
  require(lcd.state().booster == 3, "Rejected booster selection retains the pending parameter");
}
}

int main() {
  try {
    test_reset_and_unknowns();
    test_all_cells_and_saturation();
    test_read_pipeline();
    test_read_modify_write();
    test_modes_and_two_byte_commands();
    test_reset_differences_and_address_isolation();
    test_rejected_accesses();
    std::cout << "ST7565 RAM, bus holder, dummy reads, RMW, command parameters and reset checks "
                 "passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
