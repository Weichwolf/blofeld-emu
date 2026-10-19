// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/st7565.hpp"

#include "blofeld/bus.hpp"

#include <cstddef>
#include <cstdint>

namespace blofeld {
namespace {
constexpr unsigned nibble = 15;
constexpr unsigned byte_mask = 255;
constexpr unsigned icon_page = 8;
constexpr unsigned page_command = 0xb0;
constexpr unsigned start_line_command = 0x40;
constexpr unsigned start_line_mask = 0x3f;
constexpr unsigned display_command = 0xae;
constexpr unsigned adc_command = 0xa0;
constexpr unsigned all_points_command = 0xa4;
constexpr unsigned reverse_command = 0xa6;
constexpr unsigned bias_command = 0xa2;
constexpr unsigned common_command = 0xc0;
constexpr unsigned power_command = 0x28;
constexpr unsigned resistor_command = 0x20;
constexpr unsigned indicator_command = 0xac;
constexpr unsigned volume_command = 0x81;
constexpr unsigned booster_command = 0xf8;
constexpr unsigned rmw_command = 0xe0;
constexpr unsigned end_command = 0xee;
constexpr unsigned reset_command = 0xe2;
constexpr unsigned nop_command = 0xe3;
constexpr unsigned initial_volume = 32;
}

void St7565::require_reset() const {
  if (!reset_observed_) { throw Fault("ST7565 access before an observed hardware reset"); }
}

std::size_t St7565::address(const unsigned page, const unsigned column) {
  if (page >= pages || column >= columns) {
    throw Fault("Unqualified ST7565 RAM address page=" + hex(page) + " column=" + hex(column));
  }
  return (static_cast<std::size_t>(page) * columns) + column;
}

void St7565::hardware_reset() {
  state_ = {};
  software_reset();
  holder_ = {};
  reset_observed_ = true;
}

void St7565::software_reset() {
  state_.page = 0;
  state_.column = 0;
  state_.start_line = 0;
  state_.volume = initial_volume;
  state_.static_mode = 0;
  state_.static_on = false;
  state_.common_reverse = false;
  state_.read_modify_write = false;
  parameter_ = Parameter::none;
  saved_column_ = 0;
  dummy_ = true;
}

void St7565::advance_column() {
  if (state_.column < columns - 1) { ++state_.column; }
}

LcdByte St7565::ram(const unsigned page, const unsigned column) const {
  require_reset();
  const auto index = address(page, column);
  const auto mask = static_cast<std::uint8_t>(page == icon_page ? 1 : byte_mask);
  return {.value = static_cast<std::uint8_t>(bytes_[index] & mask),
          .known = static_cast<std::uint8_t>(known_[index] ? mask : 0)};
}

St7565State St7565::state() const {
  require_reset();
  return state_;
}

void St7565::write_data(const std::uint8_t value) {
  require_reset();
  if (parameter_ != Parameter::none) {
    throw Fault("Unqualified ST7565 data write during a two-byte command");
  }
  const auto index = address(state_.page, state_.column);
  const auto mask = static_cast<std::uint8_t>(state_.page == icon_page ? 1 : byte_mask);
  bytes_[index] = static_cast<std::uint8_t>(value & mask);
  known_.set(index);
  // The datasheet's common bus holder accepts MPU write data before the RAM.
  holder_ = {.value = value, .known = byte_mask};
  advance_column();
  dummy_ = true;
}

LcdByte St7565::read_output() const {
  require_reset();
  if (parameter_ != Parameter::none) {
    throw Fault("Unqualified ST7565 data read during a two-byte command");
  }
  static_cast<void>(address(state_.page, state_.column));
  return holder_;
}

LcdByte St7565::read_data() {
  const auto result = read_output();
  if (!dummy_ && !state_.read_modify_write) { advance_column(); }
  holder_ = ram(state_.page, state_.column);
  dummy_ = false;
  return result;
}

void St7565::set_address(const std::uint8_t value) {
  if ((value & start_line_command) != 0) {
    state_.start_line = static_cast<std::uint8_t>(value & start_line_mask);
  } else if ((value & page_command) == page_command) {
    const auto page = static_cast<unsigned>(value & nibble);
    if (page >= pages) { throw Fault("Unqualified ST7565 page command " + hex(value, 2)); }
    state_.page = static_cast<std::uint8_t>(page);
    dummy_ = true;
  } else {
    const auto low = static_cast<unsigned>(state_.column) & nibble;
    const auto high = static_cast<unsigned>(state_.column) & ~nibble;
    state_.column = static_cast<std::uint8_t>((value & 16U) != 0 ? ((value & nibble) << 4U) | low
                                                                 : high | (value & nibble));
    dummy_ = true;
  }
}

void St7565::set_parameter(const std::uint8_t value) {
  switch (parameter_) {
    case Parameter::volume:
      state_.volume = static_cast<std::uint8_t>(value & start_line_mask);
      break;
    case Parameter::indicator: state_.static_mode = static_cast<std::uint8_t>(value & 3U); break;
    case Parameter::booster:
      if ((value & 3U) == 2) { throw Fault("Unqualified ST7565 booster ratio " + hex(value, 2)); }
      state_.booster = static_cast<std::uint8_t>(value & 3U);
      break;
    case Parameter::none: throw Fault("ST7565 parameter without a command");
    default: throw Fault("Unknown ST7565 parameter mode");
  }
  parameter_ = Parameter::none;
}

void St7565::set_mode(const std::uint8_t value) {
  const bool enabled = (value & 1U) != 0;
  switch (value & ~1U) {
    case display_command: state_.display_on = enabled; return;
    case adc_command: state_.adc_reverse = enabled; return;
    case all_points_command: state_.all_points = enabled; return;
    case reverse_command: state_.reverse = enabled; return;
    case bias_command: state_.bias = enabled; return;
    case indicator_command:
      state_.static_on = enabled;
      if (enabled) { parameter_ = Parameter::indicator; }
      return;
    default: break;
  }
  if ((value & ~7U) == common_command) {
    state_.common_reverse = false;
  } else if ((value & ~7U) == (common_command | 8U)) {
    state_.common_reverse = true;
  } else if ((value & ~7U) == power_command) {
    state_.power = static_cast<std::uint8_t>(value & 7U);
  } else if ((value & ~7U) == resistor_command) {
    state_.resistor = static_cast<std::uint8_t>(value & 7U);
  } else {
    throw Fault("Unsupported ST7565 command " + hex(value, 2));
  }
}

void St7565::command(const std::uint8_t value) {
  require_reset();
  // Figure 2: the address/command write is itself retained in the common
  // bus holder and is returned by the next dummy read, before RAM data.
  holder_ = {.value = value, .known = byte_mask};
  dummy_ = true;
  if (parameter_ != Parameter::none) {
    set_parameter(value);
    return;
  }
  if (value < resistor_command || (value & ~start_line_mask) == start_line_command ||
      (value & ~nibble) == page_command) {
    set_address(value);
    return;
  }
  switch (value) {
    case volume_command: parameter_ = Parameter::volume; return;
    case booster_command: parameter_ = Parameter::booster; return;
    case rmw_command:
      state_.read_modify_write = true;
      saved_column_ = state_.column;
      return;
    case end_command:
      if (!state_.read_modify_write) { throw Fault("Unqualified ST7565 END without RMW"); }
      state_.column = saved_column_;
      state_.read_modify_write = false;
      dummy_ = true;
      return;
    case reset_command: software_reset(); return;
    case nop_command: return;
    default: set_mode(value); return;
  }
}

}
