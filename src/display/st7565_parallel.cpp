// SPDX-License-Identifier: GPL-3.0-only
#include "display/st7565_parallel.hpp"

#include "common/bus.hpp"

#include <optional>

namespace blofeld {
namespace {
constexpr unsigned byte_mask = 255;
constexpr unsigned address_controls = st7565_pin::read | st7565_pin::data;
}

void St7565Parallel::validate(const LcdHostPins pins) {
  if ((pins.known & ~st7565_pin::all) != 0 || (pins.control & ~pins.known) != 0 ||
      (pins.data.value & ~pins.data.known) != 0 || (pins.data.known & ~pins.drive) != 0) {
    throw Fault("Invalid ST7565 parallel pin/known/drive masks");
  }
  if ((pins.known & st7565_pin::reset_n) == 0) { throw Fault("Unknown ST7565 reset input"); }
}

bool St7565Parallel::enabled(const LcdHostPins pins) {
  const auto levels = pins.control;
  const auto known = pins.known;
  if (((known & st7565_pin::cs1_n) != 0 && (levels & st7565_pin::cs1_n) != 0) ||
      ((known & st7565_pin::cs2) != 0 && (levels & st7565_pin::cs2) == 0)) {
    return false;
  }
  constexpr unsigned selects = st7565_pin::cs1_n | st7565_pin::cs2;
  if ((known & selects) != selects) { throw Fault("Unknown ST7565 chip select inputs"); }
  if ((known & st7565_pin::enable) == 0) { throw Fault("Unknown ST7565 enable input"); }
  if ((levels & st7565_pin::enable) == 0) { return false; }
  if ((known & address_controls) != address_controls) {
    throw Fault("Unknown ST7565 R/W or A0 input while enabled");
  }
  return true;
}

void St7565Parallel::begin_cycle(const LcdHostPins pins) {
  if (!reset_seen_) { throw Fault("ST7565 parallel cycle before observed hardware reset"); }
  const bool read = (pins.control & st7565_pin::read) != 0;
  if (read) {
    if (pins.drive != 0) { throw Fault("ST7565 host drives data during read"); }
    if ((pins.control & st7565_pin::data) == 0) {
      throw Fault("Unqualified ST7565 status read: busy/reset timing is unknown");
    }
    output_ = controller_.read_output();
  }
  previous_ = pins;
  active_ = true;
}

LcdCycle St7565Parallel::finish_cycle(const LcdHostPins pins) {
  const bool read = (previous_.control & st7565_pin::read) != 0;
  const bool data = (previous_.control & st7565_pin::data) != 0;
  if (read) { return {.read = true, .data = true, .byte = controller_.read_data()}; }
  if (previous_.drive != byte_mask || previous_.data.known != byte_mask ||
      pins.drive != byte_mask || pins.data.known != byte_mask ||
      pins.data.value != previous_.data.value) {
    throw Fault("ST7565 write data not stable and fully driven before completion");
  }
  if (data) {
    controller_.write_data(pins.data.value);
  } else {
    controller_.command(pins.data.value);
  }
  return {.data = data, .byte = pins.data};
}

LcdBusResult St7565Parallel::output(const std::optional<LcdCycle> completed) const {
  if (active_ && (previous_.control & st7565_pin::read) != 0) {
    return {.drive = byte_mask, .data = output_, .completed = completed};
  }
  return {.completed = completed};
}

LcdBusResult St7565Parallel::sample(const LcdHostPins pins) {
  validate(pins);
  if ((pins.control & st7565_pin::reset_n) == 0) {
    if (!in_reset_) { controller_.hardware_reset(); }
    reset_seen_ = true;
    in_reset_ = true;
    active_ = false;
    output_ = {};
    return {};
  }
  in_reset_ = false;
  const bool next = enabled(pins);
  std::optional<LcdCycle> completed;
  if (active_) {
    // Figure 38 specifies tAH6=0: new address levels at a proven end do
    // not change the previous cycle. During an enabled phase they must hold.
    if (next && ((pins.known & address_controls) != address_controls ||
                 ((pins.control ^ previous_.control) & address_controls) != 0)) {
      throw Fault("ST7565 R/W or A0 changed during an active cycle");
    }
    if ((previous_.control & st7565_pin::read) != 0 && pins.drive != 0) {
      throw Fault("ST7565 host drives data during read");
    }
    if (!next) {
      completed = finish_cycle(pins);
      active_ = false;
      output_ = {};
    } else {
      previous_ = pins;
    }
  } else if (next) {
    begin_cycle(pins);
  }
  return output(completed);
}

}
