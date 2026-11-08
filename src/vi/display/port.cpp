// SPDX-License-Identifier: GPL-3.0-only
#include "display/port.hpp"

#include "display/st7565.hpp"
#include "mcu/ports.hpp"

#include <cstdint>
#include <exception>

namespace blofeld::vi {
namespace {
constexpr std::uint8_t select_mask = 0xc0, selected_value = 0x40;
constexpr std::uint8_t read_bit = 0x20, enable_bit = 0x10, all_bits = 0xff;
}

DisplayPort::DisplayPort() {
  // Peripheral power-on reset belongs to VM lifecycle, not inferred PCB wiring.
  controller_.hardware_reset();
}

bool DisplayPort::finish(const mcu::Ports &ports) {
  if (reading_) {
    if (data_) { static_cast<void>(controller_.read_data()); }
    return true;
  }
  const auto output = ports.output(mcu::Port::a);
  if (!output || (output->direction & output->known) != all_bits) {
    error_ = "LCD write has no complete MCU data byte";
    return false;
  }
  if (data_) {
    controller_.write_data(output->value);
  } else {
    controller_.command(output->value);
  }
  return true;
}

bool DisplayPort::update(const mcu::Ports &ports, const mcu::Ports::Write access) {
  if (access.offset == 8) { enable_ = (access.value & enable_bit) != 0; }
  const auto control = ports.output(mcu::Port::b);
  const auto selected = control &&
                        (control->direction & control->known & select_mask) == select_mask &&
                        (control->value & select_mask) == selected_value;
  const bool active = selected && enable_;
  if (active_ && !active) {
    active_ = false;
    return finish(ports);
  }
  if (!active) { return true; }
  constexpr std::uint8_t mode_mask = read_bit | 1;
  if ((control->direction & control->known & mode_mask) != mode_mask) {
    error_ = "LCD transaction has unknown control fields";
    return false;
  }
  const bool reading = (control->value & read_bit) != 0;
  const bool data = (control->value & 1) != 0;
  if (active_) {
    if (reading != reading_ || data != data_) {
      error_ = "LCD mode changed during an active transaction";
      return false;
    }
    return true;
  }
  active_ = true;
  reading_ = reading;
  data_ = data;
  if (reading_) {
    const auto output = ports.output(mcu::Port::a);
    if (!output || output->direction != 0) {
      error_ = "LCD read requires MCU data ownership release";
      return false;
    }
    held_ = data_ ? controller_.read_output()
                  : LcdByte{.value = controller_.status(), .known = all_bits};
  }
  return true;
}

bool DisplayPort::write(const mcu::Ports &ports, const mcu::Ports::Write access) noexcept {
  if (error_ != nullptr) { return false; }
  // Only A/B direction/data and E data can affect a completed display access.
  if (access.offset > 3 && access.offset != 8) { return true; }
  try {
    return update(ports, access);
  } catch (const std::exception &) {
    error_ = "LCD controller rejected command or data";
    return false;
  }
}

bool DisplayPort::read(std::uint8_t &value, const std::uint8_t mask) const noexcept {
  if (error_ != nullptr || !active_ || !reading_ || (held_.known & mask) != mask) { return false; }
  value = held_.value & mask;
  return true;
}
}
