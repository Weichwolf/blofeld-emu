// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "keyboard/hcs08_flash.hpp"

#include <cstdint>

namespace blofeld {
namespace hf = hcs08_flash;

namespace {
constexpr unsigned divider_mask = 0x7f, option_mask = 0xc3;
}

std::uint8_t Hcs08Flash::read(const std::uint16_t address) {
  if (state_.buffer == Hcs08FlashBuffer::command) { reject(hf::access_error); }
  switch (address) {
    case hf::divider:
      return static_cast<std::uint8_t>(state_.divider | (state_.divider_loaded ? hf::empty : 0));
    case hf::options: return known_register(options_, {.address = address, .mask = option_mask});
    case hf::configuration: return 0; // Only KEYACC=0 is qualified.
    case hf::protection:
      return known_register(protection_, {.address = address, .mask = hf::erased});
    case hf::status: return status();
    case hf::command: return 0; // Read-zero, not the buffered command's encoding.
    default: fail("register read", address, "unsupported register");
  }
}

void Hcs08Flash::write(const ByteWrite access) {
  if (access.address < hf::divider || access.address > hf::command ||
      access.address == hf::divider + 2) {
    fail("register write", access.address, "unsupported register");
  }
  const bool data_waiting = state_.buffer == Hcs08FlashBuffer::data;
  const bool command_waiting = state_.buffer == Hcs08FlashBuffer::command;
  if ((data_waiting && access.address != hf::command && access.address != hf::status) ||
      (command_waiting && access.address != hf::status)) {
    reject(hf::access_error);
    return;
  }
  switch (access.address) {
    case hf::divider:
      if (!state_.divider_loaded && (state_.errors & hf::access_error) == 0) {
        state_.divider = static_cast<std::uint8_t>(access.value & divider_mask);
        state_.divider_loaded = true;
      }
      return;
    case hf::options: return; // CPU and BDC writes have no meaning (4.6.2).
    case hf::configuration:
      if (access.value != 0) { fail("register write", access.address, "KEYACC is not qualified"); }
      return;
    case hf::protection:
      if (agent_ == Hcs08FlashAgent::background) {
        if (state_.active || state_.buffer == Hcs08FlashBuffer::launched) {
          fail("register write", access.address, "unqualified active protection change");
        }
        protection_ = {.value = access.value, .known = hf::erased};
      }
      return; // Application writes are architecturally ignored.
    case hf::status: write_status(access.value); return;
    case hf::command: write_command(access.value); return;
    default: fail("register write", access.address, "unsupported register");
  }
}

void Hcs08Flash::reject(const std::uint8_t errors) {
  state_.errors = static_cast<std::uint8_t>(state_.errors | errors);
  if (state_.buffer != Hcs08FlashBuffer::launched) {
    state_.buffer = Hcs08FlashBuffer::empty;
    state_.staged = {};
  }
}

void Hcs08Flash::write_command(const std::uint8_t value) {
  if (state_.errors != 0) { return; }
  const bool valid = value == hf::verify || value == hf::program || value == hf::burst ||
                     value == hf::erase_page || value == hf::erase_all;
  if (!valid || state_.buffer != Hcs08FlashBuffer::data ||
      (agent_ == Hcs08FlashAgent::background && value != hf::verify && value != hf::erase_all &&
       secure())) {
    reject(hf::access_error);
    return;
  }
  state_.staged.command = value;
  state_.buffer = Hcs08FlashBuffer::command;
}

void Hcs08Flash::write_status(const std::uint8_t value) {
  const bool launch = (value & hf::empty) != 0;
  if (launch && state_.buffer == Hcs08FlashBuffer::empty && state_.active) {
    fail("register write",
         hf::status,
         "unqualified empty-buffer launch during an executing command");
  }
  const bool acknowledge_error = (state_.errors & value & hf::access_error) != 0;
  if (launch && state_.buffer == Hcs08FlashBuffer::empty && !acknowledge_error &&
      idle_ != Hcs08IdleLaunch::reference_ignore_empty) {
    fail("register write", hf::status, "unqualified idle FCBEF launch without a staged command");
  }
  if (launch && state_.buffer == Hcs08FlashBuffer::launched) {
    fail("register write", hf::status, "unqualified duplicate launch while buffer is occupied");
  }
  state_.errors =
      static_cast<std::uint8_t>(state_.errors & ~(value & (hf::access_error | hf::violation)));
  if (state_.buffer == Hcs08FlashBuffer::empty || state_.buffer == Hcs08FlashBuffer::launched) {
    return;
  }
  if (!launch || state_.buffer != Hcs08FlashBuffer::command) {
    reject(hf::access_error);
    return;
  }
  if (state_.errors != 0) { return; }
  if (state_.active &&
      (state_.active->command != hf::burst || state_.staged.command != hf::burst)) {
    fail("register write", hf::status, "only burst commands have a qualified successor buffer");
  }
  if (command_protected(state_.staged)) {
    reject(hf::violation);
    return;
  }
  state_.blank = false;
  state_.buffer = Hcs08FlashBuffer::launched;
}
} // namespace blofeld
