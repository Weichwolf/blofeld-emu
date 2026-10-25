// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "keyboard/hcs08_adc.hpp"

#include <cstdint>

namespace blofeld {
namespace adc = hcs08_adc;

namespace {
constexpr unsigned format_mask = 0x70, writable_status = 0x7f, byte_mask = 0xff;
}

bool Hcs08Adc::contains(const unsigned address) {
  return address >= adc::control && address <= adc::pins;
}

void Hcs08Adc::abort() {
  state_.active = false;
  state_.remaining = 0;
}

std::uint8_t Hcs08Adc::read(const std::uint16_t address) {
  healthy();
  switch (address) {
    case adc::control: return state_.control;
    case adc::status:
      return static_cast<std::uint8_t>(state_.status | (state_.complete ? adc::complete : 0U));
    case adc::pins: return state_.pins;
    case adc::high:
    case adc::low:
      if (!state_.result) { fail("read", address, "result has no qualified completed conversion"); }
      state_.complete = false;
      return static_cast<std::uint8_t>(address == adc::high ? *state_.result >> 8U
                                                            : *state_.result & byte_mask);
    default: fail("read", address, "reserved or unsupported register");
  }
}

void Hcs08Adc::write(const ByteWrite access) {
  healthy();
  switch (access.address) {
    case adc::control:
      if (state_.result && ((state_.control ^ access.value) & format_mask) != 0) {
        fail("write", access.address, "reformatting an existing result is unqualified");
      }
      abort();
      state_.control = access.value;
      return;
    case adc::status:
      abort();
      state_.complete = false;
      state_.status = static_cast<std::uint8_t>(access.value & writable_status);
      if ((state_.control & adc::power) != 0) {
        state_.active = true;
        state_.remaining = continuous_periods() + clock_divisor() + 2U;
      }
      return;
    case adc::pins:
      abort();
      state_.pins = access.value;
      return;
    case adc::high:
    case adc::low: return; // Read-only result data, never a host completion path.
    default: fail("write", access.address, "reserved or unsupported register");
  }
}
} // namespace blofeld
