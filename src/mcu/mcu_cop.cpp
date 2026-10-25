// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/mcu_cop.hpp"

#include "common/bus.hpp"

#include <algorithm>
#include <array>
#include <cstdint>

namespace blofeld {
namespace {
constexpr unsigned rate_mask = 7;
constexpr unsigned control_mask = 0xc7;
constexpr unsigned window_mode = 0x80;
constexpr unsigned arm_value = 0x55;
constexpr unsigned finish_value = 0xaa;
constexpr std::array<unsigned, 8> period_bits{0, 14, 16, 18, 20, 22, 23, 24};
}

void McuCop::reset() {
  state_ = {};
}

void McuCop::healthy() const {
  if (state_.reset != McuCopReset::none) {
    throw Fault("MCU COP reset request requires system reset, not further service");
  }
}

unsigned McuCop::period() const {
  const auto rate = state_.control & rate_mask;
  return rate == 0 ? 0 : 1U << period_bits.at(rate);
}

unsigned McuCop::remaining() const {
  const auto limit = period();
  return state_.reset != McuCopReset::none || limit == 0 ? 0 : limit - state_.phase;
}

void McuCop::write_control(const std::uint8_t value) {
  healthy();
  if (state_.control_written) { return; }
  if ((value & ~control_mask) != 0) {
    throw Fault("Unqualified normal COPCTL reserved bits: value=" + hex(value, 2));
  }
  state_.control = value;
  state_.control_written = true;
}

void McuCop::service(const std::uint8_t value) {
  healthy();
  const auto limit = period();
  if (limit == 0) { return; }
  if (value != arm_value && value != finish_value) {
    state_.reset = McuCopReset::invalid_service;
    return;
  }
  if ((state_.control & window_mode) != 0 && state_.phase < limit - (limit / 4)) {
    state_.reset = McuCopReset::early_service;
    return;
  }
  if (value == arm_value) {
    state_.armed = true;
  } else if (state_.armed) {
    state_.phase = 0;
    state_.armed = false;
  }
}

unsigned McuCop::advance(const unsigned cycles) {
  if (state_.reset != McuCopReset::none) { return 0; }
  const auto limit = period();
  if (limit == 0) { return cycles; }
  const auto elapsed = std::min(cycles, limit - state_.phase);
  state_.phase += elapsed;
  if (state_.phase == limit) { state_.reset = McuCopReset::timeout; }
  return elapsed;
}
}
