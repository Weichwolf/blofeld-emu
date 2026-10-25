// SPDX-License-Identifier: GPL-3.0-only
#include "keyboard/hcs08_adc.hpp"

#include "common/bus.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

namespace blofeld {
namespace adc = hcs08_adc;

namespace {
constexpr unsigned prescaler_mask = 0xf, conversion_clocks = 28;
constexpr unsigned minimum_supply = 1800000, fast_supply = 2080000, maximum_supply = 3600000;
constexpr unsigned minimum_clock = 500000, fast_clock = 2000000, slow_clock = 1000000;
constexpr unsigned maximum_bus = 20000000, reference_high_channel = 0x1e;
constexpr unsigned word_shift = 6, byte_shift = 8, sign_bit = 0x8000, quantization_steps = 1024;
}

Hcs08Adc::Hcs08Adc(const Hcs08AdcContract contract) {
  if (contract != Hcs08AdcContract::candidate_gt8a_reference_stable_input) {
    fail("configure", adc::control, "requires explicit candidate ideal stable-input contract");
  }
}

void Hcs08Adc::context(const ExecutionContext context) {
  context_ = context;
}

void Hcs08Adc::reset() {
  const auto inputs = state_.inputs;
  state_ = {.inputs = inputs};
  healthy_ = true;
}

void Hcs08Adc::observe_reference(const Hcs08AdcReference reference) {
  healthy();
  if (state_.active) { fail("observe reference", adc::control, "conversion must be inactive"); }
  if (reference.bus_hz == 0 || reference.bus_hz > maximum_bus ||
      reference.supply_uv < minimum_supply || reference.supply_uv > maximum_supply ||
      reference.low_uv != 0 || reference.high_uv > reference.supply_uv ||
      reference.high_uv < std::min(reference.supply_uv, fast_supply)) {
    fail("observe reference", adc::control, "outside documented clock/supply/reference envelope");
  }
  state_.reference = reference;
}

void Hcs08Adc::observe_input(const unsigned channel,
                             const std::optional<std::uint32_t> microvolts) {
  healthy();
  if (state_.active || channel >= adc::channels) {
    fail("observe pin", adc::pins, "requires inactive converter and an actual analog pin");
  }
  state_.inputs.at(channel) = microvolts;
}

unsigned Hcs08Adc::clock_divisor() const {
  healthy();
  return ((state_.control & prescaler_mask) + 1U) * 2U;
}

unsigned Hcs08Adc::continuous_periods() const {
  return conversion_clocks * clock_divisor();
}

std::uint16_t Hcs08Adc::reference_result() const {
  if (!state_.reference) { fail("clock", adc::status, "unknown bus/supply/reference inputs"); }
  const auto reference = *state_.reference;
  const auto divisor = clock_divisor();
  const auto maximum = reference.supply_uv >= fast_supply ? fast_clock : slow_clock;
  if (reference.bus_hz < minimum_clock * divisor || reference.bus_hz > maximum * divisor) {
    fail("clock", adc::control, "ATD clock outside documented supply-dependent range");
  }
  const auto channel = state_.status & adc::channel_mask;
  unsigned code = 0;
  if (channel == reference_high_channel) {
    code = adc::maximum;
  } else if (channel < adc::channels && (state_.pins & (1U << channel)) != 0) {
    const auto input = state_.inputs.at(channel);
    if (!input) {
      fail("sample", adc::status, "unknown enabled analog pin=" + std::to_string(channel));
    }
    if (*input > reference.supply_uv) {
      fail("sample", adc::status, "input exceeds declared supply");
    }
    const std::uint64_t span = reference.high_uv - reference.low_uv;
    const auto scaled = (std::uint64_t{*input} * quantization_steps * 2U + span) / (span * 2U);
    code = static_cast<unsigned>(std::min<std::uint64_t>(scaled, adc::maximum));
  } // Reserved, disabled and VREFL channels convert the documented reference low.
  const bool narrow = (state_.control & adc::eight) != 0;
  unsigned word = code;
  if (narrow) {
    word = (code >> 2U) << byte_shift;
  } else if ((state_.control & adc::right) == 0) {
    word = code << word_shift;
  }
  if ((state_.control & (adc::right | adc::sign)) == adc::sign) { word ^= sign_bit; }
  return static_cast<std::uint16_t>(word);
}

void Hcs08Adc::advance_bus(const std::uint64_t periods) {
  healthy();
  if (periods == 0 || !state_.active) { return; }
  const auto result = reference_result();
  if (periods < state_.remaining) {
    state_.remaining -= static_cast<unsigned>(periods);
    return;
  }
  const auto remainder = periods - state_.remaining;
  const bool continuous = (state_.status & adc::continuous) != 0;
  const auto extra = continuous ? remainder / continuous_periods() : 0;
  if (extra >= std::numeric_limits<std::uint64_t>::max() - state_.conversions) {
    fail("clock", adc::status, "conversion diagnostic count exhausted");
  }
  state_.result = result;
  state_.complete = true;
  state_.conversions += extra + 1;
  state_.active = continuous;
  state_.remaining =
      continuous ? continuous_periods() - static_cast<unsigned>(remainder % continuous_periods())
                 : 0;
}

bool Hcs08Adc::interrupt_requested() const {
  healthy();
  return state_.complete && (state_.status & adc::interrupt) != 0;
}

Hcs08AdcState Hcs08Adc::state() const {
  return state_;
}

bool Hcs08Adc::faulted() const {
  return !healthy_;
}

void Hcs08Adc::healthy() const {
  if (!healthy_) {
    fail("access", adc::status, "incomplete operation requires coordinated recovery");
  }
}

[[noreturn]] void Hcs08Adc::fail(const std::string &operation,
                                 const unsigned address,
                                 const std::string &reason) const {
  healthy_ = false;
  throw Fault("HCS08 ADC " + operation + " address=" + hex(address) + " pc=" + hex(context_.pc) +
              " ccr=" + hex(context_.ccr) + " remaining=" + std::to_string(state_.remaining) +
              " channel=" + std::to_string(state_.status & adc::channel_mask) + ": " + reason);
}
} // namespace blofeld
