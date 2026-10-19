// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/mcu_timer.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_gpio.hpp"

#include <array>
#include <cstdint>
#include <string_view>

namespace blofeld {
namespace {
constexpr unsigned base = hc12::timer_base;
constexpr unsigned channel_base = hc12::timer_channel0;
constexpr unsigned controls_size = 16;
constexpr unsigned channels_end = 0x60;
constexpr unsigned module_end = 0x70;
constexpr unsigned accumulator_end = 0x64;
constexpr unsigned counter_high = 4;
constexpr unsigned counter_low = 5;
constexpr unsigned system_control1 = 6;
constexpr unsigned overflow_toggle = 7;
constexpr unsigned output_control1 = 8;
constexpr unsigned output_control2 = 9;
constexpr unsigned edge_control1 = 10;
constexpr unsigned edge_control2 = 11;
constexpr unsigned interrupt_enable = 12;
constexpr unsigned system_control2 = 13;
constexpr unsigned channel_flags = 14;
constexpr unsigned overflow_flags = 15;
constexpr unsigned timer_enable = 0x80;
constexpr unsigned fast_clear = 0x10;
constexpr unsigned control1_mask = 0xf0;
constexpr unsigned control2_mask = 0x8f;
constexpr unsigned counter_reset = 8;
constexpr unsigned prescale_mask = 7;
constexpr unsigned prescaler_wrap = 128;
constexpr unsigned prescaler_mask = 127;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned word_mask = 0xffff;
constexpr unsigned overflow_flag = 0x80;
constexpr unsigned channel7 = 0x80;
constexpr unsigned maximum_clock_slice = 1048576;
constexpr std::array<std::string_view, controls_size> names{"TIOS",
                                                            "CFORC",
                                                            "OC7M",
                                                            "OC7D",
                                                            "TCNT high",
                                                            "TCNT low",
                                                            "TSCR1",
                                                            "TTOV",
                                                            "TCTL1",
                                                            "TCTL2",
                                                            "TCTL3",
                                                            "TCTL4",
                                                            "TIE",
                                                            "TSCR2",
                                                            "TFLG1",
                                                            "TFLG2"};
}

void McuTimer::reset() {
  *this = McuTimer{};
}

bool McuTimer::enabled() const {
  return (controls_[system_control1] & timer_enable) != 0;
}

std::string_view McuTimer::register_name(const std::uint16_t offset) {
  if (offset < base || offset >= module_end) { return {}; }
  if (offset < channel_base) { return names.at(offset - base); }
  if (offset < channels_end) { return "Timer channel high/low"; }
  if (offset < accumulator_end) { return "Timer pulse accumulator"; }
  return "Timer reserved/test in normal mode";
}

unsigned McuTimer::edge_mode(const unsigned channel) const {
  const auto control = channel < 4 ? edge_control2 : edge_control1;
  return (controls_[control] >> ((channel % 4) * 2)) & 3U;
}

unsigned McuTimer::output_mode(const unsigned channel) const {
  const auto control = channel < 4 ? output_control2 : output_control1;
  return (controls_[control] >> ((channel % 4) * 2)) & 3U;
}

unsigned McuTimer::capture_mask() const {
  if (!enabled()) { return 0; }
  unsigned mask = 0;
  for (unsigned channel = 0; channel < channel_count; ++channel) {
    const auto bit = 1U << channel;
    if ((controls_[0] & bit) == 0 && edge_mode(channel) != 0) { mask |= bit; }
  }
  return mask;
}

unsigned McuTimer::output_mask() const {
  unsigned mask = controls_[2];
  for (unsigned channel = 0; channel < channel_count; ++channel) {
    if (output_mode(channel) != 0) { mask |= 1U << channel; }
  }
  return mask & controls_[0];
}

PinFunction McuTimer::pins() const {
  const auto outputs = enabled() ? output_mask() : 0;
  return {.inputs = static_cast<std::uint8_t>(capture_mask()),
          .outputs = static_cast<std::uint8_t>(outputs),
          .levels = static_cast<std::uint8_t>(outputs_ & outputs & outputs_known_),
          .unknown = static_cast<std::uint8_t>(outputs & ~static_cast<unsigned>(outputs_known_))};
}

std::uint8_t McuTimer::read_channel(const std::uint16_t offset) {
  const auto relative = offset - channel_base;
  const auto index = relative / 2;
  const auto value = channels_.at(index);
  if ((controls_[system_control1] & fast_clear) != 0 && (controls_[0] & (1U << index)) == 0) {
    controls_[channel_flags] = static_cast<std::uint8_t>(controls_[channel_flags] & ~(1U << index));
  }
  return static_cast<std::uint8_t>((relative % 2 == 0 ? value >> 8U : value) & byte_mask);
}

std::uint8_t McuTimer::read(const std::uint16_t offset) {
  if (register_name(offset).empty()) { throw Fault("Unknown timer read at " + hex(offset)); }
  if (offset >= channel_base && offset < channels_end) { return read_channel(offset); }
  if (offset >= channels_end && offset < accumulator_end) {
    throw Fault("Unimplemented timer pulse accumulator read at " + hex(offset));
  }
  if (offset >= accumulator_end) { return 0; }
  const auto relative = offset - base;
  if (relative == counter_high || relative == counter_low) {
    if ((controls_[system_control1] & fast_clear) != 0) { controls_[overflow_flags] = 0; }
    return static_cast<std::uint8_t>((relative == counter_high ? counter_ >> 8U : counter_) &
                                     byte_mask);
  }
  if (relative == 1) { return 0; }
  return controls_.at(relative);
}

void McuTimer::write_channel(const ByteWrite write) {
  const auto relative = write.address - channel_base;
  const auto index = relative / 2;
  const auto bit = 1U << index;
  if ((controls_[0] & bit) == 0) { return; }
  auto &value = channels_.at(index);
  value = static_cast<std::uint16_t>(
      relative % 2 == 0 ? (static_cast<unsigned>(write.value) << 8U) | (value & byte_mask)
                        : (value & (word_mask ^ byte_mask)) | write.value);
  activation_pending_ |= bit;
  if ((controls_[system_control1] & fast_clear) != 0) {
    controls_[channel_flags] = static_cast<std::uint8_t>(controls_[channel_flags] & ~bit);
  }
}

void McuTimer::apply_output(const unsigned channel) {
  const auto bit = 1U << channel;
  const auto mode = output_mode(channel);
  if (mode == 0 || (channel == 7 && (controls_[2] & channel7) != 0)) { return; }
  unsigned level = outputs_;
  if (mode == 1) {
    if ((outputs_known_ & bit) == 0) { throw Fault("Unknown timer output latch before toggle"); }
    level ^= bit;
  } else if (mode == 2) {
    level &= ~bit;
  } else {
    level |= bit;
  }
  outputs_ = static_cast<std::uint8_t>(level);
  outputs_known_ = static_cast<std::uint8_t>(outputs_known_ | bit);
}

void McuTimer::output_actions(const unsigned matches) {
  const auto overridden = (matches & channel7) != 0 ? controls_[2] & controls_[0] : 0U;
  for (unsigned channel = 0; channel < channel_count; ++channel) {
    if ((matches & ~overridden & (1U << channel)) != 0) { apply_output(channel); }
  }
  if ((matches & channel7) != 0) {
    const auto mask = static_cast<unsigned>(controls_[2] & controls_[0]);
    outputs_ = static_cast<std::uint8_t>((outputs_ & ~mask) | (controls_[3] & mask));
    outputs_known_ = static_cast<std::uint8_t>(outputs_known_ | mask);
  }
}

void McuTimer::write(const ByteWrite write) {
  const auto offset = write.address;
  const auto value = write.value;
  if (register_name(offset).empty()) { throw Fault("Unknown timer write at " + hex(offset)); }
  if (offset >= channel_base && offset < channels_end) {
    write_channel(write);
    return;
  }
  if (offset >= channels_end && offset < accumulator_end) {
    throw Fault("Unimplemented timer pulse accumulator write at " + hex(offset));
  }
  if (offset >= accumulator_end) { return; }
  const auto relative = offset - base;
  switch (relative) {
    case 0: {
      const auto newly_output = static_cast<unsigned>(value) & ~static_cast<unsigned>(controls_[0]);
      controls_[0] = value;
      activation_pending_ |= newly_output;
      return;
    }
    case 1:
      if ((value & controls_[0] & channel7) != 0 &&
          (controls_[system_control2] & counter_reset) != 0) {
        throw Fault("Unverified timer force-7/counter-reset interaction");
      }
      output_actions(value & controls_[0]);
      return;
    case counter_high:
    case counter_low:
      if ((controls_[system_control1] & fast_clear) != 0) { controls_[overflow_flags] = 0; }
      return; // TCNT writes have no effect in normal mode.
    case system_control1:
      if (!enabled() && (value & timer_enable) != 0) { activation_pending_ |= controls_[0]; }
      controls_[system_control1] = static_cast<std::uint8_t>(value & control1_mask);
      return;
    case interrupt_enable: controls_[interrupt_enable] = value; return;
    case system_control2:
      controls_[system_control2] = static_cast<std::uint8_t>(value & control2_mask);
      if (prescaler_phase_ == 0) { active_prescale_ = value & prescale_mask; }
      return;
    case channel_flags:
      if (enabled()) {
        controls_[channel_flags] =
            static_cast<std::uint8_t>(controls_[channel_flags] & ~static_cast<unsigned>(value));
      }
      return;
    case overflow_flags:
      if (enabled()) {
        controls_[overflow_flags] =
            static_cast<std::uint8_t>(controls_[overflow_flags] & ~static_cast<unsigned>(value));
      }
      return;
    default: controls_.at(relative) = value; return;
  }
}

void McuTimer::observe_inputs(const TimerInputs inputs) {
  if ((inputs.levels & ~inputs.known) != 0) { throw Fault("Invalid timer input masks"); }
  const auto active = capture_mask();
  const auto known = static_cast<unsigned>(inputs_known_) & inputs.known;
  if ((active & ~known) != 0) {
    throw Fault("Unknown timer input level/history; prime inputs before enabling capture");
  }
  const auto changed = static_cast<unsigned>(inputs.levels ^ inputs_) & active;
  if (changed != 0 && !clocked_since_observation_) {
    throw Fault("Timer input transition requires an intervening bus clock");
  }
  for (unsigned channel = 0; channel < channel_count; ++channel) {
    const auto bit = 1U << channel;
    const auto mode = edge_mode(channel);
    const bool rising = (inputs.levels & bit) != 0;
    if ((changed & bit) != 0 && (mode == 3 || mode == (rising ? 1U : 2U))) {
      channels_[channel] = counter_;
      controls_[channel_flags] = static_cast<std::uint8_t>(controls_[channel_flags] | bit);
    }
  }
  inputs_ = inputs.levels;
  inputs_known_ = inputs.known;
  clocked_since_observation_ = false;
}

void McuTimer::check_activation() {
  const auto pending = activation_pending_ & controls_[0];
  for (unsigned channel = 0; channel < channel_count; ++channel) {
    if ((pending & (1U << channel)) != 0 && channels_[channel] == counter_) {
      throw Fault("Unverified timer comparator activation at the current TCNT");
    }
  }
  activation_pending_ = 0;
}

void McuTimer::count_tick() {
  const bool overflow = counter_ == word_mask;
  counter_ = static_cast<std::uint16_t>((static_cast<unsigned>(counter_) + 1) & word_mask);
  if (overflow) { controls_[overflow_flags] = overflow_flag; }
  unsigned matches = 0;
  for (unsigned channel = 0; channel < channel_count; ++channel) {
    if ((controls_[0] & (1U << channel)) != 0 && channels_[channel] == counter_) {
      matches |= 1U << channel;
    }
  }
  const auto overridden = (matches & channel7) != 0 ? controls_[2] & controls_[0] : 0U;
  const auto toggles = overflow ? controls_[overflow_toggle] & controls_[0] & ~overridden : 0U;
  if ((toggles & matches) != 0) {
    throw Fault("Unverified coincident timer overflow/output compare actions");
  }
  if ((toggles & ~static_cast<unsigned>(outputs_known_)) != 0) {
    throw Fault("Unknown timer output latch at overflow");
  }
  outputs_ = static_cast<std::uint8_t>(outputs_ ^ toggles);
  output_actions(matches);
  controls_[channel_flags] = static_cast<std::uint8_t>(controls_[channel_flags] | matches);
  if ((matches & channel7) != 0 && (controls_[system_control2] & counter_reset) != 0) {
    reset_pending_ = true;
  }
}

void McuTimer::advance(const unsigned bus_cycles) {
  if (bus_cycles > maximum_clock_slice) { throw Fault("Timer clock slice exceeds cycle limit"); }
  if (!enabled() || bus_cycles == 0) { return; }
  if ((capture_mask() & ~static_cast<unsigned>(inputs_known_)) != 0) {
    throw Fault("Unknown active timer capture inputs mask=" +
                hex(capture_mask() & ~static_cast<unsigned>(inputs_known_), 2));
  }
  check_activation();
  clocked_since_observation_ = true;
  for (unsigned cycle = 0; cycle < bus_cycles; ++cycle) {
    if (reset_pending_) {
      for (unsigned channel = 0; channel < channel_count; ++channel) {
        if ((controls_[0] & (1U << channel)) != 0 && channels_[channel] == 0) {
          throw Fault("Unverified timer compare-at-counter-reset phase");
        }
      }
      reset_pending_ = false;
      counter_ = 0;
      prescaler_phase_ = 0;
      active_prescale_ = controls_[system_control2] & prescale_mask;
      continue;
    }
    prescaler_phase_ = (prescaler_phase_ + 1) & prescaler_mask;
    if (prescaler_phase_ % (1U << active_prescale_) == 0) { count_tick(); }
    if (prescaler_phase_ == 0) { active_prescale_ = controls_[system_control2] & prescale_mask; }
  }
}

unsigned McuTimer::cycles_until_tick() const {
  if (!enabled()) { return 0; }
  if (reset_pending_) { return 1; }
  const auto divider = 1U << active_prescale_;
  const auto next_tick = divider - (prescaler_phase_ % divider);
  const auto prescale_update = prescaler_wrap - prescaler_phase_;
  return next_tick < prescale_update ? next_tick : prescale_update;
}

std::uint8_t McuTimer::interrupt_channels() const {
  return static_cast<std::uint8_t>(controls_[channel_flags] & controls_[interrupt_enable]);
}

bool McuTimer::overflow_interrupt() const {
  return (controls_[overflow_flags] & controls_[system_control2] & overflow_flag) != 0;
}

}
