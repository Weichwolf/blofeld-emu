// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/timer.hpp"

#include <algorithm>
#include <bit>
#include <cstdint>
#include <limits>
#include <optional>

namespace blofeld::vi::mcu {
namespace {
constexpr std::uint16_t base = 0x40, end = 0x70;
constexpr unsigned select = 0, force = 1, counter_high = 4, counter_low = 5;
constexpr unsigned control1 = 6, interrupt_enable = 12, control2 = 13, flags1 = 14, flags2 = 15;
constexpr unsigned first_channel = 16, channel_end = 32;
constexpr unsigned pulse_control = 32, pulse_high = 34, pulse_low = 35;
constexpr unsigned timer_enable = 0x80, wait_disable = 0x40, fast_clear = 0x10;
constexpr unsigned overflow_enable = 0x80, counter_reset = 8, prescale_mask = 7;
constexpr unsigned counter_period = 0x10000, word_mask = 0xffff, byte_mask = 0xff;
constexpr unsigned divider_period = 128, channel0_vector = 0xffee, overflow_vector = 0xffde;
constexpr unsigned pulse_enable = 0x40, pulse_mask = 0x7f;

std::uint64_t distance(const std::uint16_t counter, const std::uint16_t compare) noexcept {
  const auto difference = (static_cast<unsigned>(compare) + counter_period - counter) & word_mask;
  return difference == 0 ? counter_period : difference;
}

std::uint16_t
replace_byte(const std::uint16_t word, const unsigned offset, const std::uint8_t value) noexcept {
  return static_cast<std::uint16_t>((offset & 1U) == 0
                                        ? (word & byte_mask) | (unsigned{value} << 8U)
                                        : (word & ~byte_mask) | value);
}
}

bool Timer::contains(const std::uint16_t offset) noexcept {
  return offset >= base && offset < end;
}

bool Timer::active() const noexcept {
  return (registers_[control1] & timer_enable) != 0 && activity_ != Activity::stopped &&
         (activity_ != Activity::waiting || (registers_[control1] & wait_disable) == 0);
}

void Timer::activity(const Activity state) noexcept {
  activity_ = state;
}

void Timer::count(const Cycle ticks) noexcept {
  auto selected = static_cast<unsigned>(registers_[select]);
  while (selected != 0) {
    const auto channel = static_cast<unsigned>(std::countr_zero(selected));
    selected &= selected - 1;
    const auto bit = 1U << channel;
    if (ticks >= distance(counter_, channels_[channel])) {
      registers_[flags1] = static_cast<std::uint8_t>(registers_[flags1] | bit);
    }
  }
  if (ticks >= counter_period - counter_) { registers_[flags2] = overflow_enable; }
  counter_ = static_cast<std::uint16_t>((counter_ + (ticks % counter_period)) & word_mask);
}

void Timer::elapsed(const Cycle cycles) noexcept {
  const Cycle divisor = Cycle{1} << prescale_;
  const auto ticks = (cycles / divisor) + (((phase_ % divisor) + (cycles % divisor)) / divisor);
  phase_ = static_cast<unsigned>((phase_ + (cycles % divider_period)) % divider_period);
  count(ticks);
}

bool Timer::advance(const Cycle now) noexcept {
  if (now < time_) { return false; }
  auto cycles = now - time_;
  time_ = now;
  if (!active() || cycles == 0) { return true; }
  const auto selected = registers_[control2] & prescale_mask;
  if (prescale_ != selected) {
    // PR changes latch only when every divider stage reaches zero.
    const auto remaining = divider_period - phase_;
    if (cycles >= remaining) {
      elapsed(remaining);
      cycles -= remaining;
      prescale_ = selected;
    }
  }
  elapsed(cycles);
  return true;
}

void Timer::clear_channel(const unsigned channel, const bool writing) noexcept {
  const auto bit = 1U << channel;
  if ((registers_[control1] & fast_clear) != 0 && ((registers_[select] & bit) != 0) == writing) {
    registers_[flags1] = static_cast<std::uint8_t>(registers_[flags1] & ~bit);
  }
}

bool Timer::read_byte(const std::uint16_t offset, std::uint8_t &value) noexcept {
  if (!contains(offset)) { return false; }
  const auto index = static_cast<unsigned>(offset - base);
  if (index == counter_high || index == counter_low) {
    value = static_cast<std::uint8_t>((index & 1U) == 0 ? counter_ >> 8U : counter_ & byte_mask);
    if ((registers_[control1] & fast_clear) != 0) { registers_[flags2] = 0; }
  } else if (index >= first_channel && index < channel_end) {
    const auto channel = (index - first_channel) / 2;
    value = static_cast<std::uint8_t>((index & 1U) == 0 ? channels_[channel] >> 8U
                                                        : channels_[channel] & byte_mask);
    clear_channel(channel, false);
  } else if (index < registers_.size()) {
    value = index == force ? 0 : registers_[index];
  } else if (index == pulse_control) {
    value = pulse_control_;
  } else if (index == pulse_high || index == pulse_low) {
    value = static_cast<std::uint8_t>((index & 1U) == 0 ? pulse_count_ >> 8U
                                                        : pulse_count_ & byte_mask);
  } else {
    value = 0;
  }
  return true;
}

bool Timer::read(const Read access, std::uint16_t &value) noexcept {
  if (access.bytes != 1 && access.bytes != 2) { return false; }
  std::uint8_t high = 0;
  std::uint8_t low = 0;
  if (!read_byte(access.offset, high)) { return false; }
  value = high;
  if (access.bytes == 1) { return true; }
  if (!read_byte(static_cast<std::uint16_t>(access.offset + 1), low)) { return false; }
  value = static_cast<std::uint16_t>((unsigned{high} << 8U) | low);
  return true;
}

bool Timer::write_control(const unsigned index, const std::uint8_t value) noexcept {
  if (index == flags1 || index == flags2) {
    if ((registers_[control1] & timer_enable) != 0) {
      registers_[index] = static_cast<std::uint8_t>(registers_[index] & ~value);
    }
    return true;
  }
  if (index == control1) {
    constexpr unsigned implemented = 0xf0;
    registers_[index] = static_cast<std::uint8_t>(value & implemented);
    return true;
  }
  if (index == control2) {
    if ((value & counter_reset) != 0) { return false; }
    registers_[index] = static_cast<std::uint8_t>(value & (overflow_enable | prescale_mask));
    if (phase_ == 0) { prescale_ = value & prescale_mask; }
    return true;
  }
  if (index == force) { return true; }
  // Compare output routing is not connected. Refuse enabling it rather than
  // silently accepting a configuration that cannot affect the GPIO registers.
  if (index == 2 || index == 7 || index == 8 || index == 9) {
    if (value != 0) { return false; }
  }
  registers_[index] = value;
  return true;
}

bool Timer::write_byte(const Write access) noexcept {
  if (!contains(access.offset)) { return false; }
  const auto value = static_cast<std::uint8_t>(access.value);
  const auto index = static_cast<unsigned>(access.offset - base);
  if (index == counter_high || index == counter_low) {
    if ((registers_[control1] & fast_clear) != 0) { registers_[flags2] = 0; }
    return true; // TCNT writes have no effect in normal mode.
  }
  if (index >= first_channel && index < channel_end) {
    const auto channel = (index - first_channel) / 2;
    if ((registers_[select] & (1U << channel)) != 0) {
      channels_[channel] = replace_byte(channels_[channel], index, value);
      clear_channel(channel, true);
    }
    return true;
  }
  if (index < registers_.size()) { return write_control(index, value); }
  if (index == pulse_control) {
    if ((value & pulse_enable) != 0) { return false; }
    pulse_control_ = static_cast<std::uint8_t>(value & pulse_mask);
    return true;
  }
  if (index == pulse_high || index == pulse_low) {
    pulse_count_ = replace_byte(pulse_count_, index, value);
    return true;
  }
  return true;
}

bool Timer::write(const Write access) noexcept {
  if (access.bytes != 1 && access.bytes != 2) { return false; }
  if (access.bytes == 1) { return access.value <= byte_mask && write_byte(access); }
  // Both bytes observe one device time; no comparison occurs between halves.
  return write_byte(
             {.offset = access.offset, .value = static_cast<std::uint16_t>(access.value >> 8U)}) &&
         write_byte({.offset = static_cast<std::uint16_t>(access.offset + 1),
                     .value = static_cast<std::uint16_t>(access.value & byte_mask)});
}

bool Timer::capture(const Capture edge) noexcept {
  if (edge.channel >= channels_.size()) { return false; }
  const auto bit = 1U << edge.channel;
  const auto index = 11 - (edge.channel / 4);
  const auto edges = (registers_[index] >> (2 * (edge.channel % 4))) & 3U;
  const auto selected = edge.rising ? 1U : 2U;
  if (active() && (registers_[select] & bit) == 0 && (edges & selected) != 0) {
    channels_[edge.channel] = counter_;
    registers_[flags1] = static_cast<std::uint8_t>(registers_[flags1] | bit);
  }
  return true;
}

bool Timer::interrupts_enabled() const noexcept {
  return registers_[interrupt_enable] != 0 || (registers_[control2] & overflow_enable) != 0;
}

std::optional<std::uint16_t> Timer::pending() const noexcept {
  const auto ready = static_cast<unsigned>(registers_[flags1] & registers_[interrupt_enable]);
  if (ready != 0) {
    const auto channel = static_cast<unsigned>(std::countr_zero(ready));
    return static_cast<std::uint16_t>(channel0_vector - (2 * channel));
  }
  if ((registers_[flags2] & registers_[control2] & overflow_enable) != 0) {
    return overflow_vector;
  }
  return {};
}

Timer::Cycle Timer::clocks(const Cycle ticks) const noexcept {
  const Cycle divisor = Cycle{1} << prescale_;
  const auto selected = registers_[control2] & prescale_mask;
  const auto until_sync = divider_period - phase_;
  const auto before_sync = (divider_period / divisor) - (phase_ / divisor);
  if (selected != prescale_ && ticks > before_sync) {
    return until_sync + ((ticks - before_sync) * (Cycle{1} << selected));
  }
  return (ticks * divisor) - (phase_ % divisor);
}

std::optional<Timer::Cycle> Timer::deadline() const noexcept {
  if (!active()) { return {}; }
  auto ticks = std::numeric_limits<Cycle>::max();
  auto channels = static_cast<unsigned>(registers_[select] & registers_[interrupt_enable]) &
                  ~static_cast<unsigned>(registers_[flags1]);
  while (channels != 0) {
    const auto channel = static_cast<unsigned>(std::countr_zero(channels));
    channels &= channels - 1;
    ticks = std::min(ticks, distance(counter_, channels_[channel]));
  }
  if ((registers_[control2] & overflow_enable) != 0 && registers_[flags2] == 0) {
    ticks = std::min<Cycle>(ticks, counter_period - counter_);
  }
  if (ticks == std::numeric_limits<Cycle>::max()) { return {}; }
  const auto delta = clocks(ticks);
  if (delta > std::numeric_limits<Cycle>::max() - time_) { return {}; }
  return time_ + delta;
}
}
