// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/clock.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <optional>

namespace blofeld::vi::mcu {
namespace {
constexpr std::uint16_t first = 0x34, last = 0x3f;
constexpr std::uint16_t divider = 0x35, flags = 0x37, interrupts = 0x38, select = 0x39;
constexpr std::uint16_t rti_control = 0x3b, cop_control = 0x3c, test_control = 0x3e;
constexpr std::uint16_t usb_control = 0x300;
constexpr std::uint8_t byte_mask = 0xff, locked = 8, lock_flag = 0x10, rti_flag = 0x80;
constexpr std::uint8_t clearable = 0xd0, interrupt_mask = 0x90, select_mask = 0xb7;
constexpr std::uint8_t pll_select = 0x80, system_wait = 0x20, rti_wait = 2, cop_wait = 1;
constexpr std::uint8_t rti_mask = 0x7f, cop_mask = 0xc7;
constexpr std::uint16_t module_enable = 0x8000, clock_enable = 0x4000, suspend = 0x0800;
constexpr std::uint16_t usb_actions = 0x3200, phy_reset = 0x0400, descriptor_support = 0x10,
                        configuration_mask = 0x2f;
constexpr unsigned oscillator_period = 5, oscillator_bus_period = 10, phy_bus_period = 2;
constexpr unsigned qualification_ticks = 3 * oscillator_period;
constexpr std::uint16_t rti_vector = 0xfff0, lock_vector = 0xffc6;

Clock::Cycle ceiling(const Clock::Cycle value, const unsigned divisor) noexcept {
  return (value / divisor) + (value % divisor != 0 ? 1U : 0U);
}
}

bool Clock::contains(const std::uint16_t offset) noexcept {
  return (offset >= first && offset <= last) || offset == usb_control || offset == usb_control + 1;
}

bool Clock::read(const std::uint16_t offset, std::uint8_t &value) const noexcept {
  if (!contains(offset)) { return false; }
  switch (offset) {
    case divider: value = divider_; break;
    case flags: value = flags_; break;
    case interrupts: value = enable_; break;
    case select: value = select_; break;
    case rti_control: value = rti_; break;
    case cop_control: value = cop_; break;
    case test_control: value = pll_select; break; // Normal-mode factory register reads $80.
    case usb_control: value = static_cast<std::uint8_t>(usb_ >> 8U); break;
    case usb_control + 1: value = static_cast<std::uint8_t>(usb_ & byte_mask); break;
    default: value = 0; break;
  }
  return true;
}

unsigned Clock::bus_period() const noexcept {
  const auto pll = (select_ & pll_select) != 0 && (flags_ & locked) != 0;
  return (divider_ + 1U) * (pll ? phy_bus_period : oscillator_bus_period);
}

Clock::Cycle Clock::rti_period() const noexcept {
  const auto prescale = unsigned{rti_} >> 4U;
  return prescale == 0 ? 0
                       : (Cycle{1} << (9U + prescale)) * ((rti_ & 15U) + 1U) * oscillator_period;
}

bool Clock::rti_running() const noexcept {
  return activity_ != Activity::stopped &&
         (activity_ != Activity::waiting || (select_ & rti_wait) == 0);
}

bool Clock::bus_running() const noexcept {
  return activity_ != Activity::stopped &&
         (activity_ != Activity::waiting || (select_ & system_wait) == 0);
}

void Clock::activity(const Activity state) noexcept {
  if (state == Activity::waiting && activity_ != state && (select_ & rti_wait) != 0) {
    rti_phase_ = 0;
  }
  activity_ = state;
}

void Clock::lock(const bool ready) noexcept {
  if (((flags_ & locked) != 0) != ready) { flags_ |= lock_flag; }
  flags_ = static_cast<std::uint8_t>(ready ? flags_ | locked : flags_ & ~locked);
}

bool Clock::elapsed(const Cycle cycles) noexcept {
  const auto period = bus_period();
  if (cycles > (std::numeric_limits<Cycle>::max() - ticks_) / period) { return false; }
  const auto span = cycles * period;
  const auto rti = rti_period();
  if (rti != 0 && rti_running()) {
    const auto partial = rti_phase_ + (span % rti);
    if (span >= rti || partial >= rti) { flags_ |= rti_flag; }
    rti_phase_ = partial % rti;
  }
  ticks_ += span;
  time_ += cycles;
  return true;
}

bool Clock::advance(const Cycle now) noexcept {
  if (now < time_) { return false; }
  auto cycles = now - time_;
  if (qualification_) {
    const auto until = ceiling(*qualification_ - ticks_, bus_period());
    if (cycles >= until) {
      if (!elapsed(until)) { return false; }
      cycles -= until;
      qualification_.reset();
      lock(true);
    }
  }
  return elapsed(cycles);
}

std::optional<std::uint16_t> Clock::pending() const noexcept {
  if ((flags_ & enable_ & rti_flag) != 0) { return rti_vector; }
  if ((flags_ & enable_ & lock_flag) != 0) { return lock_vector; }
  return {};
}

bool Clock::interrupts_enabled() const noexcept {
  return enable_ != 0;
}

std::optional<Clock::Cycle> Clock::deadline() const noexcept {
  std::optional<Cycle> delta;
  if (qualification_) { delta = ceiling(*qualification_ - ticks_, bus_period()); }
  if (rti_period() != 0 && rti_running() && (enable_ & rti_flag) != 0 && (flags_ & rti_flag) == 0) {
    const auto next = ceiling(rti_period() - rti_phase_, bus_period());
    delta = delta ? std::min(*delta, next) : next;
  }
  if (!delta || *delta > std::numeric_limits<Cycle>::max() - time_) { return {}; }
  return time_ + *delta;
}

bool Clock::write_usb(const std::uint16_t value) noexcept {
  auto next = usb_;
  if ((usb_ & module_enable) == 0) {
    next = static_cast<std::uint16_t>((next & ~clock_enable) | (value & clock_enable));
  }
  if ((next & clock_enable) != 0) {
    if ((usb_ & module_enable) == 0) {
      if ((value & usb_actions) != 0 || ((value & phy_reset) != 0 && (select_ & pll_select) == 0)) {
        return false; // Reset/resume needs the USB owner.
      }
      next =
          static_cast<std::uint16_t>((next & ~configuration_mask) | (value & configuration_mask));
    }
    next = static_cast<std::uint16_t>((next & ~descriptor_support) | (value & descriptor_support));
    if ((select_ & pll_select) == 0) {
      next = static_cast<std::uint16_t>((next & ~suspend) | (value & suspend));
    }
  }
  next = static_cast<std::uint16_t>((next & ~module_enable) | (value & module_enable));
  if (((next ^ usb_) & suspend) != 0) {
    if ((next & suspend) == 0) {
      if (ticks_ > std::numeric_limits<Cycle>::max() - qualification_ticks) { return false; }
      qualification_ = ticks_ + qualification_ticks;
    } else {
      qualification_.reset();
      lock(false);
    }
  }
  usb_ = next;
  return true;
}

bool Clock::write_byte(const Write access) noexcept {
  const auto offset = access.offset;
  const auto value = static_cast<std::uint8_t>(access.value);
  if (!contains(offset)) { return false; }
  switch (offset) {
    case divider:
      if ((select_ & pll_select) == 0) { divider_ = value & 15U; }
      return true;
    case flags: flags_ = static_cast<std::uint8_t>(flags_ & ~(value & clearable)); return true;
    case interrupts: enable_ = value & interrupt_mask; return true;
    case select:
      select_ = static_cast<std::uint8_t>((value & (select_mask & ~cop_wait)) |
                                          ((cop_wait_written_ ? select_ : value) & cop_wait));
      cop_wait_written_ = true;
      return true;
    case rti_control:
      rti_ = value & rti_mask;
      rti_phase_ = 0;
      return true;
    case cop_control:
      if (!cop_written_) {
        if ((value & 7U) != 0) { return false; } // Watchdog reset/recovery is not yet connected.
        cop_ = value & cop_mask;
        cop_written_ = true;
      }
      return true;
    case usb_control:
      return write_usb(static_cast<std::uint16_t>((unsigned{value} << 8U) | (usb_ & byte_mask)));
    case usb_control + 1: return write_usb(static_cast<std::uint16_t>((usb_ & ~byte_mask) | value));
    default: return true; // Normal-mode reserved/test writes, or disabled COP service.
  }
}

bool Clock::write(const Write access) noexcept {
  if (access.bytes == 1) { return access.value <= byte_mask && write_byte(access); }
  if (access.bytes != 2) { return false; }
  if (access.offset == usb_control) { return write_usb(access.value); }
  return write_byte(
             {.offset = access.offset, .value = static_cast<std::uint16_t>(access.value >> 8U)}) &&
         write_byte({.offset = static_cast<std::uint16_t>(access.offset + 1),
                     .value = static_cast<std::uint16_t>(access.value & byte_mask)});
}
}
