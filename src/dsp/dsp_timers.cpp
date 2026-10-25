// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_timers.hpp"

#include "common/bus.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace blofeld {
namespace {
constexpr unsigned first = 0xffff82;
constexpr unsigned tplr = 0xffff83;
constexpr unsigned first_timer = 0xffff84;
constexpr unsigned last = 0xffff8f;
constexpr unsigned te = 1;
constexpr unsigned toie = 2;
constexpr unsigned tcie = 4;
constexpr unsigned mode_mask = 0xf0;
constexpr unsigned event_mode = 0x30;
constexpr unsigned capture_mode = 0x60;
constexpr unsigned inv = 0x100;
constexpr unsigned trm = 0x200;
constexpr unsigned dir = 0x800;
constexpr unsigned di = 0x1000;
constexpr unsigned data_out = 0x2000;
constexpr unsigned pce = 0x8000;
constexpr unsigned tof = 0x100000;
constexpr unsigned tcf = 0x200000;
constexpr unsigned writable = te | toie | tcie | mode_mask | inv | trm | dir | data_out | pce;
constexpr unsigned allowed = writable | di | tof | tcf;
constexpr unsigned preload_mask = 0x1fffff;
constexpr unsigned compare_vector = 0x54;
constexpr unsigned overflow_vector = 0x56;

std::size_t index_of(const std::uint32_t address) {
  return (last - address) / 4;
}
}

DspTimers::DspTimers() {
  reset();
}

void DspTimers::reset() {
  timers_ = {};
  prescaler_load_ = 0;
  prescaler_count_ = 0;
}

bool DspTimers::handles(const std::uint32_t address) {
  return address >= first && address <= last;
}

bool DspTimers::is_control(const std::uint32_t address) {
  return address >= first_timer && handles(address) && (address & 3U) == 3;
}

DspTimerPin DspTimers::observe_tio(const unsigned index) const {
  if (index >= external_.size()) {
    throw Fault("DSP timer has no TIO pin at index " + hex(index, 1));
  }
  const auto control = timers_.at(index).control;
  if ((control & mode_mask) != 0 || (control & dir) == 0) { return {.level = external_.at(index)}; }
  const bool level = ((control & data_out) != 0) != ((control & inv) != 0);
  const auto &external = external_.at(index);
  if (external.has_value() && external.value() != level) {
    throw Fault("DSP timer TIO contention at X:" + hex(last - (index * 4), 6));
  }
  return {.output = true, .level = level};
}

void DspTimers::set_tio_drive(const unsigned index, const std::optional<bool> level) {
  if (index >= external_.size()) { throw Fault("Invalid DSP timer TIO drive index"); }
  auto next = *this;
  if (next.external_.at(index) != level) { next.timers_.at(index).synchronized_qualified = false; }
  next.external_.at(index) = level;
  static_cast<void>(next.observe_tio(index));
  *this = next;
}

std::uint32_t DspTimers::read(const std::uint32_t address, const std::uint32_t mask) const {
  if (!handles(address) || (mask & ~DspTimerWrite::word_mask) != 0) {
    throw Fault("Invalid DSP timer read at X:" + hex(address, 6));
  }
  if (address == first) { return prescaler_count_ & mask; }
  if (address == tplr) { return prescaler_load_ & mask; }
  const auto index = index_of(address);
  const auto &timer = timers_.at(index);
  switch (address & 3U) {
    case 0:
      if ((timer.control & mode_mask) == capture_mode) { return timer.captured & mask; }
      return timer.count & mask;
    case 1:
      if (!timer.compare.has_value()) {
        throw Fault("Unknown DSP timer compare read at X:" + hex(address, 6));
      }
      return timer.compare.value() & mask;
    case 2: throw Fault("Write-only DSP timer load read at X:" + hex(address, 6));
    case 3: {
      auto value = timer.control | timer.flags;
      if ((mask & di) != 0 && index < external_.size()) {
        const auto pin = observe_tio(static_cast<unsigned>(index));
        if (!pin.level.has_value()) {
          throw Fault("Unknown DSP timer DI read at X:" + hex(address, 6));
        }
        if (pin.level.value() != ((timer.control & inv) != 0)) { value |= di; }
      }
      return value & mask;
    }
    default: throw Fault("Invalid DSP timer register read");
  }
}

void DspTimers::validate_control(const std::size_t index, const std::uint32_t value) const {
  const auto &timer = timers_.at(index);
  const auto mode = value & mode_mask;
  const auto previous_mode = timer.control & mode_mask;
  if ((value & ~allowed) != 0 || (mode != 0 && mode != event_mode && mode != capture_mode) ||
      (mode == event_mode && index == 2 && (value & pce) == 0) ||
      (mode == capture_mode && (index == 2 || (value & (trm | dir | data_out)) != 0)) ||
      (index == 2 && (value & (dir | data_out | inv)) != 0)) {
    throw Fault("Unsupported DSP timer control write at X:" + hex(last - (index * 4), 6) +
                " value=" + hex(value, 6));
  }
  const bool active = (timer.control & value & te) != 0;
  // Ordered register writes do not clock the counter. Clearing TE disables it
  // before another mode can observe an edge; retaining TE cannot change TC.
  if (active && previous_mode != mode) {
    throw Fault("Unqualified active DSP timer mode change at X:" + hex(last - (index * 4), 6));
  }
  if (mode == capture_mode && (timer.control & value & te) != 0 &&
      ((timer.control ^ value) & inv) != 0) {
    throw Fault("Unqualified active DSP timer capture polarity change at X:" +
                hex(last - (index * 4), 6));
  }
  if (active && mode == event_mode && (value & pce) == 0 && ((timer.control ^ value) & inv) != 0) {
    throw Fault("Unqualified active DSP timer event polarity change at X:" +
                hex(last - (index * 4), 6));
  }
  if (active && ((timer.control ^ value) & pce) != 0) {
    throw Fault("Unqualified active DSP timer PCE write at X:" + hex(last - (index * 4), 6));
  }
  if (active && ((timer.control ^ value) & trm) != 0 &&
      (timer.reload_pending ||
       (!timer.first_clock && (!timer.compare || timer.count == *timer.compare)))) {
    throw Fault("Unqualified active DSP timer TRM change at compare/reload boundary X:" +
                hex(last - (index * 4), 6));
  }
}

void DspTimers::write_control(const std::size_t index, const std::uint32_t value) {
  validate_control(index, value);
  auto &timer = timers_.at(index);
  const auto previous_mode = timer.control & mode_mask;
  const auto mode = value & mode_mask;
  const bool was_prescaled = prescaler_enabled();
  const bool enable = (timer.control & te) == 0 && (value & te) != 0;
  if (previous_mode != mode) {
    if (mode == capture_mode) { timer.captured = timer.count; }
  }
  timer.control = value & writable;
  timer.flags &= ~(value & (tof | tcf));
  if ((value & te) == 0) {
    timer.flags = 0;
    timer.first_clock = false;
    timer.reload_pending = false;
  } else if (enable) {
    timer.count = 0;
    timer.first_clock = true;
    timer.reload_pending = false;
    timer.halted = false;
  }
  if (!was_prescaled && prescaler_enabled()) { prescaler_count_ = prescaler_load_; }
  if (index < external_.size()) { static_cast<void>(observe_tio(static_cast<unsigned>(index))); }
}

void DspTimers::write_checked(const DspTimerWrite access) {
  if (!handles(access.address) || access.value > DspTimerWrite::word_mask) {
    throw Fault("Invalid DSP timer write at X:" + hex(access.address, 6));
  }
  if (access.address == first) {
    throw Fault("Read-only DSP prescaler count write at X:" + hex(first, 6));
  }
  if (access.address == tplr) {
    if ((access.value & ~preload_mask) != 0) {
      throw Fault("Unsupported DSP prescaler source/reserved write at X:" + hex(tplr, 6) +
                  " value=" + hex(access.value, 6));
    }
    prescaler_load_ = access.value;
    return;
  }
  auto &timer = timers_.at(index_of(access.address));
  switch (access.address & 3U) {
    case 0: throw Fault("Read-only DSP timer count write at X:" + hex(access.address, 6));
    case 1: timer.compare = access.value; break;
    case 2: timer.load = access.value; break;
    case 3: write_control(index_of(access.address), access.value); break;
    default: throw Fault("Invalid DSP timer register write");
  }
}

void DspTimers::write(const DspTimerWrite access) {
  auto next = *this;
  next.write_checked(access);
  *this = next;
}

std::vector<std::uint32_t> DspTimers::interrupt_vectors() const {
  std::vector<std::uint32_t> vectors;
  for (std::size_t index = 0; index < timers_.size(); ++index) {
    const auto &timer = timers_.at(index);
    if ((timer.control & toie) != 0 && (timer.flags & tof) != 0) {
      vectors.push_back(overflow_vector + (static_cast<unsigned>(index) * 4));
    }
    if ((timer.control & tcie) != 0 && (timer.flags & tcf) != 0) {
      vectors.push_back(compare_vector + (static_cast<unsigned>(index) * 4));
    }
  }
  return vectors;
}

void DspTimers::acknowledge_interrupt(const std::uint32_t vector) {
  if (vector < compare_vector || vector > overflow_vector + 8 || (vector & 1U) != 0) {
    throw Fault("Invalid DSP timer interrupt acknowledgement");
  }
  auto &timer = timers_.at((vector - compare_vector) / 4);
  const bool overflow = ((vector - compare_vector) & 2U) != 0;
  const auto flag = overflow ? tof : tcf;
  const auto enable = overflow ? toie : tcie;
  if ((timer.control & enable) == 0 || (timer.flags & flag) == 0) {
    throw Fault("DSP timer acknowledgement has no pending enabled source");
  }
  timer.flags &= ~flag;
}

}
