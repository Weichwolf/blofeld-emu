// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/timers.hpp"

#include <algorithm>
#include <cstdint>
#include <dsp56300/memory.hpp>
#include <limits>
#include <optional>
#include <utility>

namespace blofeld::vi::dsp {
namespace {
using dsp56300::Word;
constexpr Word prescaler_count_address = 0xffff82, prescaler_address = 0xffff83;
constexpr Word last_control = 0xffff8f, first_counter = 0xffff84;
constexpr Word enable = 1, overflow_irq = 2, compare_irq = 4, invert = 0x100, reload = 0x200;
constexpr Word input_bit = 0x1000, output_bit = 0x2000, direction = 0x800, use_prescaler = 0x8000;
constexpr Word overflow = 0x100000, compare = 0x200000, flags = overflow | compare;
constexpr Word control_mask = 0x30abf7, prescaler_mask = 0x1fffff,
               prescaler_register_mask = 0x7fffff;
constexpr unsigned prescaler_source_shift = 21;
constexpr std::uint64_t modulus = 0x1000000;

unsigned mode(const Word control) noexcept {
  return (control >> 4U) & 15;
}

std::uint64_t distance(const Word count, const Word target) noexcept {
  const auto delta = (std::uint64_t{target} + modulus - count) & dsp56300::word_mask;
  return delta != 0 ? delta : modulus;
}

std::uint64_t next_event(const Word count, const Word target, const unsigned selected) noexcept {
  const auto wrap = modulus - count;
  return selected == 6 ? wrap : std::min(wrap, distance(count, target));
}
}

bool Timers::fail(const char *message) noexcept {
  error_ = message;
  return false;
}

void Timers::reset() noexcept {
  for (auto &timer : timers_) {
    timer.control = timer.count = timer.captured = 0;
    timer.preload = timer.halted = timer.level = false;
  }
  prescaler_ = prescaler_count_ = 0;
  cycles_ = 0;
  requests_ = 0;
  error_ = nullptr;
}

bool Timers::contains(const Word address) noexcept {
  return address >= prescaler_count_address && address <= last_control;
}

bool Timers::read(const Word address, Word &value) const noexcept {
  if (address == prescaler_address) {
    value = prescaler_;
    return true;
  }
  if (address == prescaler_count_address) {
    value = prescaler_count_;
    return true;
  }
  if (!contains(address)) { return false; }
  const auto &timer = timers_[(last_control - address) / 4];
  switch ((address - first_counter) & 3) {
    case 0: value = mode(timer.control) == 6 ? timer.captured : timer.count; return true;
    case 1: value = timer.compare; return true;
    case 2: return false; // TLR is write-only.
    case 3: {
      const bool driven = mode(timer.control) == 0 && (timer.control & direction) != 0;
      const auto level = driven ? (timer.control & output_bit) != 0 : timer.level;
      value = (timer.control & ~input_bit) | (level ? input_bit : 0);
      // GPIO output and input inversion cancel in its own readback path.
      if (!driven && (timer.control & invert) != 0) { value ^= input_bit; }
      return true;
    }
    default: return false;
  }
}

bool Timers::prescaler_active() const noexcept {
  return std::ranges::any_of(timers_, [](const Timer &timer) {
    return (timer.control & (enable | use_prescaler)) == (enable | use_prescaler);
  });
}

bool Timers::control(const unsigned index, const Word value) noexcept {
  const auto selected = mode(value);
  if ((value & enable) != 0 &&
      (selected != 0 && ((selected != 3 && selected != 6) || index == 2))) {
    return fail("DSP timer mode is not implemented");
  }
  if ((value & (enable | use_prescaler)) == (enable | use_prescaler) &&
      (prescaler_ >> prescaler_source_shift) == 3) {
    return fail("Reserved DSP timer prescaler source");
  }
  const bool prescaled_before = prescaler_active();
  auto &timer = timers_[index];
  const auto previous = timer.control;
  timer.control = (value & control_mask & ~flags) | (previous & flags & ~value);
  if (selected != 0) { timer.control = (timer.control & ~output_bit) | (previous & output_bit); }
  if ((timer.control & enable) == 0) {
    timer.control &= ~flags;
    requests_ &= ~(1U << index);
  } else if ((previous & enable) == 0) {
    timer.count = timer.captured = 0;
    timer.preload = true;
    timer.halted = false;
  }
  if (!prescaled_before && prescaler_active()) { prescaler_count_ = prescaler_ & prescaler_mask; }
  return true;
}

bool Timers::write(const Write access) noexcept {
  if (!contains(access.address) || access.value > dsp56300::word_mask || error_ != nullptr) {
    return false;
  }
  if (access.address == prescaler_address) {
    if ((access.value >> prescaler_source_shift & 3) == 3 && prescaler_active()) {
      return fail("Reserved DSP timer prescaler source");
    }
    prescaler_ = access.value & prescaler_register_mask;
    return true;
  }
  if (access.address == prescaler_count_address) { return true; }
  const auto index = (last_control - access.address) / 4;
  auto &timer = timers_[index];
  switch ((access.address - first_counter) & 3) {
    case 0: return true; // TCR is read-only.
    case 1: timer.compare = access.value; return true;
    case 2: timer.load = access.value; return true;
    case 3: return control(index, access.value);
    default: return false;
  }
}

std::uint64_t Timers::prescale(std::uint64_t ticks) noexcept {
  if (ticks <= prescaler_count_) {
    prescaler_count_ -= static_cast<Word>(ticks);
    return 0;
  }
  ticks -= prescaler_count_ + 1;
  const auto load = prescaler_ & prescaler_mask;
  const auto period = std::uint64_t{load} + 1;
  prescaler_count_ = load - static_cast<Word>(ticks % period);
  return 1 + (ticks / period);
}

void Timers::event(const unsigned index, const Word value) noexcept {
  timers_[index].control |= value;
  if ((value & compare) != 0) { requests_ |= 1U << index; }
}

void Timers::count(const unsigned index, std::uint64_t ticks) noexcept {
  auto &timer = timers_[index];
  const auto selected = mode(timer.control);
  if ((timer.control & enable) == 0 || timer.halted) { return; }
  while (ticks != 0) {
    if (timer.preload) {
      timer.count = timer.load;
      timer.preload = false;
      --ticks;
      if (selected != 6 && timer.count == timer.compare) {
        event(index, compare);
        timer.preload = (timer.control & reload) != 0;
      }
      continue;
    }
    const auto until_wrap = modulus - timer.count;
    const auto until_compare = distance(timer.count, timer.compare);
    const auto step = std::min(ticks, next_event(timer.count, timer.compare, selected));
    timer.count = static_cast<Word>((timer.count + step) & dsp56300::word_mask);
    ticks -= step;
    if (step == until_wrap) { event(index, overflow); }
    if (selected != 6 && step == until_compare) {
      event(index, compare);
      timer.preload = (timer.control & reload) != 0;
    }
  }
}

bool Timers::advance(const std::uint64_t cycles) noexcept {
  if (error_ != nullptr) { return false; }
  if (cycles < cycles_) { return fail("DSP timer clock moved backwards"); }
  const auto ticks = (cycles / 2) - (cycles_ / 2);
  cycles_ = cycles;
  const auto divided =
      prescaler_active() && (prescaler_ >> prescaler_source_shift) == 0 ? prescale(ticks) : 0;
  for (unsigned index = 0; index < timers_.size(); ++index) {
    const auto &timer = timers_[index];
    if ((timer.control & use_prescaler) != 0) {
      count(index, divided);
    } else if (mode(timer.control) != 3) {
      count(index, ticks);
    }
  }
  return true;
}

bool Timers::input(const unsigned index, const bool level) noexcept {
  if (error_ != nullptr) { return false; }
  if (index >= timers_.size()) { return fail("Unknown DSP timer input"); }
  auto &timer = timers_[index];
  const auto previous = timer.level;
  timer.level = level;
  if (previous == level) { return true; }
  const auto source = prescaler_ >> prescaler_source_shift;
  if (level && source == index + 1 && prescaler_active()) {
    const auto ticks = prescale(1);
    for (unsigned target = 0; target < timers_.size(); ++target) {
      if ((timers_[target].control & use_prescaler) != 0) { count(target, ticks); }
    }
  }
  if ((timer.control & enable) == 0 || level == ((timer.control & invert) != 0)) { return true; }
  if (mode(timer.control) == 3 && (timer.control & use_prescaler) == 0) { count(index, 1); }
  if (mode(timer.control) == 6 && !timer.halted) {
    timer.captured = timer.count;
    timer.halted = true;
    event(index, compare);
  }
  return true;
}

std::optional<std::uint64_t> Timers::deadline() const noexcept {
  std::optional<std::uint64_t> result;
  for (const auto &timer : timers_) {
    if ((timer.control & enable) == 0 || timer.halted) { continue; }
    const bool prescaled = (timer.control & use_prescaler) != 0;
    if (prescaled ? (prescaler_ >> prescaler_source_shift) != 0 : mode(timer.control) == 3) {
      continue;
    }
    const auto initial = timer.preload ? timer.load : timer.count;
    auto ticks = next_event(initial, timer.compare, mode(timer.control));
    if (timer.preload) {
      ticks = mode(timer.control) != 6 && timer.load == timer.compare ? 1 : ticks + 1;
    }
    if (prescaled) {
      ticks = prescaler_count_ + 1 + (ticks - 1) * ((prescaler_ & prescaler_mask) + 1ULL);
    }
    const auto base = cycles_ / 2;
    if (ticks > std::numeric_limits<std::uint64_t>::max() / 2 - base) { continue; }
    const auto next = (base + ticks) * 2;
    if (!result || next < *result) { result = next; }
  }
  return result;
}

std::optional<Word> Timers::interrupt() const noexcept {
  constexpr Word compare_vector = 0x54;
  for (unsigned index = 0; index < timers_.size(); ++index) {
    const auto control = timers_[index].control;
    if ((control & enable) == 0) { continue; }
    const auto vector = compare_vector + (index * 4);
    if ((control & overflow) != 0 && (control & overflow_irq) != 0) { return vector + 2; }
    if ((control & compare) != 0 && (control & compare_irq) != 0) { return vector; }
  }
  return {};
}

unsigned Timers::take_requests() noexcept {
  return std::exchange(requests_, 0);
}
}
