// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_timers.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
namespace {
// DSP56371 timer CSR: enable, overflow/compare IRQ enables, mode, reload,
// prescaler enable, and latched overflow/compare flags. Counters are 24 bits.
constexpr unsigned enable = 1;
constexpr unsigned overflow_irq = 2;
constexpr unsigned compare_irq = 4;
constexpr unsigned mode_mask = 0xf0;
constexpr unsigned event_mode = 0x30;
constexpr unsigned capture_mode = 0x60;
constexpr unsigned reload = 0x200;
constexpr unsigned prescale = 0x8000;
constexpr unsigned overflow = 0x100000;
constexpr unsigned compare_flag = 0x200000;
constexpr std::uint64_t counter_range = 0x1000000;

void earliest(std::optional<std::uint64_t> &current,
              const std::optional<std::uint64_t> candidate) noexcept {
  if (candidate && (!current || *candidate < *current)) { current = candidate; }
}
}

std::optional<std::uint64_t> DspTimers::timer_boundary(const Timer &timer) noexcept {
  const auto control = timer.control;
  auto interrupts = control & (overflow_irq | compare_irq);
  if ((timer.flags & overflow) != 0) { interrupts &= ~overflow_irq; }
  if ((timer.flags & compare_flag) != 0) { interrupts &= ~compare_irq; }
  if ((control & enable) == 0) { return std::nullopt; }
  const bool synchronized = timer.synchronized && timer.synchronized_qualified;
  if ((control & (mode_mask | prescale)) == event_mode) {
    // advance_clk2 validates this input, but only actual TIO edges count it.
    return synchronized ? std::nullopt : std::optional<std::uint64_t>{1};
  }
  if ((control & mode_mask) == capture_mode) { return capture_boundary(timer, interrupts); }
  return counting_boundary(timer, interrupts);
}

std::optional<std::uint64_t> DspTimers::capture_boundary(const Timer &timer,
                                                         const unsigned interrupts) noexcept {
  if (timer.halted) { return std::nullopt; }
  if (!timer.synchronized || !timer.synchronized_qualified || (timer.first_clock && !timer.load)) {
    return 1;
  }
  if ((interrupts & overflow_irq) == 0) { return std::nullopt; }
  const auto count = timer.first_clock ? *timer.load : timer.count;
  return (timer.first_clock ? 1 : 0) + counter_range - count;
}

std::optional<std::uint64_t> DspTimers::counting_boundary(const Timer &timer,
                                                          const unsigned interrupts) noexcept {
  if (!timer.load || !timer.compare) { return 1; }
  const auto control = timer.control;
  const bool initial = timer.first_clock || timer.reload_pending;
  const auto count = initial ? *timer.load : timer.count;
  const std::uint64_t prefix = initial ? 1 : 0;
  const auto to_wrap = counter_range - count;
  const auto compare = *timer.compare;
  const auto distance = (compare + counter_range - count) % counter_range;
  const auto to_compare = distance == 0 ? counter_range : distance;
  const bool initial_match = initial && count == compare;
  const bool repeating_match = initial_match && (control & reload) != 0;
  std::optional<std::uint64_t> result;
  if ((interrupts & compare_irq) != 0) { result = initial_match ? prefix : prefix + to_compare; }
  if ((interrupts & overflow_irq) != 0 && !repeating_match) {
    if ((control & reload) == 0 || to_wrap <= to_compare) {
      earliest(result, prefix + to_wrap);
    } else if (compare < *timer.load) {
      // A live load can put the following reload period across zero even
      // when the current count reaches compare before its first overflow.
      earliest(result, prefix + to_compare + 1 + counter_range - *timer.load);
    }
  }
  return result;
}

std::optional<std::uint64_t> DspTimers::next_clk2_boundary() const noexcept {
  if (!clk2_boundary_valid_) {
    clk2_boundary_ = compute_clk2_boundary();
    clk2_boundary_valid_ = true;
  }
  return clk2_boundary_;
}

std::optional<std::uint64_t> DspTimers::compute_clk2_boundary() const noexcept {
  std::optional<std::uint64_t> result;
  for (const auto &timer : timers_) {
    auto candidate = timer_boundary(timer);
    if (candidate && (timer.control & prescale) != 0) {
      // The current countdown survives live TPLR writes. Only subsequent
      // pulses use the new reload; 24-bit count * 21-bit divisor fits uint64.
      *candidate = static_cast<std::uint64_t>(prescaler_count_) + 1 +
                   ((*candidate - 1) * (static_cast<std::uint64_t>(prescaler_load_) + 1));
    }
    earliest(result, candidate);
  }
  return result;
}
}
