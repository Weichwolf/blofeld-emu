// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_timers.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace blofeld {
namespace {
constexpr unsigned te = 1;
constexpr unsigned trm = 0x200;
constexpr unsigned pce = 0x8000;
constexpr unsigned tof = 0x100000;
constexpr unsigned tcf = 0x200000;
constexpr unsigned mode_mask = 0xf0;
constexpr unsigned capture_mode = 0x60;
constexpr std::uint64_t counter_range = 0x1000000;
constexpr unsigned first_load = 0xffff8e;
constexpr unsigned first_compare = 0xffff8d;
}

bool DspTimers::prescaler_enabled() const {
  return std::ranges::any_of(
      timers_, [](const Timer &timer) { return (timer.control & (te | pce)) == (te | pce); });
}

std::uint64_t DspTimers::advance_prescaler(const std::uint64_t edges) {
  if (!prescaler_enabled()) { return 0; }
  const std::uint64_t first_pulse = static_cast<std::uint64_t>(prescaler_count_) + 1;
  if (edges < first_pulse) {
    prescaler_count_ -= static_cast<std::uint32_t>(edges);
    return 0;
  }
  const auto remaining = edges - first_pulse;
  const auto period = static_cast<std::uint64_t>(prescaler_load_) + 1;
  prescaler_count_ = prescaler_load_ - static_cast<std::uint32_t>(remaining % period);
  return 1 + (remaining / period);
}

void DspTimers::advance_free(Timer &timer, const CounterAdvance advance) {
  const auto edges = advance.edges;
  const auto compare = advance.compare;
  if (edges == 0) { return; }
  const auto to_wrap = counter_range - timer.count;
  const auto distance = (compare + counter_range - timer.count) % counter_range;
  const auto to_compare = distance == 0 ? counter_range : distance;
  if (edges >= to_wrap) { timer.flags |= tof; }
  if (edges >= to_compare) { timer.flags |= tcf; }
  timer.count = static_cast<std::uint32_t>((timer.count + (edges % counter_range)) % counter_range);
}

void DspTimers::advance_timer(Timer &timer, const std::size_t index, std::uint64_t edges) {
  if ((timer.control & te) == 0 || edges == 0) { return; }
  if ((timer.control & mode_mask) == capture_mode) {
    advance_capture(timer, index, edges);
    return;
  }
  if (!timer.load.has_value()) {
    throw Fault("Unknown DSP timer load during clock at X:" + hex(first_load - (index * 4), 6));
  }
  if (!timer.compare.has_value()) {
    throw Fault("Unknown DSP timer compare during clock at X:" +
                hex(first_compare - (index * 4), 6));
  }
  const auto load = timer.load.value();
  const auto compare = timer.compare.value();
  const bool reload = (timer.control & trm) != 0;
  if (timer.first_clock || timer.reload_pending) {
    timer.count = load;
    timer.first_clock = false;
    timer.reload_pending = reload && load == compare;
    if (load == compare) { timer.flags |= tcf; }
    --edges;
  }
  if (!reload || edges == 0) {
    advance_free(timer, {.compare = compare, .edges = edges});
    return;
  }
  if (!timer.reload_pending) {
    const auto distance = (compare + counter_range - timer.count) % counter_range;
    const auto until_compare = distance == 0 ? counter_range : distance;
    const auto advance = edges < until_compare ? edges : until_compare;
    advance_free(timer, {.compare = compare, .edges = advance});
    edges -= advance;
    timer.reload_pending = advance == until_compare;
    if (edges == 0) { return; }
  }
  const auto period = ((compare + counter_range - load) % counter_range) + 1;
  if (edges >= period) {
    timer.flags |= tcf;
    if (compare < load) { timer.flags |= tof; }
    timer.count = compare;
    timer.reload_pending = true;
    edges %= period;
  }
  if (edges != 0) {
    timer.count = load;
    if (load == compare) { timer.flags |= tcf; }
    advance_free(timer, {.compare = compare, .edges = edges - 1});
    timer.reload_pending = timer.count == compare;
  }
}

void DspTimers::advance_clk2(const std::uint64_t edges) {
  auto next = *this;
  const auto prescaled = next.advance_prescaler(edges);
  for (std::size_t index = 0; index < next.timers_.size(); ++index) {
    auto &timer = next.timers_.at(index);
    advance_timer(timer, index, (timer.control & pce) != 0 ? prescaled : edges);
  }
  *this = next;
}

}
