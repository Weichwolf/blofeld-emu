// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_timers.hpp"

#include <cstddef>
#include <cstdint>

namespace blofeld {
namespace {
constexpr unsigned te = 1;
constexpr unsigned inv = 0x100;
constexpr unsigned mode_mask = 0xf0;
constexpr unsigned capture_mode = 0x60;
constexpr unsigned tof = 0x100000;
constexpr unsigned tcf = 0x200000;
constexpr unsigned first_load = 0xffff8e;
constexpr unsigned first_control = 0xffff8f;
constexpr std::uint64_t counter_range = 0x1000000;
}

void DspTimers::observe_synchronized_tio(const unsigned index, const bool level) {
  if (index >= external_.size()) {
    throw Fault("DSP capture has no synchronized TIO at index " + hex(index, 1));
  }
  auto &timer = timers_.at(index);
  if ((timer.control & mode_mask) != capture_mode) {
    timer.synchronized = level;
    timer.synchronized_qualified = true;
    return;
  }
  const bool enabled = (timer.control & te) != 0 && !timer.halted;
  if (enabled && !timer.synchronized) {
    throw Fault("Unknown previous DSP synchronized TIO level at X:" +
                hex(first_control - (index * 4), 6));
  }
  const bool edge = timer.synchronized && *timer.synchronized != level;
  if (enabled && edge && level == ((timer.control & inv) == 0)) {
    timer.captured = timer.count;
    timer.halted = true;
    timer.flags |= tcf;
  }
  timer.synchronized = level;
  timer.synchronized_qualified = true;
}

void DspTimers::advance_capture(Timer &timer, const std::size_t index, std::uint64_t edges) {
  if (timer.halted || edges == 0) { return; }
  if (timer.first_clock) {
    if (!timer.load) {
      throw Fault("Unknown DSP timer capture load during clock at X:" +
                  hex(first_load - (index * 4), 6));
    }
    timer.count = *timer.load;
    timer.first_clock = false;
    --edges;
  }
  if (!timer.synchronized || !timer.synchronized_qualified) {
    throw Fault("Unknown DSP timer capture input during clock at X:" +
                hex(first_control - (index * 4), 6));
  }
  if (edges >= counter_range - timer.count) { timer.flags |= tof; }
  timer.count = static_cast<std::uint32_t>((timer.count + (edges % counter_range)) % counter_range);
}
}
