// SPDX-License-Identifier: GPL-3.0-only
#include "storage/nand_read_timing.hpp"

#include "common/bus.hpp"
#include "storage/nand_array.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <string>

namespace blofeld {
namespace {
constexpr unsigned nanoseconds_per_second = 1000000000;
constexpr std::chrono::nanoseconds re_access_ns{30};
constexpr std::chrono::nanoseconds ce_access_ns{35};
constexpr std::chrono::nanoseconds re_release_ns{30};
constexpr std::chrono::nanoseconds ce_release_ns{20};
constexpr std::chrono::nanoseconds output_hold_ns{15};
constexpr std::chrono::nanoseconds re_low_ns{21};
constexpr std::chrono::nanoseconds re_high_ns{15};
constexpr std::chrono::nanoseconds read_cycle_ns{42};
}

NandReadTiming::NandReadTiming(const std::uint32_t timebase_hz) : timebase_hz_(timebase_hz) {
  if (timebase_hz_ == 0) { fail("zero reference timebase"); }
}

void NandReadTiming::fail(const std::string &reason) const {
  throw Fault("NAND read timing ticks=" + std::to_string(ticks_) +
              " timebase-hz=" + std::to_string(timebase_hz_) + ": " + reason);
}

std::uint64_t NandReadTiming::deadline(const std::uint64_t ticks,
                                       const std::chrono::nanoseconds interval) const {
  if (interval.count() < 0 || static_cast<std::uint64_t>(interval.count()) >
                                  std::numeric_limits<std::uint64_t>::max() / timebase_hz_) {
    fail("invalid reference interval");
  }
  const auto scaled = static_cast<std::uint64_t>(interval.count()) * timebase_hz_;
  const auto duration =
      (scaled / nanoseconds_per_second) + (scaled % nanoseconds_per_second != 0 ? 1U : 0U);
  if (duration > std::numeric_limits<std::uint64_t>::max() - ticks) {
    fail("reference deadline overflow");
  }
  return ticks + duration;
}

void NandReadTiming::check_time(const std::uint64_t ticks) const {
  if (ticks < ticks_) { fail("reference time moved backwards to " + std::to_string(ticks)); }
}

NandReadOutput NandReadTiming::at(const std::uint64_t ticks) const {
  if (!initialized_) { return {}; }
  switch (phase_) {
    case Phase::floating: return {.driven = false};
    case Phase::reading:
      return ticks >= valid_at_ ? NandReadOutput{.driven = true, .data = data_} : NandReadOutput{};
    case Phase::releasing:
      if (ticks >= floating_at_) { return {.driven = false}; }
      if (ticks < hold_until_) { return {.driven = hold_driven_, .data = data_}; }
      return {};
    default: fail("invalid output phase");
  }
}

NandReadOutput NandReadTiming::output(const std::uint64_t ticks) {
  check_time(ticks);
  ticks_ = ticks;
  return at(ticks);
}

void NandReadTiming::release(const std::uint64_t ticks, const std::chrono::nanoseconds maximum_ns) {
  const auto hold = deadline(ticks, output_hold_ns);
  const auto floating = deadline(ticks, maximum_ns);
  if (phase_ == Phase::releasing) {
    hold_until_ = std::min(hold_until_, hold);
    floating_at_ = std::min(floating_at_, floating);
  } else if (phase_ == Phase::reading) {
    const auto before = at(ticks);
    hold_driven_ = before.driven;
    data_ = before.data;
    hold_until_ = hold;
    floating_at_ = floating;
    phase_ = Phase::releasing;
  }
}

void NandReadTiming::select_edge(const std::uint64_t ticks, const bool rising) {
  if (rising) {
    release(ticks, ce_release_ns);
  } else {
    if (!re_n_) { fail("unqualified CE assertion while RE is low"); }
    if (phase_ == Phase::releasing && ticks < floating_at_) {
      fail("unqualified reselect during output release");
    }
    selected_at_ = ticks;
    phase_ = Phase::floating;
  }
  ce_n_ = rising;
}

void NandReadTiming::read_edge(const std::uint64_t ticks, const bool rising, const Source &source) {
  if (rising) {
    if (fell_at_ && ticks < deadline(*fell_at_, re_low_ns)) {
      fail("RE low pulse shorter than tRP");
    }
    release(ticks, re_release_ns);
    rose_at_ = ticks;
  } else {
    if (rose_at_ && ticks < deadline(*rose_at_, re_high_ns)) {
      fail("RE high pulse shorter than tREH");
    }
    if (fell_at_ && ticks < deadline(*fell_at_, read_cycle_ns)) {
      fail("read cycle shorter than tRC");
    }
    if (!ce_n_) {
      const auto valid =
          std::max(deadline(ticks, re_access_ns), deadline(selected_at_, ce_access_ns));
      if (!source) { fail("selected RE edge has no protocol data source"); }
      const auto data = source();
      if ((data.value & data.known) != data.value) { fail("noncanonical protocol byte knowledge"); }
      data_ = data;
      valid_at_ = valid;
      phase_ = Phase::reading;
    }
    fell_at_ = ticks;
  }
  re_n_ = rising;
}

void NandReadTiming::observe(const std::uint64_t ticks,
                             const NandReadPins pins,
                             const Source &source) {
  check_time(ticks);
  if (!pins.ce_n || !pins.re_n) { fail("unknown CE/RE pin observation"); }
  auto next = *this;
  next.ticks_ = ticks;
  if (!initialized_) {
    if (!*pins.ce_n || !*pins.re_n) { fail("initial CE/RE idle-high observation is missing"); }
    next.phase_ = Phase::releasing;
    next.hold_until_ = ticks;
    next.floating_at_ = next.deadline(ticks, ce_release_ns);
    next.initialized_ = true;
  } else {
    const bool ce_changed = ce_n_ != *pins.ce_n;
    const bool re_changed = re_n_ != *pins.re_n;
    if (ce_changed && re_changed) { fail("unordered simultaneous CE/RE edges"); }
    if (ce_changed) { next.select_edge(ticks, *pins.ce_n); }
    if (re_changed) { next.read_edge(ticks, *pins.re_n, source); }
  }
  *this = next;
}
}
