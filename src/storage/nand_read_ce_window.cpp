// SPDX-License-Identifier: GPL-3.0-only
#include "storage/nand_read_timing.hpp"

#include <chrono>
#include <cstdint>

namespace blofeld {
namespace {
constexpr std::chrono::nanoseconds read_high{15};
constexpr std::chrono::nanoseconds read_cycle{42};
}

void NandReadTiming::qualify_ce_window(const std::uint64_t start) const {
  check_time(start);
  if (!initialized_ || !re_n_ || at(start).driven != false) {
    fail("CE window requires initialized RE high and expired output release");
  }
  if ((fell_at_ && start < deadline(*fell_at_, read_cycle)) ||
      (rose_at_ && start < deadline(*rose_at_, read_high))) {
    fail("CE window precedes expired read pulse constraints");
  }
}

void NandReadTiming::resume_ce_window(const std::uint64_t start,
                                      const std::uint64_t end,
                                      const bool ce_n) {
  qualify_ce_window(start);
  if (end < start) { fail("CE window end precedes start"); }
  ticks_ = end;
  ce_n_ = ce_n;
  selected_at_ = {.earliest = 0, .latest = end};
  valid_at_ = {};
  hold_until_ = 0;
  floating_at_ = 0;
  fell_at_.reset();
  rose_at_.reset();
  selected_re_low_ = false;
  data_ = {};
  hold_driven_.reset();
  phase_ = Phase::floating;
}
}
