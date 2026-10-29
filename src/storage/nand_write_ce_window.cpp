// SPDX-License-Identifier: GPL-3.0-only
#include "storage/nand_time_window.hpp"
#include "storage/nand_write_timing.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace blofeld {
namespace {
constexpr std::chrono::nanoseconds write_hold{5};
constexpr std::chrono::nanoseconds write_high{15};
constexpr std::chrono::nanoseconds write_cycle{42};
constexpr std::array<std::chrono::nanoseconds, 4> write_setup{std::chrono::nanoseconds{31},
                                                              std::chrono::nanoseconds{21},
                                                              std::chrono::nanoseconds{21},
                                                              std::chrono::nanoseconds{20}};
}

void NandWriteTiming::qualify_ce_window(const std::uint64_t start) const {
  if (start < ticks_ || !initialized_ || !pins_.we_n || !pins_.re_n) {
    fail("CE window requires monotonic initialized WE/RE high");
  }
  if ((rose_at_ && start < deadline(*rose_at_, write_hold)) ||
      (high_since_ && start < deadline(*high_since_, write_high)) ||
      (fell_at_ && start < deadline(*fell_at_, write_cycle))) {
    fail("CE window precedes expired write pulse/hold constraints");
  }
  for (std::size_t index = 0; index < write_setup.size(); ++index) {
    const auto &anchor = stable_since_[index];
    if (anchor && start < deadline(anchor->latest, write_setup[index])) {
      fail("CE window precedes expired input setup constraints");
    }
  }
}

void NandWriteTiming::resume_ce_window(const std::uint64_t start,
                                       const std::uint64_t end,
                                       const bool ce_n) {
  qualify_ce_window(start);
  if (end < start) { fail("CE window end precedes start"); }
  auto next = *this;
  next.ticks_ = end;
  next.pins_.ce_n = ce_n;
  next.stable_since_.fill(std::nullopt);
  next.stable_since_[0] = NandTimeWindow{.earliest = 0, .latest = end};
  next.fell_at_.reset();
  next.rose_at_.reset();
  next.high_since_.reset();
  next.low_changes_ = 0;
  *this = next;
}

}
