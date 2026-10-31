// SPDX-License-Identifier: GPL-3.0-only
#include "storage/nand_read_setup.hpp"
#include "storage/nand_time_window.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace blofeld {
namespace {
constexpr unsigned billion = 1000000000;
constexpr std::array<unsigned, 4> read_setup{60, 10, 10, 20};
}

void NandReadSetup::qualify_ce_window(const std::uint64_t start) const {
  if (start < ticks_ || !initialized_ || !pins_.we_n || !pins_.re_n || pins_.ready != true) {
    fail("CE window requires monotonic initialized WE/RE and Ready high");
  }
  check_control_hold(start);
  for (std::size_t index = 0; index < read_setup.size(); ++index) {
    const auto scaled = static_cast<std::uint64_t>(read_setup[index]) * timebase_hz_;
    const auto duration = (scaled / billion) + (scaled % billion != 0 ? 1U : 0U);
    const auto &anchor = stable_since_[index];
    if (anchor && start - *anchor < duration) {
      fail("CE window precedes expired read setup constraints");
    }
  }
}

void NandReadSetup::resume_ce_window(const std::uint64_t start,
                                     const std::uint64_t end,
                                     const bool ce_n) {
  qualify_ce_window(start);
  if (end < start) { fail("CE window end precedes start"); }
  ticks_ = end;
  if (selected_read_) {
    // RE/WE stay high throughout the opaque window, so no new read occurs.
    // A known preceding selected read may exit as late as end; this is a
    // conservative history bound, not an observed CE edge.
    read_ce_release_ = NandTimeWindow{.earliest = start, .latest = end};
    selected_read_ = !ce_n;
  }
  pins_.ce_n = ce_n;
  stable_since_.fill(std::nullopt);
}

}
