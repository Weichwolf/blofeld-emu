// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/nand_read_setup.hpp"

#include "blofeld/bus.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace blofeld {
namespace {
// Required stable WE high, ALE low, CLE low and Ready high before falling RE.
constexpr std::array<unsigned, 4> minimum_ns{60, 10, 10, 20};
constexpr std::array<const char *, 4> names{"tWHR", "tAR", "tCLR", "tRR"};
constexpr unsigned nanoseconds_per_second = 1000000000;
}

NandReadSetup::NandReadSetup(const std::uint32_t timebase_hz) : timebase_hz_(timebase_hz) {
  if (timebase_hz_ == 0) { fail("zero reference timebase"); }
}

void NandReadSetup::fail(const std::string &reason) const {
  throw Fault("NAND read setup ticks=" + std::to_string(ticks_) +
              " timebase-hz=" + std::to_string(timebase_hz_) + ": " + reason);
}

void NandReadSetup::check_read(const std::uint64_t ticks, const KnownPins &pins) const {
  if (!pins.we_n || pins.ale || pins.cle) { fail("array read requires WE high and ALE/CLE low"); }
  if (pins.ready != true) { fail("array read requires observed Ready high"); }
  for (std::size_t index = 0; index < minimum_ns.size(); ++index) {
    const auto scaled = static_cast<std::uint64_t>(minimum_ns[index]) * timebase_hz_;
    const auto duration =
        (scaled / nanoseconds_per_second) + (scaled % nanoseconds_per_second != 0 ? 1U : 0U);
    // Subtraction avoids forming future deadlines beyond the reference counter.
    if (ticks - stable_since_[index] < duration) { fail(names[index]); }
  }
}

bool NandReadSetup::observe(const std::uint64_t ticks, const NandReadSetupPins pins) {
  if (ticks < ticks_) { fail("reference time moved backwards"); }
  if (!pins.ce_n || !pins.re_n || !pins.we_n || !pins.ale || !pins.cle) {
    fail("unknown control pin observation");
  }
  const KnownPins known{.ce_n = pins.ce_n.value(),
                        .re_n = pins.re_n.value(),
                        .we_n = pins.we_n.value(),
                        .ale = pins.ale.value(),
                        .cle = pins.cle.value(),
                        .ready = pins.ready};
  auto next = *this;
  next.ticks_ = ticks;
  bool admitted = false;
  if (!initialized_) {
    if (!known.re_n) { next.fail("initial RE-high observation is missing"); }
    next.stable_since_.fill(ticks);
  } else {
    const bool re_changed = pins_.re_n != known.re_n;
    if (re_changed && (pins_.ce_n != known.ce_n || pins_.we_n != known.we_n)) {
      next.fail("unordered simultaneous RE/CE or RE/WE edges");
    }
    const std::array changes{pins_.we_n != known.we_n,
                             pins_.ale != known.ale,
                             pins_.cle != known.cle,
                             pins_.ready != known.ready};
    for (std::size_t index = 0; index < changes.size(); ++index) {
      if (changes[index]) { next.stable_since_[index] = ticks; }
    }
    if (re_changed && !known.re_n && !known.ce_n) {
      next.check_read(ticks, known);
      admitted = true;
    }
  }
  next.pins_ = known;
  next.initialized_ = true;
  *this = next;
  return admitted;
}
}
