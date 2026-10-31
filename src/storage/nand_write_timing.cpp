// SPDX-License-Identifier: GPL-3.0-only
#include "storage/nand_write_timing.hpp"

#include "common/bus.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_time_window.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

namespace blofeld {
namespace {
constexpr std::chrono::nanoseconds pulse_ns{21};
constexpr std::chrono::nanoseconds high_ns{15};
constexpr std::chrono::nanoseconds cycle_ns{42};
constexpr std::chrono::nanoseconds hold_ns{5};
constexpr unsigned byte_mask = 0xff;
// CE, CLE, ALE and data: setup is measured to rising WE, not falling WE.
constexpr std::array<std::chrono::nanoseconds, 4> setup_ns{std::chrono::nanoseconds{31},
                                                           std::chrono::nanoseconds{21},
                                                           std::chrono::nanoseconds{21},
                                                           std::chrono::nanoseconds{20}};
constexpr std::array<const char *, 4> setup_names{"tCS", "tCLS", "tALS", "tDS"};
constexpr std::array<const char *, 4> hold_names{"tCH", "tCLH", "tALH", "tDH"};
}

NandWriteTiming::NandWriteTiming(const std::uint32_t timebase_hz) : timebase_hz_(timebase_hz) {
  if (timebase_hz_ == 0) { fail("zero reference timebase"); }
}

void NandWriteTiming::fail(const std::string &reason) const {
  throw Fault("NAND write timing ticks=" + std::to_string(ticks_) +
              " timebase-hz=" + std::to_string(timebase_hz_) + ": " + reason);
}

std::uint64_t NandWriteTiming::deadline(const std::uint64_t ticks,
                                        const std::chrono::nanoseconds interval) const {
  constexpr unsigned billion = 1000000000;
  const auto scaled = static_cast<std::uint64_t>(interval.count()) * timebase_hz_;
  const auto duration = (scaled / billion) + (scaled % billion != 0 ? 1U : 0U);
  if (duration > std::numeric_limits<std::uint64_t>::max() - ticks) {
    fail("reference deadline overflow");
  }
  return ticks + duration;
}

void NandWriteTiming::changed(const std::uint64_t ticks, const KnownPins &pins) {
  const auto &old = pins_;
  const std::array changes{
      old.ce_n != pins.ce_n, old.cle != pins.cle, old.ale != pins.ale, old.data != pins.data};
  for (std::size_t index = 0; index < changes.size(); ++index) {
    if (!changes[index]) { continue; }
    if (rose_at_ && ticks < deadline(*rose_at_, hold_ns)) { fail(hold_names[index]); }
    if (!old.ce_n && !old.we_n && index < 3) {
      const unsigned bit = 1U << index;
      if ((low_changes_ & bit) != 0) { fail("control toggled twice while WE low"); }
      low_changes_ |= bit;
    }
    stable_since_[index] = NandTimeWindow{.earliest = ticks, .latest = ticks};
  }
}

NandWriteLatch NandWriteTiming::latch(const std::uint64_t ticks, const KnownPins &pins) const {
  for (std::size_t index = 0; index < setup_ns.size(); ++index) {
    const auto &anchor = stable_since_[index];
    if (anchor && ticks < deadline(anchor->latest, setup_ns[index])) { fail(setup_names[index]); }
  }
  if (pins.ce_n || !pins.re_n) { fail("write latch requires selected CE and inactive RE"); }
  if (pins.data.known != byte_mask) { fail("write latch requires eight known data bits"); }
  if (pins.cle && pins.ale) { fail("unqualified simultaneous CLE/ALE"); }
  auto kind = NandWriteKind::data;
  if (pins.cle) { kind = NandWriteKind::command; }
  if (pins.ale) { kind = NandWriteKind::address; }
  return {.kind = kind, .value = pins.data.value};
}

std::optional<NandWriteLatch> NandWriteTiming::edge(const std::uint64_t ticks,
                                                    const KnownPins &pins) {
  if (pins_.we_n == pins.we_n) { return std::nullopt; }
  if (!pins.we_n) {
    if (pins.ce_n || !pins.re_n) { fail("write pulse requires selected CE and inactive RE"); }
    if (high_since_ && ticks < deadline(*high_since_, high_ns)) { fail("tWH"); }
    if (fell_at_ && ticks < deadline(*fell_at_, cycle_ns)) { fail("tWC"); }
    fell_at_ = ticks;
    low_changes_ = 0;
    return std::nullopt;
  }
  if (!fell_at_ || ticks < deadline(*fell_at_, pulse_ns)) { fail("tWP"); }
  const auto value = latch(ticks, pins);
  rose_at_ = ticks;
  high_since_ = ticks;
  return value;
}

std::optional<NandWriteLatch> NandWriteTiming::observe_changes(const std::uint64_t ticks,
                                                               const KnownPins &pins) {
  const bool standby = pins_.ce_n && pins.ce_n;
  if (!standby && pins_.we_n != pins.we_n && pins_.re_n != pins.re_n) {
    fail("unordered simultaneous RE/WE edges");
  }
  if (pins_.ce_n && !pins.ce_n && !pins_.we_n) {
    fail("unqualified CE assertion during an existing WE-low pulse");
  }
  changed(ticks, pins);
  if (standby) {
    if (pins_.we_n != pins.we_n && pins.we_n) {
      const bool selected_low = fell_at_ && (!high_since_ || *fell_at_ >= *high_since_);
      if (selected_low && ticks < deadline(*fell_at_, pulse_ns)) {
        fail("selected WE low pulse shorter than tWP after deselection");
      }
      high_since_ = ticks;
    }
    return std::nullopt;
  }
  if (!pins_.we_n && !pins.re_n) { fail("RE asserted during write pulse"); }
  return edge(ticks, pins);
}

std::optional<NandWriteLatch> NandWriteTiming::observe(const std::uint64_t ticks,
                                                       const NandWritePins pins) {
  if (ticks < ticks_) { fail("reference time moved backwards"); }
  if (!pins.ce_n || !pins.re_n || !pins.we_n || !pins.cle || !pins.ale) {
    fail("unknown control pin observation");
  }
  if ((pins.data.value & pins.data.known) != pins.data.value) {
    fail("noncanonical data knowledge");
  }
  const KnownPins known{.ce_n = pins.ce_n.value(),
                        .re_n = pins.re_n.value(),
                        .we_n = pins.we_n.value(),
                        .cle = pins.cle.value(),
                        .ale = pins.ale.value(),
                        .data = pins.data};
  auto next = *this;
  next.ticks_ = ticks;
  std::optional<NandWriteLatch> result;
  if (!initialized_) {
    if (!known.we_n && !known.ce_n) { fail("initial WE-high observation is missing"); }
    next.stable_since_.fill(NandTimeWindow{.earliest = ticks, .latest = ticks});
    next.high_since_ = known.we_n ? std::optional{ticks} : std::nullopt;
  } else {
    result = next.observe_changes(ticks, known);
  }
  next.pins_ = known;
  next.initialized_ = true;
  *this = next;
  return result;
}
}
