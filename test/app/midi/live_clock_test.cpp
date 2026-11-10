// SPDX-License-Identifier: GPL-3.0-only
#include "audio/pcm_clock.hpp"
#include "midi/live_clock.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
using blofeld::app::LiveClock;
using blofeld::app::PcmClock;
constexpr unsigned rate = 44100;
constexpr std::size_t queue = 120;
constexpr std::uint64_t host = 1000000000, tick = 462000000;
constexpr std::uint64_t interval = 1000000;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void rejects(auto &&action) {
  bool rejected = false;
  try {
    action();
  } catch (const std::exception &) { rejected = true; }
  require(rejected, "Contradictory clocks, late MIDI and unrepresentable times must fail");
}

PcmClock::Point point(const std::uint64_t ns, const std::uint64_t ticks) {
  return {.epoch = 7, .audio_epoch = 2, .ticks = ticks, .host_ns = ns};
}

LiveClock::Window required_window(LiveClock &clock, const std::uint64_t now) {
  const auto value = clock.window(now);
  if (!value) { throw std::runtime_error("Expected an interval backed by actual audio"); }
  return *value;
}

void actual_intervals() {
  LiveClock clock({.rate = rate, .queue_frames = queue, .uncertainty_ns = 0});
  constexpr std::uint64_t expected_prime = 348300;
  constexpr std::uint64_t expected_lead = 408300;
  constexpr std::uint64_t last_delta = 31;
  constexpr std::uint64_t middle_delta = 21;
  require(clock.prime_ticks() == expected_prime && clock.lead_ticks() == expected_lead,
          "Actual 120-frame queue plus an interval/block and finite delivery allowance");
  require(!clock.window(host), "Neither the guest MCU nor host wall time invents an audio anchor");
  rejects([&] { static_cast<void>(clock.event(host)); });
  clock.observe(point(host, tick));
  require(!clock.window(host + LiveClock::delivery_ns - 1),
          "Wait until a real audio interval covers the delivery cutoff");
  clock.observe(point(host + interval, tick + 11));
  clock.observe(point(host + (2 * interval), tick + last_delta));
  constexpr auto offset = interval / 2;
  require(clock.event(host) == tick + expected_lead &&
              clock.event(host + offset) == tick + 6 + expected_lead &&
              clock.event(host + interval) == tick + 11 + expected_lead &&
              clock.event(host + interval + offset) == tick + middle_delta + expected_lead,
          "Ceiling interpolation uses exact adjacent original points, including a changed slope");
  const auto window = required_window(clock, host + interval + LiveClock::delivery_ns);
  require(window.host_ns == host + interval && window.ticks == tick + 11 + expected_lead,
          "A commitment has a real retained timestamp and explicit bounded lead");
  clock.commit(window);
  require(clock.event(host + interval + offset) == tick + middle_delta + expected_lead,
          "Equal-timestamp MIDI packets retain DIN serialization at the transport boundary");
  rejects([&] { static_cast<void>(clock.event(host + interval - 1)); });
  rejects([&] { static_cast<void>(clock.event(host + (3 * interval))); });
  const auto latest = required_window(clock, host + (4 * interval));
  require(latest.host_ns == host + (2 * interval) &&
              latest.ticks == tick + last_delta + expected_lead,
          "A newer host poll never extrapolates beyond the latest actual audio point");
  clock.commit(latest);
  rejects([&] { clock.commit(window); });
  auto invented = latest;
  ++invented.ticks;
  rejects([&] { clock.commit(invented); });
  clock.observe(point(host + (2 * interval), tick + last_delta));
  require(clock.window(host + (4 * interval)) == latest,
          "An unchanged native observation is stable");
}

void retention_and_limits() {
  LiveClock clock({.rate = rate, .queue_frames = queue, .uncertainty_ns = 0});
  constexpr std::uint64_t points = 100;
  for (std::uint64_t index = 0; index < points; ++index) {
    clock.observe(point(host + (index * interval), tick + (index * 3)));
  }
  rejects([&] { static_cast<void>(clock.event(host)); });
  require(clock.event(host + ((points - 2) * interval)) ==
              tick + ((points - 2) * 3) + clock.lead_ticks(),
          "Bounded retention wraps without aliasing an expired interval");
  rejects([&] {
    static_cast<void>(
        clock.window(host + ((points - 1) * interval) + LiveClock::maximum_gap_ns + 1));
  });
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  constexpr auto large_host = maximum - (4 * interval);
  const auto large_tick = maximum - clock.lead_ticks() - interval;
  LiveClock wide({.rate = rate, .queue_frames = queue, .uncertainty_ns = 0});
  wide.observe(point(large_host, large_tick));
  wide.observe(point(large_host + interval, large_tick + 3));
  require(wide.event(large_host + (interval / 2)) == large_tick + wide.lead_ticks() + 2,
          "Large absolute guest and host epochs keep every integer bit");
  LiveClock exhausted({.rate = rate, .queue_frames = queue, .uncertainty_ns = 0});
  exhausted.observe(point(host, maximum));
  rejects([&] { static_cast<void>(exhausted.event(host)); });
  LiveClock product({.rate = rate, .queue_frames = queue, .uncertainty_ns = 0});
  product.observe(point(host, 0));
  product.observe(point(host + interval, maximum));
  rejects([&] { static_cast<void>(product.event(host + (interval / 2))); });
}

void faults() {
  rejects([] { const LiveClock clock({.rate = 0, .queue_frames = queue, .uncertainty_ns = 0}); });
  rejects([] { const LiveClock clock({.rate = rate, .queue_frames = 0, .uncertainty_ns = 0}); });
  rejects([] { const LiveClock clock({.rate = rate, .queue_frames = rate, .uncertainty_ns = 0}); });
  LiveClock clock({.rate = rate, .queue_frames = queue, .uncertainty_ns = 0});
  clock.observe(point(host, tick));
  for (const auto invalid : {point(0, tick + 1),
                             point(host - 1, tick + 1),
                             point(host + 1, tick),
                             point(host, tick + 1),
                             point(host + LiveClock::maximum_gap_ns + 1, tick + 1)}) {
    rejects([&] { clock.observe(invalid); });
  }
  auto changed = point(host + 1, tick + 1);
  ++changed.epoch;
  rejects([&] { clock.observe(changed); });
  --changed.epoch;
  ++changed.audio_epoch;
  rejects([&] { clock.observe(changed); });
  static_cast<void>(clock.window(host));
  rejects([&] { static_cast<void>(clock.window(host - 1)); });
}

void bounded_uncertainty() {
  constexpr std::uint64_t granted = 362812;
  constexpr std::uint64_t reserve_ticks = 21769;
  constexpr std::size_t native_queue = 160;
  LiveClock clock({.rate = rate, .queue_frames = native_queue, .uncertainty_ns = granted});
  const auto base = LiveClock({.rate = rate, .queue_frames = native_queue}).lead_ticks();
  require(clock.uncertainty_ns() == granted && clock.lead_ticks() == base + reserve_ticks,
          "Immutable native grants reserve a ceiling-rounded lead before any observation");
  clock.observe(point(host, tick));
  auto next = point(host + interval, tick + 11);
  next.uncertainty_ns = granted;
  clock.observe(next);
  const auto observed = required_window(clock, host + LiveClock::delivery_ns + granted);
  require(observed.host_ns == host && observed.ticks == tick + base + reserve_ticks,
          "Silence includes the full granted uncertainty even after an exact first point");
  clock.commit(observed);
  require(clock.event(host) == tick + base + reserve_ticks,
          "Exact boundary MIDI retains the same conservative correlation lead");
  auto later = point(host + (2 * interval), tick + 16);
  clock.observe(later);
  require(clock.uncertainty_ns() == granted && clock.lead_ticks() == base + reserve_ticks,
          "Later exact observations never retime earlier commitments or queued notes");
  constexpr std::uint64_t terminal_delta = 17;
  later = point(host + (3 * interval), tick + terminal_delta);
  later.uncertainty_ns = granted + 1;
  rejects([&] { clock.observe(later); });
  LiveClock exact({.rate = rate, .queue_frames = queue, .uncertainty_ns = 0});
  exact.observe(point(host, tick));
  rejects([&] { exact.observe(next); });
  constexpr std::size_t large_queue = 188;
  rejects([] {
    const LiveClock narrow({.rate = rate, .queue_frames = large_queue, .uncertainty_ns = granted});
  });
  rejects([] {
    const LiveClock excessive(
        {.rate = rate, .queue_frames = queue, .uncertainty_ns = LiveClock::maximum_gap_ns});
  });
  rejects([] {
    const LiveClock exhausted({.rate = rate,
                               .queue_frames = queue,
                               .uncertainty_ns = std::numeric_limits<std::uint64_t>::max()});
  });
}

}

int main() try {
  actual_intervals();
  retention_and_limits();
  faults();
  bounded_uncertainty();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
