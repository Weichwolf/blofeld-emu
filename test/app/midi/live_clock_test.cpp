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
constexpr std::uint64_t interval = 1000000, native_epoch = 2;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void rejects(auto &&action) {
  bool rejected = false;
  try {
    action();
  } catch (const std::exception &) { rejected = true; }
  require(rejected, "Contradictory clocks and unrepresentable times must fail");
}

PcmClock::Point point(const std::uint64_t ns, const std::uint64_t ticks) {
  return {.epoch = 7, .audio_epoch = native_epoch, .ticks = ticks, .host_ns = ns};
}

void actual_intervals() {
  LiveClock clock({.rate = rate, .queue_frames = queue, .uncertainty_ns = 0});
  constexpr std::uint64_t expected_prime = 348300;
  require(clock.prime_ticks() == expected_prime,
          "Priming uses the actual 120-frame queue plus an interval and block");
  require(!clock.correlate(host, native_epoch), "An absent audio anchor cannot supply guest time");
  clock.observe(point(host, tick));
  require(clock.correlate(host, native_epoch) == tick && !clock.correlate(host + 1, native_epoch),
          "The first actual observation maps only its own timestamp");
  constexpr auto first_point = tick + 11;
  constexpr auto second_point = tick + 31;
  constexpr auto second_midpoint = tick + 21;
  clock.observe(point(host + interval, first_point));
  clock.observe(point(host + (2 * interval), second_point));
  constexpr auto offset = interval / 2;
  require(clock.correlate(host + offset, native_epoch) == tick + 6 &&
              clock.correlate(host + interval, native_epoch) == first_point &&
              clock.correlate(host + interval + offset, native_epoch) == second_midpoint,
          "Interpolation uses adjacent original points and their changed slope");
  require(!clock.correlate(host - 1, native_epoch) &&
              !clock.correlate(host + (3 * interval), native_epoch),
          "Absent observations on either side do not extrapolate");
  clock.observe(point(host + (2 * interval), second_point));
  require(clock.correlate(host + (2 * interval), native_epoch) == second_point,
          "An unchanged native observation is stable");
}

void observation_gap() {
  LiveClock clock({.rate = rate, .queue_frames = queue, .uncertainty_ns = 0});
  clock.observe(point(host, tick));
  constexpr auto after_gap = host + LiveClock::maximum_gap_ns + 1;
  constexpr auto first_point = tick + 40;
  constexpr auto second_point = tick + 48;
  constexpr auto midpoint = tick + 44;
  clock.observe(point(after_gap, first_point));
  require(!clock.correlate(host, native_epoch) && !clock.correlate(after_gap - 1, native_epoch) &&
              clock.correlate(after_gap, native_epoch) == first_point,
          "A new observation after a gap starts an independent correlation interval");
  clock.observe(point(after_gap + interval, second_point));
  require(clock.correlate(after_gap + (interval / 2), native_epoch) == midpoint,
          "Correlation resumes from the new observations");
}

void reset_observations() {
  LiveClock clock({.rate = rate, .queue_frames = queue, .uncertainty_ns = 0});
  constexpr std::uint64_t step = 8;
  const auto old_point = point(host + interval, tick + (step / 2));
  clock.observe(point(host, tick));
  clock.observe(old_point);
  auto reset = point(host + (2 * interval), tick + step);
  ++reset.audio_epoch;
  ++reset.epoch;
  require(clock.correlate(old_point.host_ns, native_epoch) == old_point.ticks &&
              !clock.correlate(old_point.host_ns, reset.audio_epoch),
          "The grant's new audio epoch revokes old mapping before a new playhead is available");
  clock.observe(reset);
  require(!clock.correlate(old_point.host_ns, reset.audio_epoch) &&
              clock.correlate(reset.host_ns, reset.audio_epoch) == reset.ticks,
          "The first real position after a DSP reset replaces old correlation");
  reset.ticks += step;
  reset.host_ns += interval;
  clock.observe(reset);
  require(clock.correlate(reset.host_ns - (interval / 2), reset.audio_epoch) ==
              reset.ticks - (step / 2),
          "Interpolation resumes entirely within the new PCM epoch");
  auto stale = old_point;
  stale.host_ns = reset.host_ns + interval;
  stale.ticks = reset.ticks + step;
  rejects([&] { clock.observe(stale); });
}

void retention_and_limits() {
  LiveClock clock({.rate = rate, .queue_frames = queue, .uncertainty_ns = 0});
  constexpr std::uint64_t points = 100;
  for (std::uint64_t index = 0; index < points; ++index) {
    clock.observe(point(host + (index * interval), tick + (index * 3)));
  }
  require(!clock.correlate(host, native_epoch) &&
              clock.correlate(host + ((points - 2) * interval), native_epoch) ==
                  tick + ((points - 2) * 3),
          "Bounded retention wraps without aliasing an expired interval");
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  constexpr auto large_host = maximum - (4 * interval);
  constexpr auto large_tick = maximum - interval;
  LiveClock wide({.rate = rate, .queue_frames = queue, .uncertainty_ns = 0});
  wide.observe(point(large_host, large_tick));
  wide.observe(point(large_host + interval, large_tick + 3));
  require(wide.correlate(large_host + (interval / 2), native_epoch) == large_tick + 2,
          "Large absolute guest and host epochs keep every integer bit");
  LiveClock product({.rate = rate, .queue_frames = queue, .uncertainty_ns = 0});
  product.observe(point(host, 0));
  product.observe(point(host + interval, maximum));
  rejects([&] { static_cast<void>(product.correlate(host + (interval / 2), native_epoch)); });
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
                             point(host, tick + 1)}) {
    rejects([&] { clock.observe(invalid); });
  }
  auto changed = point(host + 1, tick + 1);
  ++changed.epoch;
  rejects([&] { clock.observe(changed); });
  --changed.epoch;
  ++changed.audio_epoch;
  rejects([&] { clock.observe(changed); });
}

void bounded_uncertainty() {
  constexpr std::uint64_t granted = 362812;
  constexpr std::size_t native_queue = 160;
  constexpr std::uint64_t expected_prime = 457143;
  LiveClock clock({.rate = rate, .queue_frames = native_queue, .uncertainty_ns = granted});
  require(clock.uncertainty_ns() == granted && clock.prime_ticks() == expected_prime,
          "Actual queue grants determine bounded priming");
  clock.observe(point(host, tick));
  require(!clock.correlate(host + 1, native_epoch), "Projection needs an observed interval");
  auto next = point(host + interval, tick + 11);
  next.uncertainty_ns = granted;
  clock.observe(next);
  require(clock.correlate(host, native_epoch) == tick &&
              clock.correlate(host + interval, native_epoch) == tick + 11,
          "Timestamp uncertainty does not retime the mapped original samples");
  auto later = point(host + (2 * interval), tick + 16);
  clock.observe(later);
  require(clock.correlate(later.host_ns + granted, native_epoch) == later.ticks + 2 &&
              !clock.correlate(later.host_ns + granted + 1, native_epoch) &&
              !clock.correlate(later.host_ns + 1, native_epoch + 1),
          "Projection uses the latest slope within the current device uncertainty and epoch");
  constexpr std::uint64_t short_interval = 100;
  constexpr std::uint64_t short_tick_delta = 6;
  const auto nearby = point(later.host_ns + short_interval, later.ticks + short_tick_delta);
  clock.observe(nearby);
  require(clock.correlate(nearby.host_ns + short_interval, native_epoch) ==
                  nearby.ticks + short_tick_delta &&
              !clock.correlate(nearby.host_ns + short_interval + 1, native_epoch),
          "An uncertain timestamp may project at most one observed interval");
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
  observation_gap();
  reset_observations();
  retention_and_limits();
  faults();
  bounded_uncertainty();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
