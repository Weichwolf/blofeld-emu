// SPDX-License-Identifier: GPL-3.0-only
#include "audio/pcm_clock.hpp"
#include "audio/render_position.hpp"
#include "io/pcm_output.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <thread>

namespace {
using blofeld::app::PcmClock;
using blofeld::host::audio::RenderPosition;
using blofeld::vi::PcmOutput;
constexpr std::size_t history_frames = 8;
constexpr std::uint64_t guest_epoch = 7;
constexpr std::uint64_t native_epoch = 3;
constexpr std::array<std::uint64_t, 4> initial_ticks{100, 102, 109, 500};
constexpr std::uint64_t first_host = 10000;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void rejects(auto &&action) {
  bool rejected = false;
  try {
    action();
  } catch (const std::runtime_error &) { rejected = true; }
  require(rejected, "Missing or contradictory PCM/native clock data must fail");
}

PcmOutput::Block block(const std::uint64_t first,
                       const std::span<const std::uint64_t> ticks,
                       const std::uint64_t epoch = guest_epoch) {
  static const std::array<PcmOutput::Frame, 16> frames{};
  return {.frames = std::span(frames).first(ticks.size()),
          .ticks = ticks,
          .epoch = epoch,
          .first_frame = first};
}

RenderPosition position(const std::uint64_t first,
                        const std::uint32_t count = 1,
                        const std::uint64_t host = first_host) {
  return {.epoch = native_epoch, .first_frame = first, .host_ns = host, .frames = count};
}

void exact_points() {
  PcmClock clock(history_frames);
  require(!clock.read(), "No host correlation exists before real PCM and a native interval");
  clock.append(block(0, initial_ticks), native_epoch);
  require(!clock.read(), "Accepted PCM alone does not invent a host clock");
  clock.observe(position(1, 2));
  const PcmClock::Point expected{
      .epoch = guest_epoch, .audio_epoch = native_epoch, .ticks = 102, .host_ns = first_host};
  require(clock.read() == expected,
          "Bind the exact retained completion without nominal interpolation");
  clock.observe(position(1, 2));
  require(clock.read() == expected, "Repeated observations preserve the same clock point");
  constexpr std::array<std::uint64_t, 4> later{501, 502, 1000, 1001};
  clock.append(block(4, later), native_epoch);
  clock.observe(position(4, 4, first_host * 2));
  const auto current = clock.read();
  require(current && current->ticks == later.front() && current->host_ns == first_host * 2,
          "Skipped native observations do not synthesize or skip source frames");
}

void correlation_uncertainty() {
  PcmClock clock(history_frames);
  clock.append(block(0, initial_ticks), native_epoch);
  auto native = position(1, 2);
  constexpr std::uint64_t uncertainty = 127;
  native.uncertainty_ns = uncertainty;
  const auto point = clock.read(native);
  require(point && point->ticks == initial_ticks[1] && point->host_ns == first_host &&
              point->uncertainty_ns == uncertainty,
          "Native timing uncertainty survives direct source-frame correlation unchanged");
  clock.observe(native);
  require(clock.read() == point, "Owner publication retains the same complete bounded point");
  native.uncertainty_ns = first_host;
  require(!clock.read(native), "A native uncertainty interval cannot include time zero");
  native.host_ns = std::numeric_limits<std::uint64_t>::max();
  native.uncertainty_ns = 1;
  require(!clock.read(native), "A native uncertainty interval cannot overflow host time");
}

void selected_source_origin() {
  constexpr std::uint64_t origin = 48003;
  PcmClock clock(history_frames);
  clock.clear(origin);
  clock.append(block(origin, initial_ticks), native_epoch);
  require(!clock.read(), "A source origin alone cannot invent a native render observation");
  const auto fresh = clock.read(position(1, 2));
  require(fresh == PcmClock::Point{.epoch = guest_epoch,
                                   .audio_epoch = native_epoch,
                                   .ticks = initial_ticks[1],
                                   .host_ns = first_host},
          "The first native callback maps original frame numbers before another PCM write");
  require(!clock.read(), "A concurrent lookup does not rewrite the owner's observation stream");
  clock.observe(position(1, 2));
  require(clock.read() == fresh, "Owner publication and direct native lookup agree exactly");
  constexpr std::array<std::uint64_t, 8> later{501, 502, 600, 601, 700, 800, 900, 1000};
  clock.append(block(origin + initial_ticks.size(), later), native_epoch);
  require(!clock.read(position(3)), "An expired interval cannot alias a recycled history slot");
  const auto retained = clock.read(position(4, 4, first_host + 1));
  require(retained && retained->ticks == later.front(),
          "History wrapping uses original source indices and native-relative frame positions");
  clock.clear(origin + (2 * history_frames));
  require(!clock.read() && !clock.read(position(4)), "Reset revokes both correlation interfaces");
  rejects([&] { clock.append(block(origin, initial_ticks), native_epoch); });
}

void source_origin_limits() {
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  constexpr std::array<std::uint64_t, 1> tick{0};
  PcmClock clock(history_frames);
  clock.clear(maximum - 1);
  clock.append(block(maximum - 1, tick), native_epoch);
  const auto point = clock.read(position(0));
  require(point && point->ticks == 0,
          "A high source origin and zero original tick retain their full integer range");
  require(!clock.read(position(maximum)) && !clock.read(position(1)) &&
              !clock.read(position(0, 2)) && !clock.read(position(0, 0)),
          "Native-relative intervals cannot overflow or exceed their actual source suffix");
  rejects([&] { clock.append(block(maximum, tick), native_epoch); });
  require(!clock.read(position(0)), "A source overflow failure revokes fresh native lookup");
  clock.clear();
  clock.append(block(0, initial_ticks), native_epoch);
  require(!clock.read(position(0, 1, 0)), "Host timestamp zero is never a native clock point");
  auto wrong = position(0);
  ++wrong.epoch;
  require(!clock.read(wrong), "A native reset epoch never binds the preceding source history");
}

void retention_and_reset() {
  constexpr std::size_t retained = 4;
  PcmClock clock(retained);
  constexpr std::array<std::uint64_t, 8> ticks{1, 4, 7, 8, 12, 15, 20, 32};
  clock.append(block(0, ticks), native_epoch);
  clock.observe(position(4, 4));
  const auto wrapped = clock.read();
  require(wrapped && wrapped->ticks == ticks[4],
          "Wrapped retention includes its oldest remaining complete interval");
  rejects([&] { clock.observe(position(3)); });
  require(!clock.read(), "An expired source frame revokes the clock");
  rejects([&] { clock.observe(position(4, 4)); });
  rejects([&] { clock.append(block(8, initial_ticks), native_epoch); });
  clock.clear();
  require(!clock.read(), "Reset cannot leak the previous epoch");
  constexpr std::array<std::uint64_t, 1> zero{0};
  clock.append(block(0, zero, 0), 0);
  clock.observe({.epoch = 0, .first_frame = 0, .host_ns = 1, .frames = 1});
  require(clock.read() == PcmClock::Point{.epoch = 0, .audio_epoch = 0, .ticks = 0, .host_ns = 1},
          "Guest/audio epoch zero and original tick zero remain valid after reset");
  clock.clear();
  clock.clear();
  require(!clock.read(), "Repeated owner shutdown or reset remains revoked");
}

void native_failures() {
  for (auto candidate : {position(0, 0), position(0, 1, 0), position(4), position(3, 2)}) {
    PcmClock clock(history_frames);
    clock.append(block(0, initial_ticks), native_epoch);
    rejects([&] { clock.observe(candidate); });
    require(!clock.read(), "Invalid native intervals cannot produce an instrument clock");
  }
  PcmClock unbound(history_frames);
  rejects([&] { unbound.observe(position(0)); });
  for (auto candidate : {position(0, 1, first_host + 1),
                         position(2, 1, first_host + 1),
                         position(3, 1, first_host),
                         position(3, 1, first_host - 1)}) {
    PcmClock clock(history_frames);
    clock.append(block(0, initial_ticks), native_epoch);
    clock.observe(position(1, 2));
    rejects([&] { clock.observe(candidate); });
    require(!clock.read(), "Backwards/overlapping frames and reused host time revoke publication");
  }
  PcmClock clock(history_frames);
  clock.append(block(0, initial_ticks), native_epoch);
  auto wrong_epoch = position(0);
  ++wrong_epoch.epoch;
  rejects([&] { clock.observe(wrong_epoch); });
}

void source_failures() {
  constexpr std::array<std::uint64_t, 2> later{501, 600};
  for (const auto next :
       {block(5, later), block(4, later, guest_epoch + 1), block(4, initial_ticks)}) {
    PcmClock clock(history_frames);
    clock.append(block(0, initial_ticks), native_epoch);
    clock.observe(position(0));
    rejects([&] { clock.append(next, native_epoch); });
    require(!clock.read(), "Source gaps, epoch changes and backwards ticks revoke publication");
  }
  PcmClock clock(history_frames);
  clock.append(block(0, initial_ticks), native_epoch);
  rejects([&] { clock.append(block(4, later), native_epoch + 1); });
  clock.clear();
  constexpr std::array<std::uint64_t, 2> duplicate{1, 1};
  rejects([&] { clock.append(block(0, duplicate), native_epoch); });
  clock.clear();
  rejects([&] { clock.append(block(0, {}), native_epoch); });
}

void full_range_and_disabled() {
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  const std::array<std::uint64_t, 1> tick{maximum};
  PcmClock clock(1);
  clock.append(block(0, tick, maximum), maximum);
  clock.observe({.epoch = maximum, .first_frame = 0, .host_ns = maximum, .frames = 1});
  require(clock.read() ==
              PcmClock::Point{
                  .epoch = maximum, .audio_epoch = maximum, .ticks = maximum, .host_ns = maximum},
          "Guest/audio epoch IDs and times retain all their integer bits");
  PcmClock disabled(0);
  disabled.append(block(0, initial_ticks), native_epoch);
  require(!disabled.read(), "An unqualified backend supplies no clock or retention grant");
  rejects([&] { disabled.observe(position(0)); });
}

void concurrent() {
  PcmClock clock(history_frames);
  std::atomic<bool> done{false};
  constexpr std::uint64_t count = 20000;
  constexpr std::uint64_t per_epoch = 1024;
  std::jthread writer([&] {
    for (std::uint64_t index = 1; index <= count; ++index) {
      const auto epoch = (index - 1) / per_epoch;
      const auto frame = (index - 1) % per_epoch;
      if (frame == 0) { clock.clear(); }
      const std::array<std::uint64_t, 1> ticks{index * 11};
      clock.append(block(frame, ticks, epoch * 2), epoch);
      clock.observe({.epoch = epoch,
                     .first_frame = frame,
                     .host_ns = index * 7,
                     .frames = 1,
                     .uncertainty_ns = index * 3});
    }
    done.store(true, std::memory_order_release);
  });
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  std::uint64_t previous = 0;
  do {
    if (const auto point = clock.read()) {
      const auto index = point->host_ns / 7;
      require(index != 0 && index >= previous && index <= count && point->host_ns == index * 7 &&
                  point->ticks == index * 11 && point->audio_epoch == (index - 1) / per_epoch &&
                  point->epoch == 2 * point->audio_epoch && point->uncertainty_ns == index * 3,
              "Concurrent readers receive coherent ordered points across independent epochs");
      previous = index;
    }
    require(std::chrono::steady_clock::now() < deadline, "Concurrent publication stays bounded");
  } while (!done.load(std::memory_order_acquire));
  writer.join();
  const auto point = clock.read();
  require(point && point->host_ns == count * 7 && point->ticks == count * 11,
          "Final quiescent publication survives concurrent reads");
}

void concurrent_native_lookup() {
  PcmClock clock(history_frames);
  blofeld::host::audio::RenderPositionStore native;
  std::atomic<bool> done{false};
  constexpr std::uint64_t count = 20000;
  constexpr std::uint64_t per_epoch = 257;
  constexpr std::uint64_t base = 1000000003;
  std::jthread writer([&] {
    for (std::uint64_t index = 1; index <= count; ++index) {
      const auto epoch = (index - 1) / per_epoch;
      const auto frame = (index - 1) % per_epoch;
      const auto origin = base + (epoch * 4096);
      if (frame == 0) { clock.clear(origin); }
      const std::array<std::uint64_t, 1> ticks{index * 11};
      clock.append(block(origin + frame, ticks, epoch * 2), epoch);
      require(
          native.publish({.epoch = epoch, .first_frame = frame, .host_ns = index * 7, .frames = 1}),
          "The authored native callback sequence remains representable");
    }
    done.store(true, std::memory_order_release);
  });
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  std::uint64_t previous = 0;
  do {
    if (const auto position = native.read()) {
      if (const auto point = clock.read(*position)) {
        const auto index = position->host_ns / 7;
        require(index != 0 && index >= previous && index <= count && point->ticks == index * 11 &&
                    point->host_ns == index * 7 && point->epoch == 2 * point->audio_epoch &&
                    point->audio_epoch == (index - 1) / per_epoch,
                "Concurrent first-callback lookups stay coherent across wrap and selected origins");
        previous = index;
      }
    }
    require(std::chrono::steady_clock::now() < deadline, "Native lookups remain bounded");
  } while (!done.load(std::memory_order_acquire));
  writer.join();
  const auto position = native.read();
  const auto point = position ? clock.read(*position) : std::nullopt;
  require(point && point->host_ns == count * 7 && point->ticks == count * 11 && !clock.read(),
          "Fresh native correlation needs no later producer observation or invented anchor");
}
}

int main() try {
  exact_points();
  correlation_uncertainty();
  selected_source_origin();
  source_origin_limits();
  retention_and_reset();
  native_failures();
  source_failures();
  full_range_and_disabled();
  concurrent();
  concurrent_native_lookup();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
