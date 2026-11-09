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
      clock.observe({.epoch = epoch, .first_frame = frame, .host_ns = index * 7, .frames = 1});
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
                  point->epoch == 2 * point->audio_epoch,
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
}

int main() try {
  exact_points();
  retention_and_reset();
  native_failures();
  source_failures();
  full_range_and_disabled();
  concurrent();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
