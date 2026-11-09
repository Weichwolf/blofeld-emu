// SPDX-License-Identifier: GPL-3.0-only
#include "audio/pulse_position.hpp"
#include "audio/render_position.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string_view>

namespace {
using blofeld::host::audio::PulsePosition;
using blofeld::host::audio::RenderPosition;
constexpr std::uint64_t query_bound = 256;
constexpr std::uint64_t steady_midpoint = 96, reset_midpoint = 384;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

PulsePosition::Snapshot snapshot() {
  return {
      .sent_ns = 8, .received_ns = 13, .submitted_frames = 16, .read_bytes = 64, .playing = true};
}

void rejects(PulsePosition &clock,
             const PulsePosition::Snapshot candidate,
             const std::string_view reason) {
  bool rejected = false;
  try {
    static_cast<void>(clock.observe(candidate));
  } catch (const std::runtime_error &error) {
    rejected = std::string_view(error.what()).contains(reason);
  }
  require(rejected, "Reject the exact native observation defect");
  rejected = false;
  try {
    static_cast<void>(clock.observe(snapshot()));
  } catch (const std::runtime_error &error) {
    rejected = std::string_view(error.what()).contains("unavailable");
  }
  require(rejected, "A failed native epoch stays unavailable until explicit reset");
}

void bounded_observations() {
  PulsePosition clock(query_bound);
  clock.reset(0);
  const auto first = clock.observe(snapshot());
  require(first ==
              RenderPosition{
                  .epoch = 0, .first_frame = 7, .host_ns = 10, .frames = 1, .uncertainty_ns = 3},
          "Observe the last consumed original frame with a conservative query envelope");
  auto next = snapshot();
  next.sent_ns = 16;
  next.received_ns = 32;
  require(!clock.observe(next), "Repeated server indices never create new audio frames or points");
  next.sent_ns = 64;
  next.received_ns = 128;
  next.read_bytes = 128;
  require(clock.observe(next) == RenderPosition{.epoch = 0,
                                                .first_frame = 15,
                                                .host_ns = steady_midpoint,
                                                .frames = 1,
                                                .uncertainty_ns = 32},
          "Native observations may skip consumed frames without counting them twice");
  clock.reset(128);
  next.sent_ns = 256;
  next.received_ns = 512;
  next.read_bytes = (128 + 8);
  next.submitted_frames = 1;
  require(clock.observe(next) == RenderPosition{.epoch = 1,
                                                .first_frame = 0,
                                                .host_ns = reset_midpoint,
                                                .frames = 1,
                                                .uncertainty_ns = 128},
          "A flushed absolute native index binds new epoch frame zero");
}

void priming_and_recovery() {
  PulsePosition clock(query_bound);
  rejects(clock, snapshot(), "unavailable");
  clock.reset(0);
  auto next = snapshot();
  next.playing = false;
  next.read_bytes = -8;
  require(!clock.observe(next), "A corked server cannot establish an audio epoch");
  next.sent_ns = 16;
  next.received_ns = 32;
  next.playing = true;
  next.read_bytes = 0;
  require(!clock.observe(next), "No consumed source frame means no invented position");
  next.sent_ns = 64;
  next.received_ns = 128;
  next.read_bytes = 8;
  require(clock.observe(next).has_value(), "The first complete consumed frame enables a point");
  next.sent_ns = 256;
  next.received_ns = 512;
  next.playing = false;
  rejects(clock, next, "stopped");
  clock.reset(0);
  require(clock.observe(snapshot()).has_value(), "A genuine new epoch can recover observation");
}

void invalid_observations() {
  auto invalid = snapshot();
  invalid.sent_ns = 0;
  auto backwards = snapshot();
  backwards.received_ns = backwards.sent_ns - 1;
  auto overdue = snapshot();
  overdue.received_ns = overdue.sent_ns + query_bound + 1;
  for (const auto candidate : {invalid, backwards, overdue}) {
    PulsePosition clock(query_bound);
    clock.reset(0);
    rejects(clock, candidate, "query");
  }
  for (const auto index : {-8, 1, (128 + 8)}) {
    PulsePosition clock(query_bound);
    clock.reset(0);
    auto candidate = snapshot();
    candidate.read_bytes = index;
    rejects(clock, candidate, "PCM");
  }
  PulsePosition corrupt(query_bound);
  corrupt.reset(0);
  auto candidate = snapshot();
  candidate.corrupt = true;
  rejects(corrupt, candidate, "corrupt");
  for (const bool overlap : {false, true}) {
    PulsePosition clock(query_bound);
    clock.reset(0);
    require(clock.observe(snapshot()).has_value(), "Initial valid snapshot");
    candidate = snapshot();
    candidate.sent_ns = overlap ? 12 : 16;
    candidate.received_ns = 32;
    candidate.read_bytes = overlap ? (9 * 8) : (7 * 8);
    rejects(clock, candidate, overlap ? "query" : "backwards");
  }
}

void limits() {
  bool rejected = false;
  try {
    const PulsePosition invalid(0);
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "Every native observation needs a finite nonzero query bound");
  PulsePosition clock(query_bound);
  for (const auto origin : {-8, 1}) {
    rejected = false;
    try {
      clock.reset(origin);
    } catch (const std::runtime_error &) { rejected = true; }
    require(rejected, "Native epoch origins must be nonnegative complete PCM frames");
  }
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  constexpr auto origin = std::numeric_limits<std::int64_t>::max() - 15;
  clock.reset(origin);
  const auto point = clock.observe({.sent_ns = maximum - 2,
                                    .received_ns = maximum,
                                    .submitted_frames = 1,
                                    .read_bytes = origin + 8,
                                    .playing = true});
  require(point && point->first_frame == 0 && point->host_ns == maximum - 1 &&
              point->uncertainty_ns == 1,
          "Large absolute indices and bounded native intervals keep their integer precision");
}
}

int main() try {
  bounded_observations();
  priming_and_recovery();
  invalid_observations();
  limits();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
