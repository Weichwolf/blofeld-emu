// SPDX-License-Identifier: GPL-3.0-only
#include "audio/render_timeline.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>

namespace {
using blofeld::host::audio::RenderTimeline;
using Error = RenderTimeline::Error;
constexpr std::uint64_t first_host = 100, next_host = 200;
constexpr double fraction = 0.25, precision_limit = 0x1p53;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void cadence() {
  constexpr std::uint32_t periods = 1000;
  constexpr std::uint32_t numerator = 147;
  constexpr std::uint32_t denominator = 5;
  constexpr std::uint64_t host_begin = 100;
  constexpr std::uint64_t host_quantum = 16000;
  constexpr std::array origins{-1024.25, 0.0, fraction, 0x1p40};
  for (const auto origin : origins) {
    RenderTimeline timeline;
    for (std::uint32_t period = 0; period < periods; ++period) {
      const auto first = (period * numerator + denominator - 1) / denominator;
      const auto end = ((period + 1) * numerator + denominator - 1) / denominator;
      require(!timeline.advance({.sample = origin + first,
                                 .host = host_begin + (period * host_quantum),
                                 .frames = end - first}),
              "Rational 44.1/48 kHz conversion preserves every variable callback boundary");
    }
  }
}

void rejection(const RenderTimeline::Request request, const Error error) {
  RenderTimeline timeline;
  require(!timeline.advance({.sample = 0, .host = first_host, .frames = 16}),
          "Establish native origin");
  const auto failure = timeline.advance(request);
  if (!failure) { throw std::runtime_error("An invalid render request must fail"); }
  require(failure->error == error && failure->expected == 16 &&
              failure->previous_host == first_host && failure->request.frames == request.frames &&
              failure->request.host == request.host,
          "Rejected render records the expected boundary and actual failing request");
  const auto repeated = timeline.advance({.sample = 16, .host = next_host, .frames = 16});
  require(repeated && repeated->error == error && repeated->expected == failure->expected &&
              repeated->request.host == failure->request.host,
          "A later valid callback cannot hide a previous lost or repeated render interval");
  timeline.reset();
  require(!timeline.advance({.sample = fraction, .host = 1, .frames = 1}) &&
              !timeline.advance({.sample = 1 + fraction, .host = 2, .frames = 1}),
          "A quiescent epoch reset establishes a fresh native origin");
}

void invalid() {
  constexpr std::array jumps{-1.0, 0.0, 15.0, 16.25, 17.0, 32.0};
  for (const double sample : jumps) {
    rejection({.sample = sample, .host = next_host, .frames = 16}, Error::discontinuity);
  }
  for (const auto sample : {std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::infinity(),
                            -std::numeric_limits<double>::infinity()}) {
    rejection({.sample = sample, .host = next_host, .frames = 16}, Error::sample_time);
  }
  rejection({.sample = {}, .host = next_host, .frames = 16}, Error::sample_time);
  constexpr std::array hosts{std::uint64_t{0}, std::uint64_t{1}, first_host - 1, first_host};
  for (const auto host : hosts) {
    rejection({.sample = 16, .host = host, .frames = 16}, Error::host_time);
  }
  rejection({.sample = 16, .host = {}, .frames = 16}, Error::host_time);
  rejection({.sample = 16, .host = next_host, .frames = 0}, Error::frame_count);
}

void range() {
  for (const auto sample : {precision_limit,
                            std::numeric_limits<double>::max(),
                            std::numeric_limits<double>::lowest()}) {
    RenderTimeline timeline;
    const auto failure = timeline.advance({.sample = sample, .host = 1, .frames = 1});
    require(failure && failure->error == Error::exhausted && !failure->expected,
            "A native sample counter must represent the entire requested interval exactly");
  }
  RenderTimeline timeline;
  constexpr auto maximum = std::numeric_limits<std::uint32_t>::max();
  require(!timeline.advance({.sample = 0, .host = 1, .frames = maximum}) &&
              !timeline.advance({.sample = double{maximum}, .host = 2, .frames = 1}),
          "The full callback count range crosses 32-bit sample positions without truncation");
}
}

int main() try {
  cadence();
  invalid();
  range();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
