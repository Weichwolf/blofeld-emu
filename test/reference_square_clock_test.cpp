// SPDX-License-Identifier: GPL-3.0-only
#include "board/reference_square_clock.hpp"
#include "common/bus.hpp"
#include "esai_fixture.hpp"
#include "reference_audio_clock_fixture.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <string_view>

namespace blofeld::test::square_clock {
namespace {
using esai::rejects;
using esai::require;
using reference::SquareClock;
using reference::SquareClockConfig;
using reference::SquareClockLevel;
constexpr std::uint64_t base_hz = 1080000000;
constexpr std::uint64_t rate_44100 = 11289600;
constexpr std::uint64_t rate_48000 = 12288000;
constexpr std::uint64_t common_44100 = 211680000000;
constexpr std::uint64_t common_48000 = 138240000000;
constexpr std::uint64_t half_44100 = 9375;
constexpr std::uint64_t half_48000 = 5625;
constexpr std::uint64_t epoch = 123;

void literal_edges() {
  require(SquareClock::extend_timebase(base_hz, rate_44100) == common_44100 &&
              SquareClock::extend_timebase(base_hz, rate_48000) == common_48000,
          "Independent exact common clocks preserve the original MCU/DSP timebase");
  constexpr std::array<std::uint64_t, 8> expected{
      9498, 18873, 28248, 37623, 46998, 56373, 65748, 75123};
  for (const bool initial : {false, true}) {
    SquareClock source({.timebase_hz = common_44100,
                        .frequency_hz = rate_44100,
                        .epoch_ticks = epoch,
                        .initial_level = initial});
    require(source.current() == SquareClockLevel{.ticks = epoch, .level = initial} &&
                source.half_period_ticks() == half_44100,
            "An explicit epoch establishes a level without inventing a transition");
    for (std::size_t i = 0; i < expected.size(); ++i) {
      const auto edge = source.next();
      require(edge.ticks == expected.at(i) && edge.level == (initial == (i % 2 != 0)),
              "Literal absolute edge times and both starting polarities");
      require(source.next() == edge && source.level_at(edge.ticks - 1) != edge.level &&
                  source.level_at(edge.ticks) == edge.level,
              "Peeking is pure and a transition belongs to its exact boundary");
      source.consume(edge.ticks);
    }
  }
  require(
      SquareClock({.timebase_hz = common_48000, .frequency_hz = rate_48000}).half_period_ticks() ==
          half_48000,
      "48k uses a distinct exact half-period rather than rounding the old grid");
}

void rejects_and_limits() {
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  for (const auto config :
       {SquareClockConfig{},
        SquareClockConfig{.timebase_hz = base_hz},
        SquareClockConfig{.timebase_hz = base_hz, .frequency_hz = rate_44100},
        SquareClockConfig{.timebase_hz = 3, .frequency_hz = 1},
        SquareClockConfig{.timebase_hz = 2, .frequency_hz = 1, .epoch_ticks = maximum}}) {
    static_cast<void>(rejects([&] { static_cast<void>(SquareClock(config)); }));
  }
  for (const auto pair : {std::array<std::uint64_t, 2>{0, 1},
                          std::array<std::uint64_t, 2>{1, 0},
                          std::array<std::uint64_t, 2>{1, maximum},
                          std::array<std::uint64_t, 2>{maximum, 2}}) {
    static_cast<void>(
        rejects([&] { static_cast<void>(SquareClock::extend_timebase(pair[0], pair[1])); }));
  }
  SquareClock source(
      {.timebase_hz = common_44100, .frequency_hz = rate_44100, .epoch_ticks = epoch});
  const auto before = source;
  for (const auto at : {epoch, epoch + half_44100 - 1, epoch + (2 * half_44100)}) {
    static_cast<void>(rejects([&] { source.consume(at); }));
    require(source == before, "Early, late and skipped transitions leave the source unchanged");
  }
  static_cast<void>(rejects([&] { static_cast<void>(source.level_at(epoch - 1)); }));
  SquareClock end({.timebase_hz = 2, .frequency_hz = 1, .epoch_ticks = maximum - 1});
  end.consume(maximum);
  require(end.current().ticks == maximum && end.current().level,
          "The last representable transition is retained without wrapped time");
  const auto saved = end;
  require(rejects([&] { static_cast<void>(end.next()); }).contains("overflows") && end == saved,
          "Time exhaustion fails visibly rather than silently stopping an oscillator");
}

void long_phase() {
  constexpr std::uint64_t transitions = 1000000;
  constexpr std::uint64_t last_tick = 9375000000;
  SquareClock source({.timebase_hz = common_44100, .frequency_hz = rate_44100});
  for (std::uint64_t i = 0; i < transitions; ++i) { source.consume(source.next().ticks); }
  require(source.current() == SquareClockLevel{.ticks = last_tick, .level = false} &&
              source.next().ticks == last_tick + half_44100,
          "One million transitions retain the independent integer phase without drift");
}
}
}

int main(const int argc, char **argv) {
  try {
    if (argc != 2) { throw blofeld::Fault("Expected source, serial or snapshots"); }
    const std::string_view mode(argv[1]);
    if (mode == "source") {
      blofeld::test::square_clock::literal_edges();
      blofeld::test::square_clock::rejects_and_limits();
      blofeld::test::square_clock::long_phase();
    } else if (mode == "serial") {
      blofeld::test::square_clock::serial_checks();
    } else if (mode == "snapshots") {
      blofeld::test::square_clock::snapshot_checks();
    } else {
      throw blofeld::Fault("Unknown square clock test mode");
    }
    std::cout << "Exact reference square clock " << mode << " passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
