// SPDX-License-Identifier: GPL-3.0-only
#include "options.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#if defined(__APPLE__)
#include <limits>
#endif
#include <stdexcept>

namespace {
constexpr std::uint64_t default_watchdog = 600, extended_watchdog = 1200;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void rejects(const auto &arguments) {
  bool rejected = false;
  try {
    static_cast<void>(blofeld::app::options(arguments));
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "Reject incomplete, duplicate and unknown command-line options");
}
}

int main() try {
  using blofeld::app::options;
  const auto live = options(std::array{"--profile", "local.profile"});
  require(live.profile == "local.profile" && !live.midi && !live.wav, "Profile selects live mode");
  const auto render = options(std::array{
      "--wav", "out.wav", "--profile", "local.profile", "--tail", "12", "--midi", "score.mid"});
  require(render.wav == "out.wav" && render.midi == "score.mid" && render.tail_seconds == 12 &&
              render.startup_watchdog_seconds == default_watchdog,
          "Offline input/output and tail are independent of option order");
  const auto zero =
      options(std::array{"--profile", "p", "--midi", "m", "--wav", "w", "--tail", "0"});
  require(zero.tail_seconds == 0, "An explicit zero tail is valid");
  const auto watchdog = options(
      std::array{"--startup-watchdog", "1200", "--profile", "p", "--midi", "m", "--wav", "w"});
  require(watchdog.startup_watchdog_seconds == extended_watchdog,
          "The offline host watchdog is configurable independently of guest time");
  rejects(std::array{"--profile", "p", "--startup-watchdog", "60"});
  rejects(std::array{"--profile", "p", "--midi", "m", "--wav", "w", "--startup-watchdog", "0"});
  rejects(std::array{
      "--midi", "m", "--wav", "w", "--startup-watchdog", "1", "--startup-watchdog", "2"});
#if defined(__APPLE__)
  const auto routed = options(std::array{"--profile",
                                         "p",
                                         "--midi-group",
                                         "15",
                                         "--midi-source",
                                         "-2147483648",
                                         "--midi-destination",
                                         "123"});
  constexpr std::int32_t destination = 123;
  require(routed.midi_source == std::numeric_limits<std::int32_t>::min() &&
              routed.midi_destination == destination && routed.midi_group == 15,
          "Preserve signed native IDs and select an explicit UMP group");
  rejects(std::array{"--midi-source", "1", "--midi-source", "2"});
  rejects(std::array{"--midi-destination", "1", "--midi-destination", "2"});
  rejects(std::array{"--midi-group", "0", "--midi-group", "1"});
  rejects(std::array{"--midi-group", "16"});
  rejects(std::array{"--midi-source", "2147483648"});
  rejects(std::array{"--midi-source", "1", "--midi", "m", "--wav", "w"});
  rejects(std::array{"--midi-destination", "1", "--midi", "m", "--wav", "w"});
  rejects(std::array{"--midi-group", "0", "--midi", "m", "--wav", "w"});
#endif
  rejects(std::array{"--profile"});
  rejects(std::array{"--profile", "p", "--midi", "m"});
  rejects(std::array{"--profile", "p", "--wav", "w"});
  rejects(std::array{"--profile", "p", "--tail", "5"});
  rejects(std::array{"--profile", "p", "--profile", "q"});
  rejects(std::array{"--unknown", "x"});
  rejects(std::array{"--profile", ""});
  for (const auto *value : {"-1", "1.5", "2s", "18446744073709551616"}) {
    rejects(std::array{"--profile", "p", "--midi", "m", "--wav", "w", "--tail", value});
    rejects(std::array{"--profile", "p", "--midi", "m", "--wav", "w", "--startup-watchdog", value});
  }
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
