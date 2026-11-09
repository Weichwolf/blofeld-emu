// SPDX-License-Identifier: GPL-3.0-only
#include "options.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {
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
  require(render.wav == "out.wav" && render.midi == "score.mid" && render.tail_seconds == 12,
          "Offline input/output and tail are independent of option order");
  const auto zero =
      options(std::array{"--profile", "p", "--midi", "m", "--wav", "w", "--tail", "0"});
  require(zero.tail_seconds == 0, "An explicit zero tail is valid");
  rejects(std::array{"--profile"});
  rejects(std::array{"--profile", "p", "--midi", "m"});
  rejects(std::array{"--profile", "p", "--wav", "w"});
  rejects(std::array{"--profile", "p", "--tail", "5"});
  rejects(std::array{"--profile", "p", "--profile", "q"});
  rejects(std::array{"--unknown", "x"});
  rejects(std::array{"--profile", ""});
  for (const auto *value : {"-1", "1.5", "2s", "18446744073709551616"}) {
    rejects(std::array{"--profile", "p", "--midi", "m", "--wav", "w", "--tail", value});
  }
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
