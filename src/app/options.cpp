// SPDX-License-Identifier: GPL-3.0-only
#include "options.hpp"

#include "profile.hpp"

#include <charconv>
#include <cstddef>
#include <optional>
#include <span>
#include <stdexcept>
#include <string_view>
#include <system_error>

namespace blofeld::app {
Options options(const std::span<const char *const> arguments) {
  Options result;
  bool profile = false;
  bool tail = false;
  for (std::size_t index = 0; index < arguments.size(); ++index) {
    const std::string_view key(arguments[index]);
    if (++index == arguments.size()) { throw std::invalid_argument("Missing option value"); }
    const std::string_view value(arguments[index]);
    if (value.empty()) { throw std::invalid_argument("Empty option value"); }
    if (key == "--profile" && !profile) {
      profile = true;
      result.profile = value;
    } else if (key == "--midi" && !result.midi) {
      result.midi = value;
    } else if (key == "--wav" && !result.wav) {
      result.wav = value;
    } else if (key == "--tail" && !tail) {
      tail = true;
      const auto parsed =
          std::from_chars(value.data(), value.data() + value.size(), result.tail_seconds);
      if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size()) {
        throw std::invalid_argument("--tail requires whole nonnegative seconds");
      }
    } else {
      throw std::invalid_argument(
          "Usage: blofeld-headless [--profile FILE] [--midi FILE --wav FILE [--tail SECONDS]]");
    }
  }
  if (result.midi.has_value() != result.wav.has_value() || (tail && !result.midi)) {
    throw std::invalid_argument("Offline rendering requires both --midi and --wav");
  }
  if (!profile) { result.profile = default_profile(); }
  return result;
}
}
