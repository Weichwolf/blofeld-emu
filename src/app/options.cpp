// SPDX-License-Identifier: GPL-3.0-only
#include "options.hpp"

#include "profile.hpp"

#include <charconv>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>
#include <string_view>
#include <system_error>

namespace blofeld::app {
namespace {
struct Provided {
  bool profile = false, tail = false, group = false, watchdog = false;
};

struct Argument {
  std::string_view key, value;
};

template <typename T> T number(const std::string_view value, const char *message) {
  T result{};
  const auto parsed = std::from_chars(value.data(), value.data() + value.size(), result);
  if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size()) {
    throw std::invalid_argument(message);
  }
  return result;
}

bool live_route(Options &result, Provided &provided, const Argument argument) {
  const auto [key, value] = argument;
#if defined(__linux__)
  if (key == "--alsa-device" && result.alsa_device.empty()) {
    if (!value.starts_with("hw:") || value.size() == 3) {
      throw std::invalid_argument("--alsa-device requires an ALSA hw: PCM device");
    }
    result.alsa_device = value;
    return true;
  }
#endif
  if (key == "--midi-source" && !result.midi_source) {
    result.midi_source = number<std::int32_t>(value, "--midi-source requires a signed native ID");
    return true;
  }
  if (key == "--midi-destination" && !result.midi_destination) {
    result.midi_destination =
        number<std::int32_t>(value, "--midi-destination requires a signed native ID");
    return true;
  }
  if (key != "--midi-group" || provided.group) { return false; }
  const auto group = number<unsigned>(value, "--midi-group requires an integer from 0 to 15");
  if (group > 15) { throw std::invalid_argument("--midi-group requires an integer from 0 to 15"); }
  provided.group = true;
  result.midi_group = static_cast<std::uint8_t>(group);
  return true;
}

void validate(const Options &result, const Provided &provided) {
#if defined(__linux__)
  if (!result.alsa_device.empty() && (result.midi || result.wav)) {
    throw std::invalid_argument("Direct ALSA PCM is only available in live mode");
  }
#endif
  if (result.midi && (result.midi_source || result.midi_destination || provided.group)) {
    throw std::invalid_argument("Native MIDI routing is only available in live mode");
  }
  if (result.midi.has_value() != result.wav.has_value() ||
      ((provided.tail || provided.watchdog) && !result.midi)) {
    throw std::invalid_argument("Offline rendering requires both --midi and --wav");
  }
  if (result.startup_watchdog_seconds == 0) {
    throw std::invalid_argument("--startup-watchdog requires positive whole seconds");
  }
}

}

Options options(const std::span<const char *const> arguments) {
  Options result;
  Provided provided;
  for (std::size_t index = 0; index < arguments.size(); ++index) {
    const std::string_view key(arguments[index]);
    if (++index == arguments.size()) { throw std::invalid_argument("Missing option value"); }
    const std::string_view value(arguments[index]);
    if (value.empty()) { throw std::invalid_argument("Empty option value"); }
    if (live_route(result, provided, {.key = key, .value = value})) { continue; }
    if (key == "--profile" && !provided.profile) {
      provided.profile = true;
      result.profile = value;
    } else if (key == "--midi" && !result.midi) {
      result.midi = value;
    } else if (key == "--wav" && !result.wav) {
      result.wav = value;
    } else if (key == "--tail" && !provided.tail) {
      provided.tail = true;
      result.tail_seconds =
          number<std::uint64_t>(value, "--tail requires whole nonnegative seconds");
    } else if (key == "--startup-watchdog" && !provided.watchdog) {
      provided.watchdog = true;
      result.startup_watchdog_seconds =
          number<std::uint64_t>(value, "--startup-watchdog requires positive whole seconds");
    } else {
      throw std::invalid_argument(
          "Usage: blofeld-headless [--profile FILE] [--midi-source ID] "
          "[--midi-destination ID] [--midi-group 0..15] "
#if defined(__linux__)
          "[--alsa-device hw:CARD,DEVICE] "
#endif
          "[--midi FILE --wav FILE [--tail SECONDS] [--startup-watchdog SECONDS]]");
    }
  }
  validate(result, provided);
  if (!provided.profile) { result.profile = default_profile(); }
  return result;
}
}
