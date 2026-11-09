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
  bool profile = false, tail = false, group = false;
};

template <typename T> T number(const std::string_view value, const char *message) {
  T result{};
  const auto parsed = std::from_chars(value.data(), value.data() + value.size(), result);
  if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size()) {
    throw std::invalid_argument(message);
  }
  return result;
}

void validate(const Options &result, const Provided &provided) {
  if (result.midi && (result.midi_source || provided.group)) {
    throw std::invalid_argument("Native MIDI routing is only available in live mode");
  }
  if (result.midi.has_value() != result.wav.has_value() || (provided.tail && !result.midi)) {
    throw std::invalid_argument("Offline rendering requires both --midi and --wav");
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
    if (key == "--profile" && !provided.profile) {
      provided.profile = true;
      result.profile = value;
    } else if (key == "--midi" && !result.midi) {
      result.midi = value;
    } else if (key == "--wav" && !result.wav) {
      result.wav = value;
    } else if (key == "--midi-source" && !result.midi_source) {
      result.midi_source = number<std::int32_t>(value, "--midi-source requires a signed native ID");
    } else if (key == "--midi-group" && !provided.group) {
      provided.group = true;
      const auto parsed = number<unsigned>(value, "--midi-group requires an integer from 0 to 15");
      if (parsed > 15) {
        throw std::invalid_argument("--midi-group requires an integer from 0 to 15");
      }
      result.midi_group = static_cast<std::uint8_t>(parsed);
    } else if (key == "--tail" && !provided.tail) {
      provided.tail = true;
      result.tail_seconds =
          number<std::uint64_t>(value, "--tail requires whole nonnegative seconds");
    } else {
      throw std::invalid_argument(
          "Usage: blofeld-headless [--profile FILE] [--midi-source ID --midi-group 0..15] "
          "[--midi FILE --wav FILE [--tail SECONDS]]");
    }
  }
  validate(result, provided);
  if (!provided.profile) { result.profile = default_profile(); }
  return result;
}
}
