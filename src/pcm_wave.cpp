// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/pcm_wave.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/pcm_recording.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string_view>
#include <vector>

namespace blofeld {
namespace {
constexpr unsigned container_bits = 32;
constexpr unsigned bytes_per_sample = 4;
constexpr unsigned header_bytes = 68;
constexpr unsigned riff_overhead = header_bytes - 8;

void little(std::vector<std::uint8_t> &bytes, std::uint32_t value, const unsigned width) {
  constexpr unsigned byte_mask = 0xff;
  for (unsigned i = 0; i < width; ++i) {
    bytes.push_back(static_cast<std::uint8_t>(value & byte_mask));
    value >>= 8U;
  }
}

void tag(std::vector<std::uint8_t> &bytes, const std::string_view value) {
  for (const auto character : value) { bytes.push_back(static_cast<std::uint8_t>(character)); }
}

void validate_timing(const PcmRecording &recording, const PcmWaveClock clock) {
  const auto [timebase, rate] = clock;
  const auto &format = recording.format();
  const auto frame_bits = static_cast<std::uint64_t>(format.slot_bits) * format.channels;
  const auto denominator = static_cast<std::uint64_t>(rate) * frame_bits;
  if (timebase == 0 || denominator == 0 || timebase % denominator != 0 || timebase < denominator) {
    throw Fault("WAV rate/bit clock is not integral in the declared timebase");
  }
  const auto bit_ticks = timebase / denominator;
  const auto span = (frame_bits - 1) * bit_ticks;
  std::optional<std::uint64_t> previous_end;
  for (const auto &frame : recording.frames()) {
    if (!frame.uniform_clock || !frame.bit_ticks || *frame.bit_ticks != bit_ticks ||
        frame.last_tick - frame.first_tick != span ||
        (previous_end &&
         (frame.first_tick < *previous_end || frame.first_tick - *previous_end != bit_ticks))) {
      throw Fault("PCM sampling ticks do not match the declared WAV rate");
    }
    previous_end = frame.last_tick;
  }
}

void header(std::vector<std::uint8_t> &bytes,
            const PcmRecording &recording,
            const std::uint32_t rate,
            const std::uint32_t payload) {
  const auto &format = recording.format();
  const auto alignment = format.channels * bytes_per_sample;
  constexpr unsigned extensible = 0xfffe;
  constexpr unsigned format_bytes = 40;
  constexpr unsigned extra_bytes = 22;
  tag(bytes, "RIFF");
  little(bytes, riff_overhead + payload, 4);
  tag(bytes, "WAVEfmt ");
  little(bytes, format_bytes, 4);
  little(bytes, extensible, 2);
  little(bytes, format.channels, 2);
  little(bytes, rate, 4);
  little(bytes, rate * alignment, 4);
  little(bytes, alignment, 2);
  little(bytes, container_bits, 2);
  little(bytes, extra_bytes, 2);
  little(bytes, format.word_bits, 2);
  little(bytes, 0, 4); // Direct-out channels; no guessed physical speaker mapping.
  constexpr std::array<std::uint8_t, 16> pcm_guid{
      1, 0, 0, 0, 0, 0, 0x10, 0, 0x80, 0, 0, 0xaa, 0, 0x38, 0x9b, 0x71};
  bytes.insert(bytes.end(), pcm_guid.begin(), pcm_guid.end());
  tag(bytes, "data");
  little(bytes, payload, 4);
}
}

std::vector<std::uint8_t> encode_pcm_wave(const PcmRecording &recording, const PcmWaveClock clock) {
  if (recording.frames().empty()) {
    throw Fault("Cannot publish WAV without a complete PCM frame");
  }
  validate_timing(recording, clock);
  const auto sample_rate = clock.sample_rate;
  const auto alignment = recording.format().channels * bytes_per_sample;
  const auto maximum = std::numeric_limits<std::uint32_t>::max();
  if (sample_rate > maximum / alignment ||
      recording.frames().size() > (maximum - riff_overhead) / alignment) {
    throw Fault("WAV byte rate or RIFF payload exceeds its bound");
  }
  const auto payload = static_cast<std::uint32_t>(recording.frames().size() * alignment);
  std::vector<std::uint8_t> bytes;
  bytes.reserve(static_cast<std::size_t>(header_bytes) + payload);
  header(bytes, recording, sample_rate, payload);
  const auto precision = recording.format().word_bits;
  if (precision == 0 || precision > container_bits) { throw Fault("Invalid WAV sample precision"); }
  const auto shift = container_bits - precision;
  for (const auto &frame : recording.frames()) {
    for (unsigned channel = 0; channel < frame.channels; ++channel) {
      const auto word = static_cast<std::uint32_t>(frame.samples.at(channel));
      little(bytes, word << shift, bytes_per_sample);
    }
  }
  return bytes;
}
}
