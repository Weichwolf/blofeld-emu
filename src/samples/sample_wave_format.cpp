// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "samples/sample_wave.hpp"
#include "samples/sample_wave_detail.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace blofeld::samples::wave_detail {
std::uint64_t little(const Bytes bytes, const std::size_t offset, const unsigned width) {
  if (width > 8 || offset > bytes.size() || width > bytes.size() - offset) {
    throw Fault("Truncated WAV field");
  }
  std::uint64_t value = 0;
  for (unsigned i = 0; i < width; ++i) {
    value |= static_cast<std::uint64_t>(bytes[offset + i]) << (i * 8U);
  }
  return value;
}

namespace {
unsigned subtype(SampleWave &wave, const Bytes bytes, const unsigned type) {
  constexpr unsigned extensible = 0xfffe;
  constexpr unsigned extensible_bytes = 40;
  constexpr unsigned extension_bytes = 22;
  constexpr unsigned guid_offset = 24;
  constexpr unsigned format_ex_bytes = 18;
  constexpr unsigned mask_offset = 20;
  if (type != extensible) {
    if (bytes.size() != 16 && (bytes.size() != format_ex_bytes || little(bytes, 16, 2) != 0)) {
      throw Fault("Unsupported WAV format extension");
    }
    return type;
  }
  if (bytes.size() != extensible_bytes || little(bytes, 16, 2) != extension_bytes) {
    throw Fault("Invalid WAVEFORMATEXTENSIBLE length");
  }
  constexpr std::array<std::uint8_t, 14> guid_tail{
      0, 0, 0, 0, 0x10, 0, 0x80, 0, 0, 0xaa, 0, 0x38, 0x9b, 0x71};
  if (!std::ranges::equal(bytes.subspan(guid_offset + 2), guid_tail)) {
    throw Fault("Unsupported WAV subformat GUID");
  }
  wave.valid_bits = static_cast<unsigned>(little(bytes, format_ex_bytes, 2));
  wave.channel_mask = static_cast<std::uint32_t>(little(bytes, mask_offset, 4));
  if (wave.channel_mask != 0 &&
      static_cast<unsigned>(std::popcount(wave.channel_mask)) != wave.source_channels) {
    throw Fault("WAV speaker mask disagrees with the channel count");
  }
  return static_cast<unsigned>(little(bytes, guid_offset, 2));
}

double scalar(const SampleWave &wave, const Bytes bytes) {
  const auto raw = little(bytes, 0, wave.container_bits / 8);
  double value = 0;
  constexpr unsigned float_bits = 32;
  if (wave.floating_point) {
    value = wave.container_bits == float_bits
                ? static_cast<double>(std::bit_cast<float>(static_cast<std::uint32_t>(raw)))
                : std::bit_cast<double>(raw);
  } else {
    const auto padding = wave.container_bits - wave.valid_bits;
    if ((raw & ((std::uint64_t{1} << padding) - 1)) != 0) {
      throw Fault("Nonzero padding below the declared WAV PCM precision");
    }
    const auto sign = std::uint64_t{1} << (wave.container_bits - 1);
    const auto signed_value =
        wave.container_bits == 8
            ? static_cast<std::int64_t>(raw) - static_cast<std::int64_t>(sign)
            : static_cast<std::int64_t>(raw) -
                  ((raw & sign) != 0 ? static_cast<std::int64_t>(sign * 2) : 0);
    value = static_cast<double>(signed_value) / static_cast<double>(sign);
  }
  if (!std::isfinite(value)) { throw Fault("Nonfinite WAV audio sample"); }
  return value;
}
}

void format(SampleWave &wave, const Bytes bytes) {
  if (bytes.size() < 16) { throw Fault("Truncated WAV format"); }
  wave.source_channels = static_cast<unsigned>(little(bytes, 2, 2));
  wave.sample_rate = static_cast<unsigned>(little(bytes, 4, 4));
  wave.container_bits = static_cast<unsigned>(little(bytes, 14, 2));
  wave.valid_bits = wave.container_bits;
  const auto type = subtype(wave, bytes, static_cast<unsigned>(little(bytes, 0, 2)));
  wave.floating_point = type == 3;
  constexpr unsigned pcm_24 = 24;
  constexpr unsigned pcm_32 = 32;
  constexpr unsigned float_64 = 64;
  constexpr unsigned maximum_rate = 384000;
  const bool pcm = type == 1 && (wave.container_bits == 8 || wave.container_bits == 16 ||
                                 wave.container_bits == pcm_24 || wave.container_bits == pcm_32);
  const bool floating =
      wave.floating_point && (wave.container_bits == pcm_32 || wave.container_bits == float_64);
  if ((!pcm && !floating) || wave.valid_bits == 0 || wave.valid_bits > wave.container_bits ||
      (floating && wave.valid_bits != wave.container_bits) || wave.source_channels == 0 ||
      wave.source_channels > 2 || wave.sample_rate == 0 || wave.sample_rate > maximum_rate) {
    throw Fault("Unsupported WAV channels, encoding, precision or rate");
  }
  const auto alignment = wave.source_channels * (wave.container_bits / 8);
  if (little(bytes, 12, 2) != alignment ||
      little(bytes, 8, 4) != static_cast<std::uint64_t>(wave.sample_rate) * alignment) {
    throw Fault("WAV block alignment or byte rate disagrees with its format");
  }
}

void audio(SampleWave &wave, const Bytes bytes) {
  const auto width = wave.container_bits / 8;
  const auto alignment = wave.source_channels * width;
  if (bytes.empty() || bytes.size() % alignment != 0 ||
      bytes.size() / alignment > maximum_wave_frames) {
    throw Fault("Empty, incomplete or excessive WAV frames");
  }
  wave.mono.reserve(bytes.size() / alignment);
  for (std::size_t offset = 0; offset < bytes.size(); offset += alignment) {
    double value = 0;
    for (unsigned channel = 0; channel < wave.source_channels; ++channel) {
      value +=
          scalar(wave, bytes.subspan(offset + (static_cast<std::size_t>(channel) * width), width)) /
          static_cast<double>(wave.source_channels);
    }
    if (!std::isfinite(value)) { throw Fault("WAV mono conversion overflow"); }
    wave.mono.push_back(value);
  }
}
}
