// SPDX-License-Identifier: GPL-3.0-only
#include "samples/sample_wave.hpp"

#include "common/bus.hpp"
#include "samples/sample_wave_detail.hpp"
#include "storage/private_file.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string_view>

namespace blofeld::samples {
namespace {
using wave_detail::Bytes;

bool tag(const Bytes bytes, const std::string_view value) {
  return bytes.size() >= value.size() && std::ranges::equal(bytes.first(value.size()), value);
}

struct Chunks {
  std::optional<Bytes> format;
  std::optional<Bytes> data;
  std::optional<Bytes> sampler;
  std::optional<Bytes> instrument;
  std::optional<Bytes> fact;

  static void unique(std::optional<Bytes> &field, const Bytes payload) {
    if (field) { throw Fault("Duplicate WAV format/data/metadata chunk"); }
    field = payload;
  }

  void add(const Bytes chunk) {
    const auto header = chunk.first(8);
    const auto payload = chunk.subspan(8);
    if (tag(header, "fmt ")) {
      unique(format, payload);
    } else if (tag(header, "data")) {
      unique(data, payload);
    } else if (tag(header, "smpl")) {
      unique(sampler, payload);
    } else if (tag(header, "inst")) {
      unique(instrument, payload);
    } else if (tag(header, "fact")) {
      unique(fact, payload);
    }
    // RIFF permits ancillary chunks; never interpret their bytes as audio.
  }
};

Chunks chunks(const Bytes bytes) {
  if (bytes.size() < 12 || bytes.size() > maximum_wave_bytes || !tag(bytes, "RIFF") ||
      !tag(bytes.subspan(8), "WAVE") || wave_detail::little(bytes, 4, 4) != bytes.size() - 8) {
    throw Fault("Unsupported, truncated or oversized RIFF/WAVE file");
  }
  Chunks result;
  std::size_t offset = 12;
  unsigned count = 0;
  constexpr unsigned maximum_chunks = 16384;
  while (offset < bytes.size()) {
    if (++count > maximum_chunks || bytes.size() - offset < 8) {
      throw Fault("Excessive or truncated WAV chunk headers");
    }
    const auto header = bytes.subspan(offset, 8);
    offset += 8;
    const auto size = static_cast<std::size_t>(wave_detail::little(header, 4, 4));
    if (size > bytes.size() - offset || size + (size & 1U) > bytes.size() - offset) {
      throw Fault("WAV chunk exceeds the RIFF boundary or lacks padding");
    }
    result.add(bytes.subspan(offset - 8, size + 8));
    offset += size + (size & 1U);
  }
  return result;
}
}

SampleWave decode_sample_wave(const std::span<const std::uint8_t> bytes) {
  const auto input = chunks(bytes);
  if (!input.format || !input.data) { throw Fault("WAV needs one fmt and one data chunk"); }
  SampleWave result;
  wave_detail::format(result, *input.format);
  wave_detail::audio(result, *input.data);
  if (input.fact &&
      (input.fact->size() < 4 || wave_detail::little(*input.fact, 0, 4) != result.mono.size())) {
    throw Fault("WAV fact sample count disagrees with complete audio frames");
  }
  wave_detail::metadata(
      result, input.sampler.value_or(Bytes{}), input.instrument.value_or(Bytes{}));
  if ((input.sampler && input.sampler->empty()) ||
      (input.instrument && input.instrument->empty())) {
    throw Fault("Empty WAV sampler/instrument metadata");
  }
  return result;
}

SampleWave load_sample_wave(const std::filesystem::path &path) {
  const auto bytes = read_private_file(path, maximum_wave_bytes);
  return decode_sample_wave(bytes);
}

double mono_peak(const SampleWave &wave) {
  double peak = 0;
  for (const auto value : wave.mono) {
    if (!std::isfinite(value)) { throw Fault("Nonfinite decoded sample"); }
    peak = std::max(peak, std::abs(value));
  }
  return peak;
}

std::optional<double> metadata_root(const SampleWave &wave) {
  if (wave.sampler_root) { return wave.sampler_root; }
  if (wave.instrument) {
    return static_cast<double>(wave.instrument->note) -
           (static_cast<double>(wave.instrument->correction_cents) / 100.0);
  }
  return std::nullopt;
}
}
