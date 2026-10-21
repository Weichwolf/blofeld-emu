// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <string_view>
#include <utility>
#include <vector>

namespace sample_wave_fixture {
using Bytes = std::vector<std::uint8_t>;

inline void little(Bytes &bytes, std::uint64_t value, const unsigned width) {
  constexpr unsigned byte_mask = 0xff;
  for (unsigned i = 0; i < width; ++i) {
    bytes.push_back(static_cast<std::uint8_t>(value & byte_mask));
    value >>= 8U;
  }
}

inline void tag(Bytes &bytes, const std::string_view text) {
  for (const auto c : text) { bytes.push_back(static_cast<std::uint8_t>(c)); }
}

inline Bytes
format(const unsigned bits = 16, const unsigned channels = 1, const unsigned type = 1) {
  Bytes bytes;
  little(bytes, type, 2);
  little(bytes, channels, 2);
  constexpr unsigned sample_rate = 8000;
  little(bytes, sample_rate, 4);
  little(bytes, static_cast<std::uint64_t>(sample_rate) * channels * (bits / 8), 4);
  little(bytes, static_cast<std::uint64_t>(channels) * (bits / 8), 2);
  little(bytes, bits, 2);
  return bytes;
}

inline Bytes file(const std::vector<std::pair<std::string_view, Bytes>> &chunks) {
  Bytes body;
  tag(body, "WAVE");
  for (const auto &[name, data] : chunks) {
    tag(body, name);
    little(body, data.size(), 4);
    body.insert(body.end(), data.begin(), data.end());
    if (data.size() % 2 != 0) { body.push_back(0); }
  }
  Bytes result;
  tag(result, "RIFF");
  little(result, body.size(), 4);
  result.insert(result.end(), body.begin(), body.end());
  return result;
}

inline Bytes sampler() {
  Bytes result(12, 0);
  constexpr unsigned root = 60;
  little(result, root, 4);
  constexpr unsigned half_semitone_shift = 31;
  little(result, std::uint32_t{1} << half_semitone_shift, 4); // Half a semitone above MIDI 60.
  little(result, 0, 4);
  little(result, 0, 4);
  little(result, 1, 4);
  little(result, 0, 4);
  for (const auto v : {7U, 1U, 1U, 3U, 0U, 2U}) { little(result, v, 4); }
  return result;
}
}
