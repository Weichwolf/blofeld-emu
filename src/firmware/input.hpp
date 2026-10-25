// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace blofeld::input {
using Bytes = std::vector<std::uint8_t>;
inline constexpr std::size_t input_limit = std::size_t{64} * 1024 * 1024;
inline constexpr unsigned midi_data_mask = 0x7f;
inline constexpr std::uint8_t midi_sysex_start = 0xf0;
inline constexpr std::uint8_t midi_sysex_end = 0xf7;
inline constexpr std::uint8_t midi_meta = 0xff;
inline constexpr std::uint8_t midi_end_track = 0x2f;
inline constexpr std::uint8_t midi_status_mask = 0xf0;

class Reader {
public:
  explicit Reader(const std::span<const std::uint8_t> data) : data_(data) {}

  std::span<const std::uint8_t> take(const std::size_t length) {
    if (length > data_.size() - position_) {
      throw Fault("Truncated firmware input at " + hex(position_));
    }
    const auto result = data_.subspan(position_, length);
    position_ += length;
    return result;
  }

  std::uint8_t byte() { return take(1)[0]; }

  std::uint32_t big(const unsigned count) {
    std::uint32_t result = 0;
    for (const auto byte : take(count)) { result = (result << 8U) | byte; }
    return result;
  }

  std::uint32_t little(const unsigned count) {
    std::uint32_t result = 0;
    unsigned shift = 0;
    for (const auto byte : take(count)) {
      result |= static_cast<std::uint32_t>(byte) << shift;
      shift += 8;
    }
    return result;
  }

  std::uint32_t vlq() {
    std::uint32_t result = 0;
    for (unsigned index = 0; index < 4; ++index) {
      const auto value = byte();
      result = (result << 7U) | (value & midi_data_mask);
      if (value < 0x80) { return result; }
    }
    throw Fault("Oversized MIDI variable-length quantity");
  }

  [[nodiscard]] std::span<const std::uint8_t> rest() const { return data_.subspan(position_); }

  [[nodiscard]] bool empty() const { return position_ == data_.size(); }

private:
  std::span<const std::uint8_t> data_;
  std::size_t position_ = 0;
};

Bytes archive_midi(const Bytes &input);
std::vector<Bytes> sysex_messages(const Bytes &input);
}
