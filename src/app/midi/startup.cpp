// SPDX-License-Identifier: GPL-3.0-only
#include "midi/startup.hpp"

#include "io/midi_port.hpp"

#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace blofeld::app {
namespace {
constexpr std::uint8_t exclusive = 0xf0, end = 0xf7, realtime = 0xf8;
constexpr std::uint8_t universal = 0x7e, manufacturer = 0x3e, family = 0x13;
constexpr std::array<std::uint8_t, 6> query{exclusive, universal, 0x7f, 6, 1, end};
constexpr std::uint64_t interval = vi::MidiPort::reference_hz / 10;
}

void MidiStartup::receive(const std::uint8_t byte) noexcept {
  if (byte >= realtime) { return; }
  if (byte == exclusive) { used_ = 0; }
  if ((used_ == 0 && byte != exclusive) || used_ == reply_.size()) {
    used_ = 0;
    return;
  }
  reply_[used_++] = byte;
  if (byte != end) { return; }
  // Universal identity format, Waldorf manufacturer/family and model. Version
  // bytes are observed, not rewritten: original 1.25 identifies itself as 1.24.
  observations_.identity =
      observations_.identity ||
      (used_ == reply_.size() && reply_[1] == universal && reply_[2] < 0x80 && reply_[3] == 6 &&
       reply_[4] == 2 && reply_[5] == manufacturer && reply_[6] == family && reply_[7] == 0 &&
       reply_[8] == 0 && reply_[9] == 0 && reply_[10] < 0x80 && reply_[11] < 0x80 &&
       reply_[12] < 0x80 && reply_[13] < 0x80);
  used_ = 0;
}

std::optional<std::uint64_t> MidiStartup::poll(vi::MidiPort &port, const bool pcm) {
  while (const auto event = port.read_output()) {
    ++observations_.output_bytes;
    receive(event->byte);
  }
  if (port.error() != nullptr) { throw std::runtime_error(port.error()); }
  if (origin_) { return origin_; }
  const auto frontier = port.input_limit();
  if (!frontier) { throw std::logic_error("Offline startup requires a streaming MIDI input"); }
  if (observations_.identity && pcm) {
    origin_ = frontier;
    return origin_;
  }
  // Publish bounded silence/query windows. No guest clock advances on the host;
  // both original execution owners must actually reach the admission frontier.
  const auto time = port.time();
  if (time < *frontier && *frontier - time > vi::MidiPort::byte_ticks) { return {}; }
  if (*frontier > std::numeric_limits<std::uint64_t>::max() - interval) {
    throw std::overflow_error("Startup MIDI timeline is exhausted");
  }
  if ((!observations_.identity && !port.write_input(query, *frontier)) ||
      !port.advance_input(*frontier + interval)) {
    throw std::runtime_error(port.error() != nullptr ? port.error()
                                                     : "Startup MIDI admission failed");
  }
  if (!observations_.identity) { ++observations_.queries; }
  return {};
}
}
