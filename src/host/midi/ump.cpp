// SPDX-License-Identifier: GPL-3.0-only
#include "midi/ump.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>

namespace blofeld::host::midi {
namespace {
constexpr unsigned octet_mask = 0xff, nibble_mask = 0xf, group_shift = 24;
constexpr std::uint8_t time_code = 0xf1, song_position = 0xf2, song_select = 0xf3;
constexpr std::uint8_t tune = 0xf6, timing = 0xf8, start = 0xfa, resume = 0xfb;
constexpr std::uint8_t stop = 0xfc, sensing = 0xfe, reset = 0xff;
constexpr std::uint8_t exclusive_start = 0xf0, exclusive_end = 0xf7, realtime = 0xf8;

std::uint8_t octet(const std::uint32_t word, const unsigned shift) noexcept {
  return static_cast<std::uint8_t>((word >> shift) & octet_mask);
}
}

Ump::Ump(const std::uint8_t group) : group_(group) {
  if (group > nibble_mask) { throw std::invalid_argument("MIDI group exceeds 0..15"); }
}

bool Ump::fail(const char *reason) noexcept {
  error_ = reason;
  return false;
}

bool Ump::voice(Packet &packet, const std::uint8_t status) noexcept {
  if (exclusive_) { return fail("MIDI channel message interrupts SysEx"); }
  const auto kind = status >> 4U;
  if (kind < 8 || kind > 14) { return fail("Invalid MIDI 1.0 channel status"); }
  packet.size = kind == 12 || kind == 13 ? 2 : 3;
  if (packet.bytes[1] >= 0x80 || packet.bytes[2] >= 0x80 ||
      (packet.size == 2 && packet.bytes[2] != 0)) {
    return fail("Invalid MIDI 1.0 channel data or padding");
  }
  return true;
}

bool Ump::system(Packet &packet, const std::uint8_t status) noexcept {
  if (exclusive_ && status < realtime) { return fail("MIDI system message interrupts SysEx"); }
  switch (status) {
    case time_code:
    case song_select: packet.size = 2; break;
    case song_position: packet.size = 3; break;
    case tune:
    case timing:
    case start:
    case resume:
    case stop:
    case sensing:
    case reset: packet.size = 1; break;
    default: return fail("Unsupported MIDI system status");
  }
  if (packet.bytes[1] >= 0x80 || packet.bytes[2] >= 0x80 ||
      (packet.size < 3 && packet.bytes[2] != 0) || (packet.size == 1 && packet.bytes[1] != 0)) {
    return fail("Invalid MIDI system data or padding");
  }
  return true;
}

bool Ump::sysex(Packet &packet, const std::span<const std::uint32_t> words) noexcept {
  const auto descriptor = octet(words[0], 16);
  const auto status = descriptor >> 4U;
  const auto count = descriptor & nibble_mask;
  if (status > 3 || count > 6) { return fail("Invalid SysEx7 packet status or byte count"); }
  const bool begins = status < 2;
  const bool ends = status == 0 || status == 3;
  if (begins == exclusive_) { return fail("Inconsistent SysEx7 start/continuation"); }
  if (begins) { packet.bytes[packet.size++] = exclusive_start; }
  for (unsigned index = 0; index < 6; ++index) {
    const auto byte =
        index < 2 ? octet(words[0], (1 - index) * 8) : octet(words[1], (5 - index) * 8);
    if (byte >= 0x80 || (index >= count && byte != 0)) {
      return fail("Invalid SysEx7 data or padding");
    }
    if (index < count) { packet.bytes[packet.size++] = byte; }
  }
  if (ends) { packet.bytes[packet.size++] = exclusive_end; }
  exclusive_ = !ends;
  return true;
}

std::optional<Packet> Ump::decode(const std::span<const std::uint32_t> words,
                                  const std::uint64_t timestamp) noexcept {
  if (error_ != nullptr) { return {}; }
  if (words.empty()) {
    static_cast<void>(fail("Empty universal MIDI packet"));
    return {};
  }
  const auto type = words[0] >> 28U;
  const auto count = type == 3 ? 2U : 1U;
  if ((type != 1 && type != 2 && type != 3) || words.size() != count) {
    static_cast<void>(fail("Unsupported or truncated MIDI 1.0 universal packet"));
    return {};
  }
  if (((words[0] >> group_shift) & nibble_mask) != group_) { return {}; }
  Packet packet{.timestamp = timestamp};
  packet.bytes[0] = octet(words[0], 16);
  packet.bytes[1] = octet(words[0], 8);
  packet.bytes[2] = octet(words[0], 0);
  bool valid = false;
  switch (type) {
    case 1: valid = system(packet, packet.bytes[0]); break;
    case 2: valid = voice(packet, packet.bytes[0]); break;
    case 3: valid = sysex(packet, words); break;
    default: break;
  }
  return valid ? std::optional(packet) : std::nullopt;
}
}
