// SPDX-License-Identifier: GPL-3.0-only
#include "midi/ump.hpp"

#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>

namespace blofeld::host::midi {
namespace {
constexpr unsigned octet_mask = 0xff, nibble_mask = 0xf, group_shift = 24;
constexpr unsigned sysex_type = 0x30000000, sysex_status_shift = 20;
constexpr std::uint8_t time_code = 0xf1, song_position = 0xf2, song_select = 0xf3;
constexpr std::uint8_t tune = 0xf6, timing = 0xf8, start = 0xfa, resume = 0xfb;
constexpr std::uint8_t stop = 0xfc, sensing = 0xfe, reset = 0xff;
constexpr std::uint8_t exclusive_start = 0xf0, exclusive_end = 0xf7, realtime = 0xf8;

std::uint8_t octet(const std::uint32_t word, const unsigned shift) noexcept {
  return static_cast<std::uint8_t>((word >> shift) & octet_mask);
}

std::uint8_t channel_data(const std::uint8_t status) noexcept {
  const auto kind = status >> 4U;
  return kind == 12 || kind == 13 ? 1 : 2;
}

unsigned system_data(const std::uint8_t status) {
  switch (status) {
    case time_code:
    case song_select: return 1;
    case song_position: return 2;
    case tune:
    case timing:
    case start:
    case resume:
    case stop:
    case sensing:
    case reset: return 0;
    default: return 3;
  }
}

std::uint32_t checked_group(const std::uint8_t group) {
  if (group > nibble_mask) { throw std::invalid_argument("MIDI group exceeds 0..15"); }
  return group;
}
}

DinEncoder::DinEncoder(const std::uint8_t group) : group_(checked_group(group) << group_shift) {}

void DinEncoder::message(Batch &batch,
                         const std::uint8_t status,
                         const std::uint8_t first,
                         const std::uint8_t second) const noexcept {
  const auto type = status < exclusive_start ? 0x20000000U : 0x10000000U;
  batch.words[batch.size++] =
      type | group_ | (std::uint32_t{status} << 16U) | (std::uint32_t{first} << 8U) | second;
}

void DinEncoder::fragment(Batch &batch, const bool end) noexcept {
  enum class Fragment : std::uint8_t { complete, start, continuation, end };
  const auto initial = exclusive_ == Exclusive::start;
  auto status = initial ? Fragment::start : Fragment::continuation;
  if (end) { status = initial ? Fragment::complete : Fragment::end; }
  auto high = sysex_type | group_ | (static_cast<unsigned>(status) << sysex_status_shift) |
              (std::uint32_t{used_} << 16U);
  std::uint32_t low = 0;
  for (unsigned index = 0; index < used_; ++index) {
    if (index < 2) {
      high |= std::uint32_t{bytes_[index]} << ((1U - index) * 8U);
    } else {
      low |= std::uint32_t{bytes_[index]} << ((5U - index) * 8U);
    }
  }
  batch.words[batch.size++] = high;
  batch.words[batch.size++] = low;
  used_ = 0;
  exclusive_ = end ? Exclusive::idle : Exclusive::continuation;
}

void DinEncoder::system_exclusive(Batch &batch, const std::uint8_t byte) {
  if (byte == exclusive_end) {
    fragment(batch, true);
    return;
  }
  if (byte >= 0x80) { throw std::runtime_error("DIN status interrupts SysEx before EOX"); }
  if (used_ == bytes_.size()) { fragment(batch, false); }
  bytes_[used_++] = byte;
}

void DinEncoder::begin_message(Batch &batch, const std::uint8_t status) {
  // A new status replaces an incomplete non-realtime MIDI message.
  remaining_ = 0;
  used_ = 0;
  status_ = status;
  if (status < exclusive_start) {
    running_ = status;
    remaining_ = channel_data(status);
    return;
  }
  running_ = 0;
  if (status == exclusive_start) {
    exclusive_ = Exclusive::start;
    return;
  }
  const auto count = system_data(status);
  if (count == 3) { throw std::runtime_error("Undefined DIN system status"); }
  if (count == 0) {
    message(batch, status);
  } else {
    remaining_ = static_cast<std::uint8_t>(count);
  }
}

DinEncoder::Batch DinEncoder::push(const std::uint8_t byte) {
  Batch result;
  if (byte >= realtime) {
    if (system_data(byte) != 0) { throw std::runtime_error("Undefined DIN real-time status"); }
    if (exclusive_ != Exclusive::idle && (used_ != 0 || exclusive_ == Exclusive::start)) {
      fragment(result, false);
    }
    message(result, byte);
  } else if (exclusive_ != Exclusive::idle) {
    system_exclusive(result, byte);
  } else if (byte >= 0x80) {
    begin_message(result, byte);
  } else {
    if (remaining_ == 0) {
      if (running_ == 0) { return result; }
      status_ = running_;
      remaining_ = channel_data(running_);
      used_ = 0;
    }
    bytes_[used_++] = byte;
    if (--remaining_ == 0) { message(result, status_, bytes_[0], used_ == 2 ? bytes_[1] : 0); }
  }
  return result;
}

Ump::Ump(const std::uint8_t group) : group_(static_cast<std::uint8_t>(checked_group(group))) {}

bool Ump::fail(const char *reason) noexcept {
  error_ = reason;
  return false;
}

bool Ump::voice(Packet &packet, const std::uint8_t status) noexcept {
  if (exclusive_) { return fail("MIDI channel message interrupts SysEx"); }
  const auto kind = status >> 4U;
  if (kind < 8 || kind > 14) { return fail("Invalid MIDI 1.0 channel status"); }
  packet.size = static_cast<std::uint8_t>(channel_data(status) + 1);
  if (packet.bytes[1] >= 0x80 || packet.bytes[2] >= 0x80 ||
      (packet.size == 2 && packet.bytes[2] != 0)) {
    return fail("Invalid MIDI 1.0 channel data or padding");
  }
  return true;
}

bool Ump::system(Packet &packet, const std::uint8_t status) noexcept {
  if (exclusive_ && status < realtime) { return fail("MIDI system message interrupts SysEx"); }
  const auto count = system_data(status);
  if (count == 3) { return fail("Unsupported MIDI system status"); }
  packet.size = static_cast<std::uint8_t>(count + 1);
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
