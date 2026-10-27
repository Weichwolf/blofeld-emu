// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "midi/midi1_status.hpp"
#include "midi/sequence_codec.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <sound/asequencer.h>
#include <string>

namespace blofeld::detail {
namespace {
constexpr unsigned status_bit = midi1::note_off, system = midi1::family_mask,
                   realtime_start = midi1::clock, channel_mask = 15;
constexpr unsigned bend_bias = 8192;

unsigned message_length(const std::uint8_t value) {
  if (value < system) {
    return (value & system) == midi1::program_change || (value & system) == midi1::channel_pressure
               ? 2U
               : 3U;
  }
  if (value == midi1::quarter_frame || value == midi1::song_select) { return 2; }
  if (value == midi1::song_position) { return 3; }
  if (value == midi1::tune_request) { return 1; }
  throw Fault("Unsupported host MIDI output status=" + std::to_string(value));
}
}

SequenceEncoder::SequenceEncoder(const std::size_t fragment_limit)
    : fragment_limit_(fragment_limit) {
  if (fragment_limit == 0 || fragment_limit > sysex_bytes_.size()) {
    throw Fault("Host MIDI SysEx fragment limit exceeds its bounded packet");
  }
}

const SequencePacket *SequenceEncoder::pending() const {
  return ready_ ? &packet_ : nullptr;
}

void SequenceEncoder::consume() {
  if (!ready_) { throw Fault("Host MIDI output has no completed packet"); }
  ready_ = false;
}

void SequenceEncoder::fragment() {
  packet_ = {};
  packet_.event.type = SNDRV_SEQ_EVENT_SYSEX;
  packet_.event.flags = SNDRV_SEQ_EVENT_LENGTH_VARIABLE;
  packet_.event.data.ext.len = static_cast<unsigned>(sysex_size_);
  packet_.size = sysex_size_;
  std::copy_n(sysex_bytes_.begin(), sysex_size_, packet_.data.begin());
  sysex_size_ = 0;
  ready_ = true;
}

void SequenceEncoder::realtime(const std::uint8_t value) {
  packet_ = {};
  switch (value) {
    case midi1::clock: packet_.event.type = SNDRV_SEQ_EVENT_CLOCK; break;
    case midi1::start: packet_.event.type = SNDRV_SEQ_EVENT_START; break;
    case midi1::resume: packet_.event.type = SNDRV_SEQ_EVENT_CONTINUE; break;
    case midi1::stop: packet_.event.type = SNDRV_SEQ_EVENT_STOP; break;
    case midi1::active_sensing: packet_.event.type = SNDRV_SEQ_EVENT_SENSING; break;
    case midi1::reset: packet_.event.type = SNDRV_SEQ_EVENT_RESET; break;
    default: throw Fault("Unsupported host MIDI realtime output=" + std::to_string(value));
  }
  ready_ = true;
}

void SequenceEncoder::fixed() {
  packet_ = {};
  const auto first = message_.front();
  const auto second = message_[1];
  const auto third = message_[2];
  const auto wide = static_cast<unsigned>(second) | (static_cast<unsigned>(third) << 7U);
  const auto family = first < system ? first & system : first;
  switch (family) {
    case midi1::note_off: packet_.event.type = SNDRV_SEQ_EVENT_NOTEOFF; break;
    case midi1::note_on: packet_.event.type = SNDRV_SEQ_EVENT_NOTEON; break;
    case midi1::key_pressure: packet_.event.type = SNDRV_SEQ_EVENT_KEYPRESS; break;
    case midi1::control_change: packet_.event.type = SNDRV_SEQ_EVENT_CONTROLLER; break;
    case midi1::program_change: packet_.event.type = SNDRV_SEQ_EVENT_PGMCHANGE; break;
    case midi1::channel_pressure: packet_.event.type = SNDRV_SEQ_EVENT_CHANPRESS; break;
    case midi1::pitch_bend: packet_.event.type = SNDRV_SEQ_EVENT_PITCHBEND; break;
    case midi1::quarter_frame: packet_.event.type = SNDRV_SEQ_EVENT_QFRAME; break;
    case midi1::song_position: packet_.event.type = SNDRV_SEQ_EVENT_SONGPOS; break;
    case midi1::song_select: packet_.event.type = SNDRV_SEQ_EVENT_SONGSEL; break;
    case midi1::tune_request: packet_.event.type = SNDRV_SEQ_EVENT_TUNE_REQUEST; break;
    default: throw Fault("Unsupported completed host MIDI output");
  }
  if (family >= midi1::note_off && family <= midi1::key_pressure) {
    packet_.event.data.note.channel = static_cast<std::uint8_t>(first & channel_mask);
    packet_.event.data.note.note = second;
    packet_.event.data.note.velocity = third;
  } else if (family != midi1::tune_request) {
    auto &control = packet_.event.data.control;
    control.channel = first < system ? static_cast<std::uint8_t>(first & channel_mask) : 0;
    control.param = family == midi1::control_change ? second : 0;
    control.value = second;
    if (family == midi1::control_change) { control.value = third; }
    if (family == midi1::song_position) { control.value = static_cast<int>(wide); }
    if (family == midi1::pitch_bend) {
      control.value = static_cast<int>(wide) - static_cast<int>(bend_bias);
    }
  }
  used_ = 0;
  expected_ = 0;
  ready_ = true;
}

void SequenceEncoder::status(const std::uint8_t value) {
  used_ = expected_ = 0;
  running_ = value < system ? value : 0;
  message_ = {value, 0, 0};
  used_ = 1;
  if (value == midi1::sysex_start || value == midi1::sysex_end) {
    used_ = 0;
    sysex_bytes_[0] = value;
    sysex_size_ = 1;
    sysex_ = value == midi1::sysex_start;
    if (!sysex_ || sysex_size_ == fragment_limit_) { fragment(); }
    return;
  }
  expected_ = message_length(value);
  if (expected_ == 1) { fixed(); }
}

bool SequenceEncoder::feed(const std::uint8_t value) {
  if (ready_) { throw Fault("Host MIDI packet must be delivered before accepting another byte"); }
  if (value >= realtime_start) {
    if (sysex_ && sysex_size_ != 0) {
      fragment();
      return false;
    }
    realtime(value);
    return true;
  }
  if (sysex_) {
    if (value >= status_bit && value != midi1::sysex_end) {
      throw Fault("Host MIDI SysEx output interrupted by a non-realtime status");
    }
    sysex_bytes_.at(sysex_size_++) = value;
    if (value == midi1::sysex_end) { sysex_ = false; }
    if (sysex_size_ == fragment_limit_ || !sysex_) { fragment(); }
    return true;
  }
  if (value >= status_bit) {
    status(value);
    return true;
  }
  if (used_ == 0) {
    if (running_ == 0) { throw Fault("Host MIDI output data has no running status"); }
    message_ = {running_, 0, 0};
    used_ = 1;
    expected_ = message_length(running_);
  }
  message_.at(used_++) = value;
  if (used_ == expected_) { fixed(); }
  return true;
}
}
