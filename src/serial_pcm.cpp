// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/serial_pcm.hpp"

#include "blofeld/bus.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <string_view>

namespace blofeld {
SerialPcmReceiver::SerialPcmReceiver(const SerialPcmFormat format) : format_(format) {
  constexpr unsigned maximum_bits = 32;
  if (format.slot_bits == 0 || format.slot_bits > maximum_bits || format.word_bits == 0 ||
      format.word_bits > format.slot_bits || format.channels < 2 ||
      format.channels > SerialPcmFrame::maximum_channels ||
      (format.sample_edge != SerialPcmEdge::rising &&
       format.sample_edge != SerialPcmEdge::falling)) {
    throw Fault("Invalid serial PCM format");
  }
  word_modulus_ = std::int64_t{1} << format.word_bits;
  word_sign_ = static_cast<std::uint32_t>(word_modulus_ / 2);
}

void SerialPcmReceiver::reset() {
  *this = SerialPcmReceiver(format_);
}

void SerialPcmReceiver::fail(const std::uint64_t ticks, const std::string_view reason) {
  faulted_ = true;
  throw Fault("Serial PCM input at ticks=" + std::to_string(ticks) +
              " frames=" + std::to_string(frames_) + " slot=" + std::to_string(slot_) +
              " bit=" + std::to_string(bit_) + ": " + std::string(reason));
}

std::int32_t SerialPcmReceiver::signed_word() const {
  const auto value = static_cast<std::int64_t>(word_);
  return static_cast<std::int32_t>((word_ & word_sign_) == 0 ? value : value - word_modulus_);
}

std::optional<SerialPcmFrame> SerialPcmReceiver::sample(const std::uint64_t ticks,
                                                        const bool active,
                                                        const std::optional<bool> data) {
  if (!locked_) {
    if (!active) { seen_inactive_ = true; }
    if (!active || !seen_inactive_) { return std::nullopt; }
    locked_ = true;
  }
  if (active != (slot_ == 0)) { fail(ticks, "Unexpected word-length frame sync"); }
  if (slot_ == 0 && bit_ == 0) { frame_.first_tick = ticks; }
  if (bit_ < format_.word_bits) {
    if (!data) { fail(ticks, "Unknown or undriven payload bit"); }
    word_ = (word_ << 1U) | static_cast<unsigned>(*data);
  }
  ++bit_;
  if (bit_ != format_.slot_bits) { return std::nullopt; }
  frame_.samples.at(slot_) = signed_word();
  word_ = 0;
  bit_ = 0;
  ++slot_;
  if (slot_ != format_.channels) { return std::nullopt; }
  if (frames_ == std::numeric_limits<std::uint64_t>::max()) {
    fail(ticks, "Frame count exhausted");
  }
  frame_.channels = format_.channels;
  frame_.last_tick = ticks;
  const auto result = frame_;
  frame_ = {};
  slot_ = 0;
  ++frames_;
  return result;
}

std::optional<SerialPcmFrame> SerialPcmReceiver::observe(const std::uint64_t ticks,
                                                         const SerialPcmPins pins) {
  if (faulted_) { fail(ticks, "Receiver is quarantined; restore a trusted copy or reset"); }
  if (previous_tick_ && ticks <= *previous_tick_) {
    fail(ticks, "Non-increasing observation time");
  }
  if (!pins.clock || !pins.frame_sync) { fail(ticks, "Unknown or undriven clock/frame sync"); }
  const bool active = *pins.frame_sync == format_.frame_sync_high;
  const bool sample_high = format_.sample_edge == SerialPcmEdge::rising;
  const bool edge =
      previous_clock_ && *previous_clock_ != *pins.clock && *pins.clock == sample_high;
  if (!previous_clock_) { seen_inactive_ = !active; }
  previous_clock_ = pins.clock;
  previous_tick_ = ticks;
  return edge ? sample(ticks, active, pins.data) : std::nullopt;
}
}
