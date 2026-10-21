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

void SerialPcmReceiver::fail(const SerialPcmStamp stamp, const std::string_view reason) {
  faulted_ = true;
  throw Fault("Serial PCM input at ticks=" + std::to_string(stamp.ticks) +
              " order=" + std::to_string(stamp.order) + " frames=" + std::to_string(frames_) +
              " slot=" + std::to_string(slot_) + " bit=" + std::to_string(bit_) + ": " +
              std::string(reason));
}

std::int32_t SerialPcmReceiver::signed_word() const {
  const auto value = static_cast<std::int64_t>(word_);
  return static_cast<std::int32_t>((word_ & word_sign_) == 0 ? value : value - word_modulus_);
}

std::optional<SerialPcmFrame> SerialPcmReceiver::sample(const SerialPcmStamp stamp,
                                                        const bool active,
                                                        const std::optional<bool> data) {
  if (!locked_) {
    if (!active) { seen_inactive_ = true; }
    if (!active || !seen_inactive_) { return std::nullopt; }
    locked_ = true;
  }
  if (active != (slot_ == 0)) { fail(stamp, "Unexpected word-length frame sync"); }
  if (slot_ == 0 && bit_ == 0) {
    frame_.first_tick = stamp.ticks;
    frame_.first_order = stamp.order;
  }
  if (bit_ < format_.word_bits) {
    if (!data) { fail(stamp, "Unknown or undriven payload bit"); }
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
    fail(stamp, "Frame count exhausted");
  }
  frame_.channels = format_.channels;
  frame_.last_tick = stamp.ticks;
  frame_.last_order = stamp.order;
  const auto result = frame_;
  frame_ = {};
  slot_ = 0;
  ++frames_;
  return result;
}

std::optional<SerialPcmFrame> SerialPcmReceiver::observe(const std::uint64_t ticks,
                                                         const SerialPcmPins pins) {
  return observe_ordered({.ticks = ticks}, pins);
}

std::optional<SerialPcmFrame> SerialPcmReceiver::observe_ordered(const SerialPcmStamp stamp,
                                                                 const SerialPcmPins pins) {
  if (faulted_) { fail(stamp, "Receiver is quarantined; restore a trusted copy or reset"); }
  if (previous_stamp_ &&
      (stamp.ticks < previous_stamp_->ticks ||
       (stamp.ticks == previous_stamp_->ticks && stamp.order <= previous_stamp_->order))) {
    fail(stamp, "Non-increasing observation time/order");
  }
  if (!pins.clock || !pins.frame_sync) { fail(stamp, "Unknown or undriven clock/frame sync"); }
  const bool active = *pins.frame_sync == format_.frame_sync_high;
  const bool sample_high = format_.sample_edge == SerialPcmEdge::rising;
  const bool edge =
      previous_clock_ && *previous_clock_ != *pins.clock && *pins.clock == sample_high;
  if (!previous_clock_) { seen_inactive_ = !active; }
  previous_clock_ = pins.clock;
  previous_stamp_ = stamp;
  return edge ? sample(stamp, active, pins.data) : std::nullopt;
}
}
