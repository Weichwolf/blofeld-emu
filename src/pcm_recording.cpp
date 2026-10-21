// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/pcm_recording.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/serial_pcm.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace blofeld {
PcmRecording::PcmRecording(const SerialPcmFormat format, const std::size_t capacity)
    : format_(format),
      capacity_(capacity),
      frames_(std::make_shared<std::vector<SerialPcmFrame>>()) {
  const SerialPcmReceiver validated(format);
  if (capacity == 0 || capacity > maximum_capacity) {
    throw Fault("Invalid PCM recording capacity");
  }
  constexpr unsigned maximum_bits = 32;
  if (format.word_bits == 0 || format.word_bits > maximum_bits) {
    throw Fault("Invalid PCM precision");
  }
  limit_ = std::int64_t{1} << (format.word_bits - 1);
}

void PcmRecording::append(const SerialPcmFrame &frame) {
  if (!frames_) { throw Fault("PCM recording is disabled"); }
  if (frames_->size() == capacity_) { throw Fault("PCM recording frame budget exhausted"); }
  if (frame.channels != format_.channels || frame.last_tick < frame.first_tick ||
      (frame.last_tick == frame.first_tick && frame.last_order <= frame.first_order)) {
    throw Fault("Invalid PCM recording frame shape/time");
  }
  for (unsigned channel = 0; channel < frame.channels; ++channel) {
    const auto sample = frame.samples.at(channel);
    if (sample < -limit_ || sample >= limit_) {
      throw Fault("PCM sample exceeds declared precision");
    }
  }
  if (!frames_->empty()) {
    const auto &last = frames_->back();
    if (frame.first_tick < last.last_tick ||
        (frame.first_tick == last.last_tick && frame.first_order <= last.last_order)) {
      throw Fault("PCM recording frame overlaps its completed prefix");
    }
  }
  if (frames_.unique()) {
    frames_->push_back(frame);
    return;
  }
  auto next = std::make_shared<std::vector<SerialPcmFrame>>(*frames_);
  next->push_back(frame);
  frames_ = std::move(next);
}

bool PcmRecording::operator==(const PcmRecording &other) const {
  return format_ == other.format_ && capacity_ == other.capacity_ &&
         (frames_ == other.frames_ || (frames_ && other.frames_ && *frames_ == *other.frames_));
}
}
