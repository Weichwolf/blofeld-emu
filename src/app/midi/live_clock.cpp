// SPDX-License-Identifier: GPL-3.0-only
#include "midi/live_clock.hpp"

#include "audio/pcm_clock.hpp"
#include "io/midi_port.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

namespace blofeld::app {
namespace {
constexpr std::uint64_t ns_per_second = 1000000000;
constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();

std::uint64_t add(const std::uint64_t a, const std::uint64_t b) {
  if (b > maximum - a) { throw std::overflow_error("Live MIDI timeline exhausted"); }
  return a + b;
}
}

LiveClock::LiveClock(const unsigned rate, const std::size_t queue_frames) {
  if (rate == 0 || queue_frames == 0 || queue_frames > rate) {
    throw std::invalid_argument("Live MIDI requires bounded actual audio queue grants");
  }
  // Cover a full queue, one consumed interval and a pending original PCM block.
  constexpr std::uint64_t block_frames = 16;
  const auto frames = (std::uint64_t{queue_frames} * 2) + block_frames;
  const auto duration = (frames * ns_per_second + rate - 1) / rate;
  if (duration > maximum_gap_ns - delivery_ns) {
    throw std::runtime_error("Chosen live MIDI admission exceeds 10 ms: queue-frames=" +
                             std::to_string(queue_frames) + " rate=" + std::to_string(rate));
  }
  prime_ = (frames * vi::MidiPort::reference_hz + rate - 1) / rate;
  lead_ = prime_ + delivery_ns * vi::MidiPort::reference_hz / ns_per_second;
}

const PcmClock::Point &LiveClock::point(const std::size_t index) const noexcept {
  return points_[(head_ + index) % capacity];
}

void LiveClock::observe(const PcmClock::Point next) {
  if (next.host_ns == 0) { throw std::runtime_error("Live audio host timestamp is missing"); }
  if (next.uncertainty_ns > maximum_uncertainty_ns || next.uncertainty_ns >= next.host_ns ||
      next.uncertainty_ns > maximum - next.host_ns) {
    throw std::runtime_error("Native audio correlation exceeds the live uncertainty bound");
  }
  if (size_ != 0) {
    if (next.uncertainty_ns > uncertainty_) {
      throw std::runtime_error("Native audio uncertainty requires a new readiness handshake");
    }
    const auto &previous = point(size_ - 1);
    if (next == previous) { return; }
    if (next.epoch != previous.epoch || next.audio_epoch != previous.audio_epoch) {
      throw std::runtime_error("Live audio epoch changed; a new readiness handshake is required");
    }
    if (next.host_ns <= previous.host_ns || next.ticks <= previous.ticks ||
        next.host_ns - previous.host_ns > maximum_gap_ns) {
      throw std::runtime_error("Live audio correlation moved backwards or missed its deadline");
    }
  } else if (next.uncertainty_ns != 0) {
    constexpr auto reserve = maximum_uncertainty_ns * vi::MidiPort::reference_hz / ns_per_second;
    constexpr auto bound = maximum_gap_ns * vi::MidiPort::reference_hz / ns_per_second;
    if (lead_ > bound - (2 * reserve)) {
      throw std::runtime_error("Native audio uncertainty exceeds the live admission window");
    }
    uncertainty_ = maximum_uncertainty_ns;
    lead_ += reserve;
  }
  if (size_ == capacity) {
    head_ = (head_ + 1) % capacity;
    --size_;
  }
  points_[(head_ + size_) % capacity] = next;
  ++size_;
}

std::optional<std::uint64_t> LiveClock::at(const std::uint64_t timestamp) const {
  if (size_ == 0 || timestamp < point(0).host_ns) { return {}; }
  for (std::size_t index = 0; index < size_; ++index) {
    const auto &right = point(index);
    if (timestamp == right.host_ns) { return add(right.ticks, lead_); }
    if (timestamp > right.host_ns) { continue; }
    const auto &left = point(index - 1);
    const auto offset = timestamp - left.host_ns;
    const auto ticks = right.ticks - left.ticks;
    const auto span = right.host_ns - left.host_ns;
    if (offset > maximum / ticks) {
      throw std::overflow_error("Live audio interpolation is not representable");
    }
    const auto product = offset * ticks;
    const auto delta = (product / span) + (product % span != 0 ? 1U : 0U);
    return add(add(left.ticks, delta), lead_);
  }
  return {};
}

std::optional<LiveClock::Window> LiveClock::window(const std::uint64_t now) {
  if (previous_poll_ && now < *previous_poll_) {
    throw std::runtime_error("Live input host clock moved backwards");
  }
  previous_poll_ = now;
  if (size_ == 0) { return {}; }
  const auto last = point(size_ - 1).host_ns;
  if (now > last && now - last > maximum_gap_ns) {
    throw std::runtime_error("Live audio correlation expired");
  }
  const auto delay = delivery_ns + uncertainty_;
  if (now < delay) { return {}; }
  const auto cutoff = std::min(now - delay, last);
  const auto ticks = at(cutoff);
  if (!ticks) { return {}; }
  return Window{.host_ns = cutoff, .ticks = *ticks};
}

std::uint64_t LiveClock::event(const std::uint64_t timestamp) {
  if ((committed_ && timestamp < *committed_) ||
      (previous_event_ && timestamp < *previous_event_)) {
    throw std::runtime_error("Native MIDI event missed committed time or moved backwards");
  }
  const auto ticks = at(timestamp);
  if (!ticks) { throw std::runtime_error("Native MIDI event has no retained audio interval"); }
  previous_event_ = timestamp;
  return *ticks;
}

void LiveClock::commit(const Window window) {
  const auto ticks = at(window.host_ns);
  if (!ticks || *ticks != window.ticks || (committed_ && window.host_ns < *committed_)) {
    throw std::logic_error("Live MIDI admission is not backed by a retained native interval");
  }
  committed_ = window.host_ns;
}
}
