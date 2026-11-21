// SPDX-License-Identifier: GPL-3.0-only
#include "midi/live_clock.hpp"

#include "audio/pcm_clock.hpp"
#include "io/midi_port.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

namespace blofeld::app {
namespace {
constexpr std::uint64_t ns_per_second = 1000000000;
constexpr std::uint64_t delivery_ns = 1000000;
constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
}

LiveClock::LiveClock(const Grant grant) : uncertainty_(grant.uncertainty_ns) {
  const auto [rate, queue_frames, uncertainty_ns] = grant;
  if (rate == 0 || queue_frames == 0 || queue_frames > rate) {
    throw std::invalid_argument("Live MIDI requires bounded actual audio queue grants");
  }
  // Cover a full queue, one consumed interval and a pending original PCM block.
  constexpr std::uint64_t block_frames = 16;
  const auto frames = (std::uint64_t{queue_frames} * 2) + block_frames;
  const auto duration = (frames * ns_per_second + rate - 1) / rate;
  if (uncertainty_ns > (maximum_gap_ns - delivery_ns) / 2 ||
      duration > maximum_gap_ns - delivery_ns - (2 * uncertainty_ns)) {
    throw std::runtime_error("Chosen live MIDI admission exceeds 10 ms: queue-frames=" +
                             std::to_string(queue_frames) + " rate=" + std::to_string(rate));
  }
  prime_ = (frames * vi::MidiPort::reference_hz + rate - 1) / rate;
}

const PcmClock::Point &LiveClock::point(const std::size_t index) const noexcept {
  return points_[(head_ + index) % capacity];
}

void LiveClock::observe(const PcmClock::Point next) {
  if (next.host_ns == 0) { throw std::runtime_error("Live audio host timestamp is missing"); }
  if (next.uncertainty_ns > uncertainty_ || next.uncertainty_ns >= next.host_ns ||
      next.uncertainty_ns > maximum - next.host_ns) {
    throw std::runtime_error("Native audio correlation exceeds the live uncertainty bound");
  }
  if (size_ != 0) {
    const auto &previous = point(size_ - 1);
    if (next == previous) { return; }
    const bool reset = next.epoch != previous.epoch || next.audio_epoch != previous.audio_epoch;
    if (reset && (next.epoch <= previous.epoch || next.audio_epoch <= previous.audio_epoch)) {
      throw std::runtime_error("Live audio and DSP reset epochs do not advance together");
    }
    if (next.host_ns <= previous.host_ns || next.ticks <= previous.ticks) {
      throw std::runtime_error("Live audio correlation moved backwards");
    }
    if (reset || next.host_ns - previous.host_ns > maximum_gap_ns) {
      // Correlation starts again after a reset or a gap in native observations.
      head_ = 0;
      size_ = 0;
    }
  }
  if (size_ == capacity) {
    head_ = (head_ + 1) % capacity;
    --size_;
  }
  points_[(head_ + size_) % capacity] = next;
  ++size_;
}

std::optional<std::uint64_t> LiveClock::correlate(const std::uint64_t host_ns,
                                                  const std::uint64_t audio_epoch) const {
  if (size_ == 0 || point(0).audio_epoch != audio_epoch || host_ns < point(0).host_ns) {
    return {};
  }
  for (std::size_t index = 0; index < size_; ++index) {
    const auto &right = point(index);
    if (host_ns == right.host_ns) { return right.ticks; }
    if (host_ns > right.host_ns) { continue; }
    const auto &left = point(index - 1);
    const auto offset = host_ns - left.host_ns;
    const auto ticks = right.ticks - left.ticks;
    const auto span = right.host_ns - left.host_ns;
    if (offset > maximum / ticks) {
      throw std::overflow_error("Live audio interpolation is not representable");
    }
    const auto product = offset * ticks;
    const auto delta = (product / span) + (product % span != 0 ? 1U : 0U);
    return left.ticks + delta;
  }
  return {};
}

}
