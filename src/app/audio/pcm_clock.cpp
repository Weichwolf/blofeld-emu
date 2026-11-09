// SPDX-License-Identifier: GPL-3.0-only
#include "audio/pcm_clock.hpp"

#include "audio/render_position.hpp"
#include "io/pcm_output.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace blofeld::app {
static_assert(std::atomic<std::uint64_t>::is_always_lock_free);

PcmClock::PcmClock(const std::size_t history_frames) : ticks_(history_frames) {
  static_assert(std::atomic<Phase>::is_always_lock_free);
}

void PcmClock::fail(const char *message) {
  phase_.store(Phase::failed);
  throw std::runtime_error(message);
}

void PcmClock::append(const vi::PcmOutput::Block block, const std::uint64_t audio_epoch) {
  const auto phase = phase_.load();
  if (phase == Phase::failed) { fail("PCM clock requires reset after failure"); }
  if (ticks_.empty()) { return; }
  const auto head = next_frame_.load();
  const auto origin = origin_.load();
  constexpr std::size_t maximum_block = 16;
  if (block.ticks.empty() || block.ticks.size() != block.frames.size() ||
      block.ticks.size() > maximum_block || block.first_frame != head ||
      block.ticks.size() > std::numeric_limits<std::uint64_t>::max() - head ||
      (phase != Phase::empty &&
       (epoch_.load() != block.epoch || audio_epoch_.load() != audio_epoch))) {
    fail("PCM clock source sequence or epoch is discontinuous");
  }
  auto previous = head == origin
                      ? std::optional<std::uint64_t>{}
                      : std::optional<std::uint64_t>{ticks_[(head - 1) % ticks_.size()].load()};
  for (const auto tick : block.ticks) {
    if (previous && tick <= *previous) { fail("PCM clock completion time moved backwards"); }
    previous = tick;
  }
  const auto version = source_version_.load();
  if (version > std::numeric_limits<std::uint64_t>::max() - 2) {
    fail("PCM source publication exhausted");
  }
  source_version_.store(version + 1);
  epoch_.store(block.epoch);
  audio_epoch_.store(audio_epoch);
  for (std::size_t index = 0; index < block.ticks.size(); ++index) {
    ticks_[static_cast<std::size_t>((head + index) % ticks_.size())].store(block.ticks[index]);
  }
  next_frame_.store(head + block.ticks.size());
  source_version_.store(version + 2);
  if (phase == Phase::empty) { phase_.store(Phase::buffering); }
}

void PcmClock::observe(const host::audio::RenderPosition position) {
  const auto point = read(position);
  if (!point) { fail("Native audio position has no retained original PCM interval"); }
  if (previous_) {
    if (position == *previous_) { return; }
    if (position.first_frame <= previous_->first_frame || position.host_ns <= previous_->host_ns ||
        position.first_frame < previous_->first_frame + previous_->frames) {
      fail("Native audio position moved backwards or overlapped");
    }
  }
  const auto version = version_.load();
  if (version > std::numeric_limits<std::uint64_t>::max() - 2) {
    fail("PCM clock publication exhausted");
  }
  // One SC order encloses payload access; clear() revokes without recycling a version.
  version_.store(version + 1);
  published_epoch_.store(point->epoch);
  published_audio_epoch_.store(point->audio_epoch);
  published_ticks_.store(point->ticks);
  published_host_ns_.store(point->host_ns);
  version_.store(version + 2);
  phase_.store(Phase::published);
  previous_ = position;
}

void PcmClock::clear(const std::uint64_t source_origin) noexcept {
  phase_.store(Phase::empty);
  previous_.reset();
  const auto version = source_version_.load();
  if (version > std::numeric_limits<std::uint64_t>::max() - 2) {
    phase_.store(Phase::failed);
    return;
  }
  source_version_.store(version + 1);
  origin_.store(source_origin);
  next_frame_.store(source_origin);
  source_version_.store(version + 2);
}

std::optional<PcmClock::Point>
PcmClock::read(const host::audio::RenderPosition position) const noexcept {
  constexpr unsigned attempts = 3;
  for (unsigned attempt = 0; attempt < attempts; ++attempt) {
    const auto phase = phase_.load();
    if ((phase != Phase::buffering && phase != Phase::published) || ticks_.empty()) { return {}; }
    const auto version = source_version_.load();
    if ((version & 1U) != 0) { continue; }
    const auto origin = origin_.load();
    const auto head = next_frame_.load();
    const auto audio_epoch = audio_epoch_.load();
    if (head < origin || position.epoch != audio_epoch || position.frames == 0 ||
        position.host_ns == 0 || position.first_frame >= head - origin ||
        position.frames > head - origin - position.first_frame) {
      return {};
    }
    const auto first = origin + position.first_frame;
    if (head - first > ticks_.size()) { return {}; }
    const Point point{.epoch = epoch_.load(),
                      .audio_epoch = audio_epoch,
                      .ticks = ticks_[static_cast<std::size_t>(first % ticks_.size())].load(),
                      .host_ns = position.host_ns};
    if (version == source_version_.load()) {
      const auto current = phase_.load();
      if (current == Phase::buffering || current == Phase::published) { return point; }
    }
  }
  return {};
}

std::optional<PcmClock::Point> PcmClock::read() const noexcept {
  constexpr unsigned attempts = 3;
  for (unsigned attempt = 0; attempt < attempts; ++attempt) {
    if (phase_.load() != Phase::published) { return {}; }
    const auto version = version_.load();
    if ((version & 1U) != 0) { continue; }
    const Point point{.epoch = published_epoch_.load(),
                      .audio_epoch = published_audio_epoch_.load(),
                      .ticks = published_ticks_.load(),
                      .host_ns = published_host_ns_.load()};
    if (version_.load() == version && phase_.load() == Phase::published) { return point; }
  }
  return {};
}
}
