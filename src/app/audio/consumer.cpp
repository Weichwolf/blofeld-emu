// SPDX-License-Identifier: GPL-3.0-only
#include "audio/consumer.hpp"

#include "audio/pcm_clock.hpp"
#include "audio/playback.hpp"
#include "io/pcm_output.hpp"

#include <atomic>
#include <bit>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <stop_token>

namespace blofeld::app {
AudioConsumer::AudioConsumer(host::audio::Playback &playback, const bool deferred)
    : playback_(playback),
      clock_(playback.grants().position_history_frames.value_or(0)),
      phase_(deferred ? Phase::waiting : Phase::writing) {}

void AudioConsumer::begin_at(const std::uint64_t ticks) {
  auto expected = std::numeric_limits<std::uint64_t>::max();
  if (phase_.load(std::memory_order_acquire) != Phase::waiting || !has_pcm() ||
      ticks <= observed_.load(std::memory_order_acquire) || ticks == expected ||
      !begin_.compare_exchange_strong(expected, ticks, std::memory_order_release)) {
    throw std::logic_error("Live PCM origin is retroactive, unavailable or already selected");
  }
}

bool AudioConsumer::has_pcm() const noexcept {
  return pcm_.load(std::memory_order_acquire);
}

vi::PcmOutput AudioConsumer::output() noexcept {
  return {.owner = this,
          .write =
              [](void *owner, const vi::PcmOutput::Block block, const std::stop_token &stop) {
                return static_cast<AudioConsumer *>(owner)->write(block, stop);
              },
          .close =
              [](void *owner) noexcept {
                auto &consumer = *static_cast<AudioConsumer *>(owner);
                consumer.clock_.clear();
                consumer.producer_.reset();
                consumer.pcm_.store(false, std::memory_order_release);
                if (consumer.phase_.load() != Phase::failed) {
                  consumer.phase_.store(Phase::closed, std::memory_order_release);
                }
              }};
}

bool AudioConsumer::write(const vi::PcmOutput::Block block, const std::stop_token &stop) {
  try {
    if (producer_) { producer_->end_work(); }
    if (!stop.stop_requested() && submit(block, stop)) { return true; }
  } catch (...) {
    clock_.clear();
    pcm_.store(false, std::memory_order_release);
    phase_.store(Phase::failed, std::memory_order_release);
    throw;
  }
  clock_.clear();
  pcm_.store(false, std::memory_order_release);
  phase_.store(Phase::failed, std::memory_order_release);
  return false;
}

bool AudioConsumer::source(const vi::PcmOutput::Block block, const std::stop_token &stop) {
  if (block.frames.empty() || block.frames.size() > 16 ||
      block.ticks.size() != block.frames.size() || block.frame_ticks == 0 ||
      std::uint64_t{block.frame_ticks} * playback_.grants().rate != block.reference_hz) {
    throw std::runtime_error("Unsupported original PCM block format/rate");
  }
  if ((std::uint32_t{block.underruns} >> block.frames.size()) != 0) {
    throw std::runtime_error("PCM underrun mask exceeds the original frame block");
  }
  if (epoch_ && *epoch_ != block.epoch) {
    if (block.epoch < *epoch_) { throw std::runtime_error("PCM reset epoch moved backwards"); }
    clock_.clear();
    if (producer_) {
      if (audio_epoch_ == std::numeric_limits<std::uint64_t>::max()) {
        throw std::overflow_error("PCM host audio epoch exhausted");
      }
      if (!playback_.reset(stop)) { return false; }
      ++audio_epoch_;
    }
    next_frame_ = 0;
    last_tick_.reset();
    previous_.reset();
  }
  epoch_ = block.epoch;
  if (block.first_frame != next_frame_ ||
      block.frames.size() > std::numeric_limits<std::uint64_t>::max() - next_frame_) {
    throw std::runtime_error("PCM frame sequence is discontinuous");
  }
  auto tick = last_tick_;
  for (const auto next : block.ticks) {
    if (tick && next <= *tick) { throw std::runtime_error("PCM completion time moved backwards"); }
    tick = next;
  }
  last_tick_ = tick;
  next_frame_ += block.frames.size();
  observed_.store(block.ticks.back(), std::memory_order_release);
  pcm_.store(true, std::memory_order_release);
  return true;
}

bool AudioConsumer::submit(vi::PcmOutput::Block block, const std::stop_token &stop) {
  const auto phase = phase_.load(std::memory_order_acquire);
  if (phase != Phase::waiting && phase != Phase::writing) {
    throw std::logic_error("Live PCM owner is closed or failed");
  }
  if (!source(block, stop)) { return false; }
  if (phase == Phase::waiting) {
    const auto begin = begin_.load(std::memory_order_acquire);
    if (begin == std::numeric_limits<std::uint64_t>::max()) { return true; }
    std::size_t first = 0;
    while (first < block.ticks.size() && block.ticks[first] < begin) { ++first; }
    if (first == block.ticks.size()) { return true; }
    block.frames = block.frames.subspan(first);
    block.ticks = block.ticks.subspan(first);
    block.first_frame += first;
    block.underruns = static_cast<std::uint16_t>(block.underruns >> first);
    clock_.clear(block.first_frame);
    phase_.store(Phase::writing, std::memory_order_release);
  }
  if (!producer_) {
    producer_.emplace(playback_.producer());
    producer_->end_work();
  }
  const auto arrived = std::chrono::steady_clock::now();
  if (!playback_.write(block.frames, stop)) {
    clock_.clear();
    return false;
  }
  if (playback_.grants().position_history_frames) {
    clock_.append(block, audio_epoch_);
    if (const auto position = playback_.status().position) { clock_.observe(*position); }
  }
  producer_->begin_work();
  const auto completed = std::chrono::steady_clock::now();
  if (previous_) {
    const auto elapsed =
        std::chrono::duration_cast<std::chrono::nanoseconds>(arrived - *previous_).count();
    active_ns_ += static_cast<std::uint64_t>(elapsed);
    timed_frames_ += block.frames.size();
    measured_ns_.store(active_ns_, std::memory_order_relaxed);
    measured_frames_.store(timed_frames_, std::memory_order_relaxed);
  }
  previous_ = completed;
  accepted_.fetch_add(block.frames.size(), std::memory_order_relaxed);
  underruns_.fetch_add(static_cast<std::uint64_t>(std::popcount(block.underruns)),
                       std::memory_order_relaxed);
  return true;
}

std::optional<PcmClock::Point> AudioConsumer::clock() const noexcept {
  if (!playback_.grants().position_history_frames) { return {}; }
  const auto native = playback_.status().position;
  if (!native) { return {}; }
  return clock_.read(*native);
}

PcmStatistics AudioConsumer::statistics() const noexcept {
  const auto elapsed = measured_ns_.load(std::memory_order_relaxed);
  const auto frames = measured_frames_.load(std::memory_order_relaxed);
  PcmStatistics result{.accepted_frames = accepted_.load(std::memory_order_relaxed),
                       .device_underruns = underruns_.load(std::memory_order_relaxed),
                       .production_reserve = {}};
  constexpr double ns_per_second = 1000000000.0;
  if (elapsed != 0 && frames != 0) {
    result.production_reserve =
        static_cast<double>(frames) * ns_per_second /
        (static_cast<double>(playback_.grants().rate) * static_cast<double>(elapsed));
  }
  return result;
}
}
