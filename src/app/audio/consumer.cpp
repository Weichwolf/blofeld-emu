// SPDX-License-Identifier: GPL-3.0-only
#include "audio/consumer.hpp"

#include "audio/playback.hpp"
#include "io/pcm_output.hpp"

#include <atomic>
#include <bit>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <stop_token>

namespace blofeld::app {
vi::PcmOutput AudioConsumer::output() noexcept {
  return {.owner = this,
          .write =
              [](void *owner, const vi::PcmOutput::Block block, const std::stop_token &stop) {
                return static_cast<AudioConsumer *>(owner)->write(block, stop);
              },
          .close =
              [](void *owner) noexcept { static_cast<AudioConsumer *>(owner)->producer_.reset(); }};
}

bool AudioConsumer::write(const vi::PcmOutput::Block block, const std::stop_token &stop) {
  if (block.frames.empty() || block.frames.size() > 16 ||
      block.ticks.size() != block.frames.size() || block.frame_ticks == 0 ||
      std::uint64_t{block.frame_ticks} * playback_.grants().rate != block.reference_hz) {
    throw std::runtime_error("Unsupported original PCM block format/rate");
  }
  if (!producer_) { producer_.emplace(playback_.producer()); }
  producer_->end_work();
  const auto arrived = std::chrono::steady_clock::now();
  if (epoch_ && *epoch_ != block.epoch) {
    if (block.epoch < *epoch_) { throw std::runtime_error("PCM reset epoch moved backwards"); }
    if (!playback_.reset(stop)) { return false; }
    next_frame_ = 0;
    last_tick_.reset();
    previous_.reset();
  }
  epoch_ = block.epoch;
  if (block.first_frame != next_frame_) {
    throw std::runtime_error("PCM frame sequence is discontinuous");
  }
  auto tick = last_tick_;
  for (const auto next : block.ticks) {
    if (tick && next <= *tick) { throw std::runtime_error("PCM completion time moved backwards"); }
    tick = next;
  }
  if (!playback_.write(block.frames, stop)) { return false; }
  last_tick_ = tick;
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
  next_frame_ += block.frames.size();
  accepted_.fetch_add(block.frames.size(), std::memory_order_relaxed);
  underruns_.fetch_add(static_cast<std::uint64_t>(std::popcount(block.underruns)),
                       std::memory_order_relaxed);
  return true;
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
