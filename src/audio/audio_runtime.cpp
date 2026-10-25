// SPDX-License-Identifier: GPL-3.0-only
#include "audio/audio_runtime.hpp"

#include "audio/audio_worker.hpp"
#include "audio/serial_pcm.hpp"
#include "common/bus.hpp"

#include <atomic>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <stop_token>
#include <thread>
#include <utility>

namespace blofeld {
AudioRuntime::Worker::Worker(AudioRuntimeOptions options)
    : config(std::move(options.output)), queue(options.queue_frames) {
  static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
  static_assert(std::atomic<bool>::is_always_lock_free);
  static_assert(std::atomic<AudioRuntimeState>::is_always_lock_free);
  thread = std::jthread([this](const std::stop_token &stop) { run(stop); });
  while (state.load(std::memory_order_acquire) == AudioRuntimeState::connecting) {
    state.wait(AudioRuntimeState::connecting, std::memory_order_acquire);
  }
  check();
}

AudioRuntime::Worker::~Worker() {
  thread.request_stop();
}

void AudioRuntime::Worker::publish(const AudioRuntimeState value) noexcept {
  state.store(value, std::memory_order_release);
  state.notify_all();
}

void AudioRuntime::Worker::check() const {
  const auto current = state.load(std::memory_order_acquire);
  if (current == AudioRuntimeState::failed) { std::rethrow_exception(failure); }
  if (retired.load(std::memory_order_acquire) || current == AudioRuntimeState::retired) {
    throw Fault("Audio output retired; bind a new session after restore or fault");
  }
  if (current == AudioRuntimeState::cancelled) { throw Fault("Audio output cancelled"); }
}

PcmSubmission AudioRuntime::Worker::submit(const SerialPcmFrame &frame) noexcept {
  if (retired.load(std::memory_order_acquire)) { return PcmSubmission::retired; }
  const auto current = state.load(std::memory_order_acquire);
  if (current == AudioRuntimeState::failed) { return PcmSubmission::failed; }
  if (closed.load(std::memory_order_acquire) || current == AudioRuntimeState::cancelled ||
      current == AudioRuntimeState::finished) {
    return PcmSubmission::closed;
  }
  const auto total = accepted.load(std::memory_order_relaxed);
  if (total == std::numeric_limits<std::uint64_t>::max()) { return PcmSubmission::exhausted; }
  if (!queue.try_push(frame)) { return PcmSubmission::full; }
  accepted.store(total + 1, std::memory_order_release);
  return PcmSubmission::accepted;
}

AudioRuntime::AudioRuntime(AudioRuntimeOptions options)
    : worker_(std::make_unique<Worker>(std::move(options))) {}

AudioRuntime::~AudioRuntime() = default;

PcmSubmission AudioRuntime::try_submit(const SerialPcmFrame &frame) noexcept {
  return worker_->submit(frame);
}

void AudioRuntime::accept(const SerialPcmFrame &frame) {
  switch (try_submit(frame)) {
    case PcmSubmission::accepted: return;
    case PcmSubmission::full: throw Fault("Audio handoff full; refusing to drop a PCM frame");
    case PcmSubmission::closed: throw Fault("Audio output is closed to new PCM frames");
    case PcmSubmission::retired: throw Fault("Audio output retired; bind a new session");
    case PcmSubmission::exhausted: throw Fault("Audio output frame counter exhausted");
    case PcmSubmission::failed: check(); break;
    default: throw Fault("Unknown audio submission result");
  }
  throw Fault("Audio worker failure lacks its recorded exception");
}

void AudioRuntime::check() const {
  worker_->check();
  if (worker_->closed.load(std::memory_order_acquire)) {
    throw Fault("Audio output closed; bind a new session");
  }
}

void AudioRuntime::invalidate() noexcept {
  worker_->retired.store(true, std::memory_order_release);
}

void AudioRuntime::finish() {
  worker_->closed.store(true, std::memory_order_release);
  if (worker_->thread.joinable()) { worker_->thread.join(); }
  worker_->check();
  if (worker_->state.load(std::memory_order_acquire) != AudioRuntimeState::finished) {
    throw Fault("Audio output did not complete its accepted PCM prefix");
  }
}

AudioRuntimeStatus AudioRuntime::status() const noexcept {
  return {.state = worker_->state.load(std::memory_order_acquire),
          .accepted = worker_->accepted.load(std::memory_order_acquire),
          .written = worker_->written.load(std::memory_order_relaxed),
          .underflows = worker_->underflows.load(std::memory_order_relaxed)};
}
}
