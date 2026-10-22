// SPDX-License-Identifier: GPL-3.0-only
#include "audio_worker.hpp"

#include "blofeld/audio_runtime.hpp"
#include "blofeld/bus.hpp"
#include "blofeld/host_audio.hpp"

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <pthread.h>
#include <span>
#include <stop_token>
#include <string>
#include <thread>

namespace blofeld {
void AudioRuntime::Worker::observe(const HostAudioStatus &value) noexcept {
  written.store(value.frames_written, std::memory_order_relaxed);
  underflows.store(value.underflows, std::memory_order_relaxed);
}

void AudioRuntime::Worker::check_underflow(const HostAudioStatus &value) {
  observe(value);
  if (value.underflows == 0) { return; }
  constexpr std::uint64_t host_frame_bytes = 8;
  constexpr auto maximum =
      static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) / host_frame_bytes;
  const auto closing = closed.load(std::memory_order_acquire);
  const auto total = accepted.load(std::memory_order_acquire);
  if (closing && total <= maximum &&
      value.earliest_underflow_byte >= static_cast<std::int64_t>(total * host_frame_bytes)) {
    return; // A known event at/after the complete source EOF is not a lost prefix.
  }
  throw Fault("Audio output underrun byte=" + std::to_string(value.earliest_underflow_byte) +
              " before complete accepted prefix; source-frames=" + std::to_string(total));
}

void AudioRuntime::Worker::refill() {
  if (cursor != count) { return; }
  cursor = 0;
  count = 0;
  while (count < batch.size() && queue.try_pop(batch[count])) { ++count; }
}

void AudioRuntime::Worker::check_progress(const std::size_t admitted) {
  if (admitted != 0 || cursor == count) {
    stalled_since.reset();
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  if (!stalled_since) { stalled_since = now; }
  if (now - *stalled_since >= config.timeout) {
    throw Fault("Audio output write progress deadline expired");
  }
}

void AudioRuntime::Worker::start_if_ready(HostAudioOutput &output,
                                          const bool blocked,
                                          const bool closing) {
  const auto status = output.status();
  if (status.started || status.frames_written == 0) { return; }
  if (status.frames_written >= config.buffer_frames || blocked || (closing && cursor == count)) {
    output.start();
    publish(AudioRuntimeState::playing);
  }
}

bool AudioRuntime::Worker::finish_if_ready(HostAudioOutput &output, const bool closing) {
  if (!closing || cursor != count) { return false; }
  // Closure was acquired before this read. A concurrently published final frame
  // cannot be missed, and a completed batch is not proof of an empty queue.
  if (queue.try_pop(batch[0])) {
    cursor = 0;
    count = 1;
    return false;
  }
  if (output.status().frames_written == 0) {
    throw Fault("Audio output finished without a complete PCM frame");
  }
  output.drain();
  check_underflow(output.status());
  publish(AudioRuntimeState::finished);
  return true;
}

void AudioRuntime::Worker::transfer(HostAudioOutput &output, const std::stop_token &stop) {
  while (!stop.stop_requested() && !retired.load(std::memory_order_acquire)) {
    const auto closing = closed.load(std::memory_order_acquire);
    refill();
    if (retired.load(std::memory_order_acquire)) { break; }
    std::size_t admitted = 0;
    if (count != cursor) {
      admitted = output.write(std::span(batch).first(count).subspan(cursor));
      cursor += admitted;
    }
    check_progress(admitted);
    observe(output.status());
    start_if_ready(output, count != cursor && admitted == 0, closing);
    check_underflow(output.status());
    if (finish_if_ready(output, closing)) { return; }
    output.poll();
    check_underflow(output.status());
    std::this_thread::sleep_for(std::chrono::milliseconds{1});
  }
  publish(retired.load(std::memory_order_acquire) ? AudioRuntimeState::retired
                                                  : AudioRuntimeState::cancelled);
}

void AudioRuntime::Worker::run(const std::stop_token &stop) noexcept {
  try {
    const auto named = ::pthread_setname_np(::pthread_self(), "blofeld-audio");
    if (named != 0) { throw Fault("Cannot name audio worker thread: " + std::to_string(named)); }
    HostAudioOutput output(config);
    publish(AudioRuntimeState::ready);
    transfer(output, stop);
  } catch (...) {
    failure = std::current_exception();
    publish(AudioRuntimeState::failed);
  }
}
}
