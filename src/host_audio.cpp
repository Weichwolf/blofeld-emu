// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/host_audio.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/serial_pcm.hpp"
#include "host_audio_internal.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <pulse/context.h>
#include <pulse/error.h>
#include <pulse/mainloop.h>
#include <pulse/sample.h>
#include <pulse/stream.h>
#include <span>
#include <string>
#include <string_view>

namespace blofeld {
namespace {
constexpr unsigned channels = 2;
constexpr unsigned precision = SerialPcmFormat::dsp_word_bits;
constexpr std::int32_t minimum_sample = -8388608;
constexpr std::int32_t maximum_sample = 8388607;
}

void HostAudio::Stream::require_ready() const {
  if (failed || retired) {
    throw Fault("Host audio stream is faulted/retired; create a new transport");
  }
}

[[noreturn]] void HostAudio::Stream::fail(const std::string_view operation, const int error) {
  failed = true;
  close();
  throw Fault("Host audio " + std::string(operation) + ": " + pa_strerror(error));
}

void HostAudio::Stream::validate(const std::span<const SerialPcmFrame> frames) const {
  if (frames.empty() || frames.size() > maximum_block_frames ||
      frames.size() > std::numeric_limits<std::uint64_t>::max() - accepted) {
    throw Fault("Invalid host audio block size or frame counter exhausted");
  }
  if (finishing) { throw Fault("Host audio input is already finished"); }
  if (frames.size() > queue.available()) { throw Fault("Host audio queue capacity exhausted"); }
  auto previous_tick = previous_end;
  for (const auto &frame : frames) {
    if (frame.channels != channels || !frame.uniform_clock || !frame.bit_ticks ||
        *frame.bit_ticks != bit_ticks || frame.last_tick < frame.first_tick ||
        frame.last_tick - frame.first_tick != frame_span ||
        (previous_tick &&
         (frame.first_tick < *previous_tick || frame.first_tick - *previous_tick != bit_ticks))) {
      throw Fault("Host audio frame shape/timing does not match the declared serial clock");
    }
    for (unsigned channel = 0; channel < channels; ++channel) {
      if (frame.samples.at(channel) < minimum_sample ||
          frame.samples.at(channel) > maximum_sample) {
        throw Fault("Host audio sample exceeds signed 24-bit precision");
      }
    }
    previous_tick = frame.last_tick;
  }
}

HostAudio::HostAudio(const SerialPcmFormat format,
                     const HostAudioClock clock,
                     const HostAudioTarget &target)
    : stream_(std::make_unique<Stream>()) {
  const SerialPcmReceiver checked(format);
  if (target.server.contains('\0') || target.sink.contains('\0')) {
    throw Fault("Host audio target contains an embedded NUL");
  }
  stream_->target = target;
  stream_->spec = {.format = PA_SAMPLE_S32NE, .rate = clock.sample_rate, .channels = channels};
  const auto denominator =
      static_cast<std::uint64_t>(clock.sample_rate) * format.slot_bits * channels;
  if (format.channels != channels || format.word_bits != precision ||
      pa_sample_spec_valid(&stream_->spec) == 0 || denominator == 0 ||
      clock.timebase_hz < denominator || clock.timebase_hz % denominator != 0) {
    throw Fault("Host audio requires stereo signed 24-bit PCM and an integral declared clock");
  }
  stream_->bit_ticks = clock.timebase_hz / denominator;
  stream_->frame_span =
      (static_cast<std::uint64_t>(format.slot_bits) * channels - 1) * stream_->bit_ticks;
  stream_->loop.reset(pa_mainloop_new());
  if (!stream_->loop) { stream_->fail("mainloop creation", PA_ERR_INTERNAL); }
  stream_->context.reset(
      pa_context_new(pa_mainloop_get_api(stream_->loop.get()), "Blofeld emulator"));
  if (!stream_->context) { stream_->fail("context creation", PA_ERR_INTERNAL); }
  if (pa_context_connect(stream_->context.get(),
                         target.server.empty() ? nullptr : target.server.c_str(),
                         PA_CONTEXT_NOAUTOSPAWN,
                         nullptr) < 0) {
    stream_->fail("connect", pa_context_errno(stream_->context.get()));
  }
}

HostAudio::~HostAudio() = default;

void HostAudio::write(const std::span<const SerialPcmFrame> frames) {
  stream_->require_ready();
  stream_->validate(frames);
  for (const auto &frame : frames) {
    if (!stream_->queue.push(frame)) {
      throw Fault("Host audio queue changed during single-owner enqueue");
    }
  }
  stream_->previous_end = frames.back().last_tick;
  stream_->accepted += frames.size();
}

void HostAudio::poll(const HostAudioBoundary &boundary) {
  stream_->require_ready();
  stream_->boundary(boundary);
  stream_->progress();
}

void HostAudio::finish() {
  stream_->require_ready();
  stream_->finishing = true;
}

void HostAudio::retire() {
  stream_->close();
}

std::uint64_t HostAudio::frames_written() const {
  return stream_->written;
}

std::size_t HostAudio::available_frames() const {
  return stream_->queue.available();
}

std::size_t HostAudio::queued_frames() const {
  return stream_->queue.size();
}

bool HostAudio::ready() const {
  return !stream_->retired && stream_->connection &&
         pa_stream_get_state(stream_->connection.get()) == PA_STREAM_READY;
}

bool HostAudio::finished() const {
  return stream_->drained && stream_->drain_success;
}

bool HostAudio::faulted() const {
  return stream_->failed;
}
}
