// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/host_audio.hpp"
#include "blofeld/serial_pcm.hpp"
#include "host_audio_session.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <pulse/stream.h>
#include <span>

namespace blofeld {
void HostAudioOutput::Session::validate(const std::span<const SerialPcmFrame> frames) const {
  auto end = previous_end;
  for (const auto &frame : frames) {
    if (frame.channels != channels || !frame.uniform_clock || frame.bit_ticks != bit_ticks ||
        frame.last_tick < frame.first_tick || frame.last_tick - frame.first_tick != frame_span ||
        (end && (frame.first_tick < *end || frame.first_tick - *end != bit_ticks))) {
      throw Fault("Host audio PCM shape/ticks do not match the declared rate");
    }
    for (unsigned channel = 0; channel < channels; ++channel) {
      const auto sample = frame.samples.at(channel);
      if (sample < -sample_limit || sample >= sample_limit) {
        throw Fault("Host audio PCM sample exceeds declared precision");
      }
    }
    end = frame.last_tick;
  }
}

std::size_t HostAudioOutput::Session::write(const std::span<const SerialPcmFrame> frames) {
  poll();
  if (status.drained) { throw Fault("Host audio cannot write after drain"); }
  if (frames.size() > batch_frames) { throw Fault("Host audio PCM batch exceeds its bound"); }
  validate(frames);
  const auto writable = pa_stream_writable_size(stream);
  if (writable == std::numeric_limits<std::size_t>::max()) { fail("writable size"); }
  if (writable % frame_bytes != 0) { fail("unaligned server capacity"); }
  const auto count = std::min(frames.size(), writable / frame_bytes);
  if (count == 0) { return 0; }
  if (count > std::numeric_limits<std::uint64_t>::max() - status.frames_written) {
    throw Fault("Host audio frame counter exhausted");
  }
  for (std::size_t index = 0; index < count; ++index) {
    for (unsigned channel = 0; channel < channels; ++channel) {
      samples.at((index * channels) + channel) =
          static_cast<std::int32_t>(frames[index].samples.at(channel) * scale);
    }
  }
  if (pa_stream_write(stream, samples.data(), count * frame_bytes, nullptr, 0, PA_SEEK_RELATIVE) <
      0) {
    fail("write");
  }
  previous_end = frames[count - 1].last_tick;
  status.frames_written += count;
  return count;
}

std::size_t HostAudioOutput::write(const std::span<const SerialPcmFrame> frames) {
  return session_->write(frames);
}
}
