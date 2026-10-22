// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/host_audio.hpp"
#include "blofeld/serial_pcm.hpp"
#include "host_audio_internal.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <pulse/channelmap.h>
#include <pulse/context.h>
#include <pulse/def.h>
#include <pulse/mainloop.h>
#include <pulse/stream.h>

namespace blofeld {
void HostAudio::Stream::close() {
  drain.reset();
  connection.reset();
  context.reset();
  loop.reset();
  retired = true;
}

void HostAudio::Stream::boundary(const HostAudioBoundary &value) {
  if (!value.owner || (previous && (value.owner != previous->owner ||
                                    value.restore_revision != previous->restore_revision ||
                                    value.ticks < previous->ticks))) {
    failed = true;
    close();
    throw Fault(
        "Host audio board owner/restore/time changed; transport retired before playback progress");
  }
  previous = value;
}

void HostAudio::Stream::connect_stream() {
  pa_channel_map map{};
  pa_channel_map_init_stereo(&map);
  connection.reset(pa_stream_new(context.get(), "DSP PCM output", &spec, &map));
  if (!connection) { fail("stream creation", pa_context_errno(context.get())); }
  if (pa_stream_connect_playback(connection.get(),
                                 target.sink.empty() ? nullptr : target.sink.c_str(),
                                 nullptr,
                                 PA_STREAM_NOFLAGS,
                                 nullptr,
                                 nullptr) < 0) {
    fail("stream connect", pa_context_errno(context.get()));
  }
}

void HostAudio::Stream::submit() {
  constexpr unsigned channels = 2;
  constexpr auto frame_bytes = channels * sizeof(std::int32_t);
  const auto writable = pa_stream_writable_size(connection.get());
  if (writable == std::numeric_limits<std::size_t>::max()) {
    fail("writable size", pa_context_errno(context.get()));
  }
  const auto count = std::min({maximum_block_frames, queue.size(), writable / frame_bytes});
  if (count == 0) { return; }
  std::array<std::int32_t, maximum_block_frames * channels> samples{};
  constexpr std::int64_t container_scale = 256; // Original signed 24 bits in high S32NE bits.
  for (std::size_t frame = 0; frame < count; ++frame) {
    const auto *const input = queue.peek(frame);
    if (input == nullptr) { throw Fault("Host audio queue changed during single-owner submit"); }
    for (unsigned channel = 0; channel < channels; ++channel) {
      samples.at((frame * channels) + channel) =
          static_cast<std::int32_t>(input->samples.at(channel) * container_scale);
    }
  }
  if (pa_stream_write(
          connection.get(), samples.data(), count * frame_bytes, nullptr, 0, PA_SEEK_RELATIVE) <
      0) {
    fail("write", pa_context_errno(context.get()));
  }
  static_cast<void>(queue.pop(count));
  written += count;
}

void HostAudio::Stream::drained_callback(pa_stream *source,
                                         const int success,
                                         void *opaque) noexcept {
  static_cast<void>(source);
  auto &self = *static_cast<Stream *>(opaque);
  self.drained = true;
  self.drain_success = success != 0;
}

void HostAudio::Stream::progress() {
  if (pa_mainloop_iterate(loop.get(), 0, nullptr) < 0) {
    fail("mainloop", pa_context_errno(context.get()));
  }
  const auto state = pa_context_get_state(context.get());
  if (!PA_CONTEXT_IS_GOOD(state)) { fail("connection", pa_context_errno(context.get())); }
  if (state != PA_CONTEXT_READY) { return; }
  if (!connection) { connect_stream(); }
  const auto stream_state = pa_stream_get_state(connection.get());
  if (!PA_STREAM_IS_GOOD(stream_state)) { fail("stream state", pa_context_errno(context.get())); }
  if (stream_state != PA_STREAM_READY) { return; }
  submit();
  if (finishing && queue.size() == 0 && !drain) {
    drain.reset(pa_stream_drain(connection.get(), drained_callback, this));
    if (!drain) { fail("drain", pa_context_errno(context.get())); }
  }
  if (drained && !drain_success) { fail("drain completion", pa_context_errno(context.get())); }
}
}
