// SPDX-License-Identifier: GPL-3.0-only
#include "audio/host_audio.hpp"
#include "audio/host_audio_session.hpp"
#include "common/bus.hpp"

#include <optional>
#include <pulse/channelmap.h>
#include <pulse/context.h>
#include <pulse/def.h>
#include <pulse/sample.h>
#include <pulse/stream.h>
#include <string>
#include <string_view>

namespace blofeld {
void HostAudioOutput::Session::incompatible(const std::string_view reason) {
  faulted = true;
  throw Fault("Host audio incompatible negotiated stream: " + std::string(reason));
}

void HostAudioOutput::Session::refresh_stream() {
  const auto *spec = pa_stream_get_sample_spec(stream);
  const auto *mapping = pa_stream_get_channel_map(stream);
  const auto *buffer = pa_stream_get_buffer_attr(stream);
  if (spec == nullptr || mapping == nullptr || buffer == nullptr) { fail("stream readback"); }
  pa_channel_map stereo{};
  pa_channel_map_init_stereo(&stereo);
  if (spec->format != PA_SAMPLE_S32NE || spec->channels != channels ||
      spec->rate != config.sample_rate || pa_channel_map_equal(mapping, &stereo) == 0) {
    incompatible(
        "client PCM must retain signed32-native front-left/front-right at source rate=" +
        std::to_string(config.sample_rate) + "; actual format=" + std::to_string(spec->format) +
        " channels=" + std::to_string(spec->channels) + " rate=" + std::to_string(spec->rate));
  }
  if (buffer->maxlength == 0 || buffer->tlength == 0 || buffer->minreq == 0 ||
      buffer->maxlength % frame_bytes != 0 || buffer->tlength % frame_bytes != 0 ||
      buffer->prebuf % frame_bytes != 0 || buffer->minreq % frame_bytes != 0 ||
      buffer->tlength > buffer->maxlength || buffer->prebuf > buffer->tlength ||
      buffer->minreq > buffer->tlength) {
    incompatible("buffer dimensions must hold aligned complete frames: maximum=" +
                 std::to_string(buffer->maxlength) + " target=" + std::to_string(buffer->tlength) +
                 " prebuffer=" + std::to_string(buffer->prebuf) +
                 " minimum-request=" + std::to_string(buffer->minreq));
  }
  HostAudioStreamInfo actual{.sample_rate = spec->rate,
                             .channels = spec->channels,
                             .maximum_frames = buffer->maxlength / frame_bytes,
                             .target_frames = buffer->tlength / frame_bytes,
                             .prebuffer_frames = buffer->prebuf / frame_bytes,
                             .minimum_request_frames = buffer->minreq / frame_bytes,
                             .stream_index = pa_stream_get_index(stream),
                             .device_index = std::nullopt,
                             .suspended = std::nullopt};
  if (actual.stream_index == PA_INVALID_INDEX) { fail("stream index readback"); }
  const auto device = pa_stream_get_device_index(stream);
  if (device != PA_INVALID_INDEX) {
    actual.device_index = device;
  } else if (pa_context_errno(context) != PA_ERR_NOTSUPPORTED) {
    fail("device index readback");
  }
  const auto suspended = pa_stream_is_suspended(stream);
  if (suspended >= 0) {
    actual.suspended = suspended != 0;
  } else if (pa_context_errno(context) != PA_ERR_NOTSUPPORTED) {
    fail("suspend state readback");
  }
  // Publish only after the complete snapshot validates. This is client stream
  // information; the audio service owns conversion to a different sink format.
  observed_stream = actual;
}
}
