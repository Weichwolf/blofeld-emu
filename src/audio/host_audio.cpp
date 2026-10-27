// SPDX-License-Identifier: GPL-3.0-only
#include "audio/host_audio.hpp"

#include "audio/host_audio_session.hpp"
#include "audio/serial_pcm.hpp"
#include "common/bus.hpp"

#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <pulse/channelmap.h>
#include <pulse/context.h>
#include <pulse/def.h>
#include <pulse/error.h>
#include <pulse/mainloop.h>
#include <pulse/operation.h>
#include <pulse/sample.h>
#include <pulse/stream.h>
#include <string>
#include <string_view>
#include <thread>
#include <utility>

namespace blofeld {
namespace {
constexpr std::chrono::milliseconds maximum_timeout{10000};
constexpr std::uint32_t maximum_buffer_frames = 65536;
constexpr unsigned container_bits = 32;
}

HostAudioOutput::Session::Session(HostAudioConfig setup) : config(std::move(setup)) {
  const SerialPcmReceiver validated(config.pcm);
  if (config.pcm.channels != channels || config.sample_rate == 0 || config.timeout.count() <= 0 ||
      config.timeout > maximum_timeout || config.buffer_frames < batch_frames ||
      config.buffer_frames > maximum_buffer_frames ||
      config.server.find('\0') != std::string::npos) {
    throw Fault("Invalid host audio channels, rate, timeout, buffer or server");
  }
  const auto bits = static_cast<std::uint64_t>(config.pcm.slot_bits) * channels;
  const auto divisor = static_cast<std::uint64_t>(config.sample_rate) * bits;
  if (config.timebase_hz < divisor || config.timebase_hz % divisor != 0) {
    throw Fault("Host audio rate/bit clock is not integral in the declared timebase");
  }
  bit_ticks = config.timebase_hz / divisor;
  frame_span = (bits - 1) * bit_ticks;
  sample_limit = std::int64_t{1} << (config.pcm.word_bits - 1);
  scale = std::int64_t{1} << (container_bits - config.pcm.word_bits);
}

HostAudioOutput::Session::~Session() {
  if (stream != nullptr) {
    pa_stream_set_underflow_callback(stream, nullptr, nullptr);
    (void)pa_stream_disconnect(stream);
    pa_stream_unref(stream);
  }
  if (context != nullptr) {
    pa_context_disconnect(context);
    pa_context_unref(context);
  }
  if (loop != nullptr) { pa_mainloop_free(loop); }
}

void HostAudioOutput::Session::fail(const std::string_view name) {
  faulted = true;
  const auto code = context == nullptr ? PA_ERR_INTERNAL : pa_context_errno(context);
  throw Fault("Host audio " + std::string(name) + ": " + pa_strerror(code));
}

void HostAudioOutput::Session::timeout(const std::string_view name) {
  faulted = true;
  throw Fault("Host audio " + std::string(name) + " deadline expired");
}

void HostAudioOutput::Session::check() {
  if (faulted) { throw Fault("Host audio failure requires a new session"); }
  if (context != nullptr && !PA_CONTEXT_IS_GOOD(pa_context_get_state(context))) {
    fail("connection");
  }
  if (stream != nullptr && !PA_STREAM_IS_GOOD(pa_stream_get_state(stream))) { fail("stream"); }
}

void HostAudioOutput::Session::poll() {
  check();
  if (pa_mainloop_iterate(loop, 0, nullptr) < 0) { fail("mainloop"); }
  check();
  if (stream != nullptr && pa_stream_get_state(stream) == PA_STREAM_READY) { refresh_stream(); }
}

void HostAudioOutput::Session::connect() {
  loop = pa_mainloop_new();
  if (loop == nullptr) { fail("mainloop allocation"); }
  context = pa_context_new(pa_mainloop_get_api(loop), "Blofeld Emulator");
  if (context == nullptr) { fail("context allocation"); }
  const auto *server = config.server.empty() ? nullptr : config.server.c_str();
  if (pa_context_connect(context, server, PA_CONTEXT_NOAUTOSPAWN, nullptr) < 0) { fail("connect"); }
  const auto deadline = std::chrono::steady_clock::now() + config.timeout;
  while (pa_context_get_state(context) != PA_CONTEXT_READY) {
    poll();
    if (std::chrono::steady_clock::now() >= deadline) { timeout("connect"); }
    std::this_thread::sleep_for(std::chrono::milliseconds{1});
  }
  const pa_sample_spec spec{
      .format = PA_SAMPLE_S32NE, .rate = config.sample_rate, .channels = channels};
  if (pa_sample_spec_valid(&spec) == 0) { throw Fault("Unsupported host audio sample rate"); }
  pa_channel_map mapping{};
  pa_channel_map_init_stereo(&mapping);
  stream = pa_stream_new(context, "Blofeld DSP output", &spec, &mapping);
  if (stream == nullptr) { fail("stream allocation"); }
  pa_stream_set_underflow_callback(stream, underflow, this);
  const auto bytes = config.buffer_frames * frame_bytes;
  const pa_buffer_attr buffer{.maxlength = bytes,
                              .tlength = bytes,
                              .prebuf = bytes,
                              .minreq = std::numeric_limits<std::uint32_t>::max(),
                              .fragsize = frame_bytes};
  if (pa_stream_connect_playback(
          stream, nullptr, &buffer, PA_STREAM_START_CORKED, nullptr, nullptr) < 0) {
    fail("playback connect");
  }
  while (pa_stream_get_state(stream) != PA_STREAM_READY) {
    poll();
    if (std::chrono::steady_clock::now() >= deadline) { timeout("playback connect"); }
    std::this_thread::sleep_for(std::chrono::milliseconds{1});
  }
}

void HostAudioOutput::Session::completed(pa_stream * /*stream*/,
                                         const int success,
                                         void *data) noexcept {
  *static_cast<int *>(data) = success != 0 ? 1 : -1;
}

void HostAudioOutput::Session::underflow(pa_stream *stream, void *data) noexcept {
  auto &session = *static_cast<Session *>(data);
  const auto position = pa_stream_get_underflow_index(stream);
  if (session.status.underflows == 0 || position < session.status.earliest_underflow_byte) {
    session.status.earliest_underflow_byte = position;
  }
  if (session.status.underflows != UINT64_MAX) { ++session.status.underflows; }
}

void HostAudioOutput::Session::operation(pa_operation *pending,
                                         int &result,
                                         const std::string_view name) {
  if (pending == nullptr) { fail(name); }
  const std::unique_ptr<pa_operation, decltype(&pa_operation_unref)> release(pending,
                                                                             pa_operation_unref);
  const auto deadline = std::chrono::steady_clock::now() + config.timeout;
  try {
    while (pa_operation_get_state(pending) == PA_OPERATION_RUNNING) {
      poll();
      if (std::chrono::steady_clock::now() >= deadline) { timeout(name); }
      std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
    if (pa_operation_get_state(pending) != PA_OPERATION_DONE || result != 1) { fail(name); }
  } catch (...) {
    pa_operation_cancel(pending);
    throw;
  }
}

HostAudioOutput::HostAudioOutput(HostAudioConfig config)
    : session_(std::make_unique<Session>(std::move(config))) {
  session_->connect();
}

HostAudioOutput::~HostAudioOutput() = default;

void HostAudioOutput::poll() {
  session_->poll();
}

HostAudioStatus HostAudioOutput::status() const {
  return session_->status;
}

HostAudioStreamInfo HostAudioOutput::stream_info() const {
  return session_->observed_stream;
}

void HostAudioOutput::start() {
  auto &session = *session_;
  session.poll();
  if (session.status.started || session.status.frames_written == 0) {
    throw Fault("Host audio start requires a queued, corked stream");
  }
  int success = 0;
  session.operation(
      pa_stream_cork(session.stream, 0, Session::completed, &success), success, "start");
  session.status.started = true;
}

void HostAudioOutput::drain() {
  auto &session = *session_;
  session.poll();
  if (!session.status.started || session.status.drained) {
    throw Fault("Host audio drain requires a running stream");
  }
  int success = 0;
  session.operation(
      pa_stream_drain(session.stream, Session::completed, &success), success, "drain");
  session.status.drained = true;
}
}
