// SPDX-License-Identifier: GPL-3.0-only
#include "audio/playback.hpp"

#include "audio/buffer.hpp"

#include <algorithm>
#include <atomic>
#include <bit>
#include <bits/types/struct_timeval.h>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <pulse/context.h>
#include <pulse/def.h>
#include <pulse/error.h>
#include <pulse/introspect.h>
#include <pulse/mainloop-api.h>
#include <pulse/mainloop.h>
#include <pulse/operation.h>
#include <pulse/rtclock.h>
#include <pulse/sample.h>
#include <pulse/stream.h>
#include <semaphore>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <string_view>
#include <thread>

namespace blofeld::host::audio {
namespace {
constexpr std::size_t frame_bytes = sizeof(Frame);
constexpr std::uint64_t timeout_us = 2000000;
constexpr std::int64_t unknown_latency = std::numeric_limits<std::int64_t>::min();
constexpr unsigned stereo_channels = 2;
static_assert(frame_bytes == stereo_channels * sizeof(std::int32_t));
static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
}

struct Playback::Impl {
  using Loop = std::unique_ptr<pa_mainloop, decltype(&pa_mainloop_free)>;
  using Context = std::unique_ptr<pa_context, decltype(&pa_context_unref)>;
  using Stream = std::unique_ptr<pa_stream, decltype(&pa_stream_unref)>;
  using Operation = std::unique_ptr<pa_operation, decltype(&pa_operation_unref)>;
  Loop loop{pa_mainloop_new(), pa_mainloop_free};
  Context context{nullptr, pa_context_unref};
  Stream stream{nullptr, pa_stream_unref};
  Buffer buffer;
  Grants granted;
  std::string name;
  unsigned rate;
  pa_time_event *timer = nullptr;
  pa_sample_format_t sink_format = PA_SAMPLE_INVALID;
  bool timed_out = false, query_done = false, query_failed = false;
  bool operation_done = false, operation_success = false, moved = false;
  std::size_t primed = 0;
  std::size_t target_frames = 0;
  std::atomic<std::size_t> server_buffer_frames{0};
  pa_usec_t next_measurement = 0;
  std::exception_ptr error;
  std::atomic<bool> ready{false}, failed{false}, started{false}, reset_pending{false};
  std::atomic<std::uint64_t> submitted{0}, underruns{0};
  std::atomic<std::int64_t> latency{unknown_latency};
  std::binary_semaphore reset_done{0};
  std::jthread owner;

  Impl(const unsigned requested_rate, const std::string_view client)
      : name(client), rate(requested_rate) {
    const pa_sample_spec specification{
        .format = PA_SAMPLE_S24_32NE, .rate = rate, .channels = stereo_channels};
    if (pa_sample_spec_valid(&specification) == 0) {
      throw std::invalid_argument("Invalid audio sample rate");
    }
    if (!loop) { throw std::runtime_error("Cannot create audio main loop"); }
    // Resolve the native wake pipe's cached descriptor type before cross-thread use.
    pa_mainloop_wakeup(loop.get());
    owner = std::jthread([this](const std::stop_token &stop) { run(stop); });
    ready.wait(false, std::memory_order_acquire);
    if (failed.load(std::memory_order_acquire)) { std::rethrow_exception(error); }
  }

  ~Impl() { stop(); }

  void stop() noexcept {
    owner.request_stop();
    buffer.close();
    pa_mainloop_wakeup(loop.get());
    if (owner.joinable()) { owner.join(); }
  }

  [[noreturn]] void fail(const std::string_view operation) const {
    throw std::runtime_error(std::string(operation) + ": " +
                             pa_strerror(pa_context_errno(context.get())));
  }

  void check(const std::stop_token &stop) const {
    if (stop.stop_requested()) { throw std::runtime_error("Audio operation canceled"); }
    if (timed_out) { throw std::runtime_error("Audio server operation timed out"); }
    const auto state = pa_context_get_state(context.get());
    if (!PA_CONTEXT_IS_GOOD(state)) { fail("Audio context disconnected"); }
    if (stream && !PA_STREAM_IS_GOOD(pa_stream_get_state(stream.get()))) {
      fail("Audio stream disconnected");
    }
    if (moved) { throw std::runtime_error("Audio endpoint changed; reopen required"); }
    if (underruns.load(std::memory_order_relaxed) != 0) {
      throw std::runtime_error("Host audio underrun");
    }
  }

  void deadline(const bool enabled) {
    const auto *api = pa_mainloop_get_api(loop.get());
    if (timer != nullptr) {
      api->time_free(timer);
      timer = nullptr;
    }
    timed_out = false;
    if (enabled) {
      timer = pa_context_rttime_new(
          context.get(),
          pa_rtclock_now() + timeout_us,
          [](pa_mainloop_api *, pa_time_event *, const timeval *, void *data) noexcept {
            static_cast<Impl *>(data)->timed_out = true;
          },
          this);
      if (timer == nullptr) { fail("Create audio deadline"); }
    }
  }

  void iterate(const std::stop_token &stop) const {
    check(stop);
    if (pa_mainloop_prepare(loop.get(), -1) < 0) { fail("Prepare audio operation"); }
    if (stop.stop_requested()) { pa_mainloop_wakeup(loop.get()); }
    if (pa_mainloop_poll(loop.get()) < 0 || pa_mainloop_dispatch(loop.get()) < 0) {
      fail("Dispatch audio events");
    }
    check(stop);
  }

  void open(const std::stop_token &stop) {
    context.reset(pa_context_new(pa_mainloop_get_api(loop.get()), name.c_str()));
    if (!context) { throw std::runtime_error("Cannot create audio context"); }
    deadline(true);
    if (pa_context_connect(context.get(), nullptr, PA_CONTEXT_NOAUTOSPAWN, nullptr) < 0) {
      fail("Connect audio server");
    }
    while (pa_context_get_state(context.get()) != PA_CONTEXT_READY) { iterate(stop); }
    const pa_sample_spec specification{
        .format = PA_SAMPLE_S24_32NE, .rate = rate, .channels = stereo_channels};
    stream.reset(pa_stream_new(context.get(), "PCM output", &specification, nullptr));
    if (!stream) { fail("Create audio stream"); }
    constexpr std::uint32_t target = 512;
    constexpr std::uint32_t minimum = 128;
    const pa_buffer_attr requested{.maxlength = static_cast<std::uint32_t>(rate * frame_bytes / 2),
                                   .tlength = static_cast<std::uint32_t>(target * frame_bytes),
                                   .prebuf = 0,
                                   .minreq = static_cast<std::uint32_t>(minimum * frame_bytes),
                                   .fragsize = std::numeric_limits<std::uint32_t>::max()};
    constexpr auto flags =
        std::bit_cast<pa_stream_flags_t>(PA_STREAM_START_CORKED | PA_STREAM_AUTO_TIMING_UPDATE |
                                         PA_STREAM_INTERPOLATE_TIMING | PA_STREAM_ADJUST_LATENCY);
    if (pa_stream_connect_playback(stream.get(), nullptr, &requested, flags, nullptr, nullptr) <
        0) {
      fail("Connect audio stream");
    }
    while (pa_stream_get_state(stream.get()) != PA_STREAM_READY) { iterate(stop); }
    query(stop);
    deadline(false);
    pa_stream_set_underflow_callback(
        stream.get(),
        [](pa_stream *, void *data) noexcept {
          auto &self = *static_cast<Impl *>(data);
          if (self.started.load(std::memory_order_relaxed)) {
            self.underruns.fetch_add(1, std::memory_order_relaxed);
          }
        },
        this);
    pa_stream_set_moved_callback(
        stream.get(),
        [](pa_stream *, void *data) noexcept { static_cast<Impl *>(data)->moved = true; },
        this);
  }

  const pa_buffer_attr &attributes() {
    const auto *value = pa_stream_get_buffer_attr(stream.get());
    if (value == nullptr || value->tlength == 0 || value->maxlength < value->tlength ||
        value->maxlength > rate * frame_bytes || value->tlength % frame_bytes != 0 ||
        value->minreq % frame_bytes != 0) {
      throw std::runtime_error("Unsupported or unbounded audio buffer grants");
    }
    target_frames = value->tlength / frame_bytes;
    server_buffer_frames.store(target_frames, std::memory_order_relaxed);
    return *value;
  }

  void query(const std::stop_token &stop) {
    const auto *actual = pa_stream_get_sample_spec(stream.get());
    const auto &limits = attributes();
    if (actual == nullptr || actual->rate != rate || actual->channels != stereo_channels ||
        actual->format != PA_SAMPLE_S24_32NE) {
      throw std::runtime_error("Unsupported or unbounded audio stream grants");
    }
    granted.rate = actual->rate;
    granted.target_frames = limits.tlength / frame_bytes;
    granted.maximum_frames = limits.maxlength / frame_bytes;
    granted.minimum_request_frames = limits.minreq / frame_bytes;
    const auto *device = pa_stream_get_device_name(stream.get());
    if (device == nullptr) { fail("Read audio endpoint"); }
    granted.sink = device;
    const Operation request(
        pa_context_get_sink_info_by_index(
            context.get(),
            pa_stream_get_device_index(stream.get()),
            [](pa_context *, const pa_sink_info *info, const int end, void *data) noexcept {
              auto &self = *static_cast<Impl *>(data);
              if (info != nullptr) {
                self.granted.sink_rate = info->sample_spec.rate;
                self.sink_format = info->sample_spec.format;
              }
              if (end != 0) {
                self.query_done = true;
                self.query_failed = end < 0;
              }
            },
            this),
        pa_operation_unref);
    if (!request) { fail("Query audio endpoint"); }
    while (!query_done) { iterate(stop); }
    if (query_failed || granted.sink_rate == 0) { fail("Read audio endpoint grants"); }
    const auto *format = pa_sample_format_to_string(sink_format);
    if (format == nullptr) { throw std::runtime_error("Unknown audio sink format"); }
    granted.sink_format = format;
  }

  void operation(pa_operation *request, const std::stop_token &stop) {
    const Operation pending(request, pa_operation_unref);
    if (!pending) { fail("Start audio control operation"); }
    deadline(true);
    try {
      while (!operation_done) { iterate(stop); }
    } catch (...) {
      pa_operation_cancel(pending.get());
      throw;
    }
    deadline(false);
    if (!operation_success) { fail("Complete audio control operation"); }
    operation_done = operation_success = false;
  }

  static void completed(pa_stream * /*stream*/, const int success, void *data) noexcept {
    auto &self = *static_cast<Impl *>(data);
    self.operation_success = success != 0;
    self.operation_done = true;
  }

  void reset(const std::stop_token &stop) {
    started.store(false, std::memory_order_release);
    operation(pa_stream_cork(stream.get(), 1, completed, this), stop);
    operation(pa_stream_flush(stream.get(), completed, this), stop);
    buffer.discard();
    primed = 0;
    latency.store(unknown_latency, std::memory_order_relaxed);
    reset_done.release();
  }

  void write_available(const std::stop_token &stop) {
    for (;;) {
      const auto frames = buffer.readable();
      if (frames.empty()) { break; }
      const auto available = pa_stream_writable_size(stream.get());
      if (available == std::numeric_limits<std::size_t>::max()) {
        fail("Query audio write capacity");
      }
      const auto count = std::min(frames.size(), available / frame_bytes);
      if (count == 0) { break; }
      if (pa_stream_write(
              stream.get(), frames.data(), count * frame_bytes, nullptr, 0, PA_SEEK_RELATIVE) < 0) {
        fail("Submit audio frames");
      }
      if (!buffer.consume(count)) { throw std::logic_error("Audio queue ownership is invalid"); }
      submitted.fetch_add(count, std::memory_order_relaxed);
      if (!started.load(std::memory_order_relaxed)) { primed += count; }
      if (!started.load(std::memory_order_relaxed) && primed >= target_frames) {
        operation(pa_stream_cork(stream.get(), 0, completed, this), stop);
        started.store(true, std::memory_order_release);
      }
    }
  }

  void update_latency() noexcept {
    pa_usec_t value = 0;
    int negative = 0;
    auto measured = unknown_latency;
    if (pa_stream_get_latency(stream.get(), &value, &negative) == 0 &&
        value <= static_cast<pa_usec_t>(std::numeric_limits<std::int64_t>::max())) {
      measured = static_cast<std::int64_t>(value);
      if (negative != 0) { measured = -measured; }
    }
    latency.store(measured, std::memory_order_relaxed);
  }

  void poll(const std::stop_token &stop) const {
    if (pa_mainloop_prepare(loop.get(), -1) < 0) { fail("Prepare audio events"); }
    // Recheck after prepare clears its wake pipe, preventing a lost producer wake.
    if (stop.stop_requested() || reset_pending.load(std::memory_order_acquire) ||
        (!buffer.readable().empty() && pa_stream_writable_size(stream.get()) >= frame_bytes)) {
      pa_mainloop_wakeup(loop.get());
    }
    if (pa_mainloop_poll(loop.get()) < 0 || pa_mainloop_dispatch(loop.get()) < 0) {
      fail("Poll audio events");
    }
  }

  void cleanup() noexcept {
    if (timer != nullptr) {
      pa_mainloop_get_api(loop.get())->time_free(timer);
      timer = nullptr;
    }
    if (stream) {
      pa_stream_set_underflow_callback(stream.get(), nullptr, nullptr);
      pa_stream_set_moved_callback(stream.get(), nullptr, nullptr);
      pa_stream_disconnect(stream.get());
      stream.reset();
    }
    if (context) {
      pa_context_disconnect(context.get());
      context.reset();
    }
  }

  void run(const std::stop_token &stop) noexcept {
    const std::stop_callback wake(stop, [this] { pa_mainloop_wakeup(loop.get()); });
    try {
      open(stop);
      ready.store(true, std::memory_order_release);
      ready.notify_all();
      while (!stop.stop_requested() && !buffer.closed()) {
        check(stop);
        if (reset_pending.exchange(false, std::memory_order_acq_rel)) { reset(stop); }
        write_available(stop);
        const auto now = pa_rtclock_now();
        if (now >= next_measurement) {
          static_cast<void>(attributes());
          update_latency();
          constexpr pa_usec_t measurement_period = 100000;
          next_measurement = now + measurement_period;
        }
        poll(stop);
      }
    } catch (...) {
      if (!stop.stop_requested()) {
        error = std::current_exception();
        failed.store(true, std::memory_order_release);
      }
    }
    cleanup();
    started.store(false, std::memory_order_release);
    buffer.close();
    ready.store(true, std::memory_order_release);
    ready.notify_all();
  }
};

Playback::Playback(const unsigned rate, const std::string_view name)
    : impl_(std::make_unique<Impl>(rate, name)) {}

Playback::~Playback() = default;

Playback::Producer Playback::producer() const {
  return Producer(std::chrono::nanoseconds::zero());
}

void Playback::stop() noexcept {
  impl_->stop();
}

const Grants &Playback::grants() const noexcept {
  return impl_->granted;
}

void Playback::check() const {
  if (impl_->failed.load(std::memory_order_acquire)) { std::rethrow_exception(impl_->error); }
}

bool Playback::write(std::span<const Frame> frames, const std::stop_token &stop) {
  while (!frames.empty() && !stop.stop_requested() && !impl_->buffer.closed()) {
    const auto count = impl_->buffer.produce(frames);
    if (count != 0) {
      frames = frames.subspan(count);
      pa_mainloop_wakeup(impl_->loop.get());
    } else if (!impl_->buffer.wait_space(stop)) {
      return false;
    }
  }
  return frames.empty() && !stop.stop_requested() && !impl_->buffer.closed();
}

bool Playback::reset(const std::stop_token &stop) {
  if (stop.stop_requested() || impl_->buffer.closed()) { return false; }
  impl_->reset_pending.store(true, std::memory_order_release);
  pa_mainloop_wakeup(impl_->loop.get());
  while (!stop.stop_requested() && !impl_->buffer.closed()) {
    if (impl_->reset_done.try_acquire_for(std::chrono::milliseconds(1))) { return true; }
  }
  // An abandoned reset is terminal: its completion token cannot satisfy a later reset.
  impl_->buffer.close();
  pa_mainloop_wakeup(impl_->loop.get());
  return false;
}

PlaybackStatus Playback::status() const noexcept {
  const auto delay = impl_->latency.load(std::memory_order_relaxed);
  return {.submitted_frames = impl_->submitted.load(std::memory_order_relaxed),
          .underruns = impl_->underruns.load(std::memory_order_relaxed),
          .queued_frames = impl_->buffer.queued(),
          .server_buffer_frames = impl_->server_buffer_frames.load(std::memory_order_relaxed),
          .latency_us = delay == unknown_latency ? std::optional<std::int64_t>{} : delay,
          .started = impl_->started.load(std::memory_order_acquire),
          .failed = impl_->failed.load(std::memory_order_acquire)};
}
}
