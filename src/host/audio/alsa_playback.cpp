// SPDX-License-Identifier: GPL-3.0-only
#include "audio/alsa_playback.hpp"

#include "audio/buffer.hpp"
#include "audio/playback.hpp"
#include "audio/render_position.hpp"

#include <algorithm>
#include <alsa/asoundlib.h>
#include <array>
#include <atomic>
#include <bit>
#include <bits/time.h>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <semaphore>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <string_view>
#include <sys/eventfd.h>
#include <sys/poll.h>
#include <thread>
#include <tuple>
#include <unistd.h>
#include <vector>

extern "C" {
#include <time.h>
}

namespace blofeld::host::audio {
namespace {
constexpr auto stereo = std::tuple_size_v<Frame>;
constexpr unsigned max_channels = 32, max_sample_bytes = 4;
constexpr snd_pcm_uframes_t requested_period = 16, requested_ring = 128;
constexpr std::int64_t unknown_delay = -1, us_per_second = 1'000'000;
constexpr std::uint64_t ns_per_second = 1'000'000'000, query_bound_ns = 200'000;
constexpr timespec status_interval{.tv_sec = 0, .tv_nsec = 250'000};

std::uint64_t stamp_ns(const timespec stamp) noexcept {
  return (static_cast<std::uint64_t>(stamp.tv_sec) * ns_per_second) +
         static_cast<std::uint64_t>(stamp.tv_nsec);
}

std::uint64_t monotonic_ns() {
  timespec stamp{};
  if (clock_gettime(CLOCK_MONOTONIC, &stamp) != 0) {
    throw std::runtime_error("Read monotonic ALSA clock");
  }
  return stamp_ns(stamp);
}

struct Wake {
  Wake() : fd(eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK)) {
    if (fd < 0) { throw std::runtime_error("Create ALSA owner wake descriptor"); }
  }

  ~Wake() { close(fd); }

  Wake(const Wake &) = delete;
  Wake &operator=(const Wake &) = delete;

  void signal() const noexcept {
    constexpr std::uint64_t value = 1;
    static_cast<void>(::write(fd, &value, sizeof(value)));
  }

  void drain() const noexcept {
    std::uint64_t value = 0;
    static_cast<void>(::read(fd, &value, sizeof(value)));
  }

  const int fd;
};

snd_pcm_format_t exact_format(snd_pcm_t *raw, snd_pcm_hw_params_t *parameters) {
  for (const auto format : {SND_PCM_FORMAT_S24_3LE, SND_PCM_FORMAT_S24_LE, SND_PCM_FORMAT_S32_LE}) {
    if (snd_pcm_hw_params_test_format(raw, parameters, format) == 0) { return format; }
  }
  throw std::runtime_error("ALSA endpoint cannot preserve 24-bit stereo PCM");
}
}

struct AlsaPlayback::Impl {
  using Pcm = std::unique_ptr<snd_pcm_t, decltype(&snd_pcm_close)>;
  Pcm pcm{nullptr, snd_pcm_close};
  Wake wake;
  Buffer buffer;
  RenderPositionStore position;
  Grants granted;
  std::exception_ptr failure;
  std::atomic<bool> ready{false}, failed{false}, started{false}, reset_pending{false};
  std::atomic<std::uint64_t> submitted{0}, underruns{0};
  std::atomic<std::int64_t> delay_us{unknown_delay};
  std::binary_semaphore reset_done{0};
  std::vector<pollfd> descriptors;
  std::array<std::uint8_t, Buffer::capacity * max_channels * max_sample_bytes> packed{};
  std::uint64_t epoch = 0, epoch_started = 0, written = 0, consumed = 0, timestamp = 0;
  snd_pcm_uframes_t length = requested_ring, period = requested_period;
  unsigned channels = stereo, sample_bytes = 0, sample_shift = 0;
  const unsigned rate;
  const std::string device;
  std::jthread owner;

  Impl(const unsigned sample_rate, const std::string_view pcm_device)
      : rate(sample_rate), device(pcm_device) {
    if (rate == 0 || !device.starts_with("hw:") || device.size() == 3) {
      throw std::invalid_argument(
          "Direct ALSA requires an explicit hw: PCM device and sample rate");
    }
    owner = std::jthread([this](const std::stop_token &stop) { run(stop); });
    ready.wait(false, std::memory_order_acquire);
    check();
  }

  ~Impl() { stop(); }

  void stop() noexcept {
    owner.request_stop();
    buffer.close();
    wake.signal();
    if (owner.joinable()) { owner.join(); }
  }

  void check() const {
    if (failed.load(std::memory_order_acquire)) { std::rethrow_exception(failure); }
  }

  void require(const int result, const char *action) {
    if (result >= 0) { return; }
    if (result == -EPIPE) { underruns.fetch_add(1, std::memory_order_relaxed); }
    throw std::runtime_error(std::string(action) + ": " + snd_strerror(result));
  }

  void open() {
    snd_pcm_t *raw = nullptr;
    require(snd_pcm_open(&raw, device.c_str(), SND_PCM_STREAM_PLAYBACK, SND_PCM_NONBLOCK),
            "Open direct ALSA PCM");
    pcm.reset(raw);
    if (snd_pcm_type(raw) != SND_PCM_TYPE_HW) {
      throw std::runtime_error("Direct ALSA requires a hardware PCM");
    }
    snd_pcm_hw_params_t *hardware;
    snd_pcm_hw_params_alloca(&hardware);
    require(snd_pcm_hw_params_any(raw, hardware), "Query ALSA hardware");
    require(snd_pcm_hw_params_set_access(raw, hardware, SND_PCM_ACCESS_RW_INTERLEAVED),
            "Set ALSA interleaved access");
    require(snd_pcm_hw_params_set_rate(raw, hardware, rate, 0), "Set native ALSA sample rate");
    const auto format = exact_format(raw, hardware);
    require(snd_pcm_hw_params_set_format(raw, hardware, format), "Set ALSA sample format");
    require(snd_pcm_hw_params_set_channels_near(raw, hardware, &channels), "Set ALSA channels");
    if (channels < stereo || channels > max_channels) {
      throw std::runtime_error("ALSA hardware granted an unsupported channel count");
    }
    sample_bytes = format == SND_PCM_FORMAT_S24_3LE ? 3U : max_sample_bytes;
    sample_shift = format == SND_PCM_FORMAT_S32_LE ? 8U : 0U;
    require(snd_pcm_hw_params_set_period_size_near(raw, hardware, &period, nullptr),
            "Request ALSA period");
    require(snd_pcm_hw_params_set_buffer_size_near(raw, hardware, &length), "Request ALSA ring");
    require(snd_pcm_hw_params(raw, hardware), "Apply ALSA hardware parameters");
    require(snd_pcm_get_params(raw, &length, &period), "Read ALSA grants");
    if (period == 0 || period >= length || length > rate) {
      throw std::runtime_error("Invalid ALSA period or ring grant");
    }
    const auto maximum = length + Buffer::capacity;
    constexpr auto history_queues = 3;
    granted = {.rate = rate,
               .sink_rate = rate,
               .target_frames = length,
               .maximum_frames = maximum,
               .minimum_request_frames = period,
               .position_history_frames = history_queues * maximum,
               .maximum_position_uncertainty_ns = query_bound_ns,
               .sink = device,
               .sink_format = snd_pcm_format_name(format)};
    configure(raw);
  }

  void configure(snd_pcm_t *raw) {
    snd_pcm_sw_params_t *software;
    snd_pcm_sw_params_alloca(&software);
    require(snd_pcm_sw_params_current(raw, software), "Query ALSA software parameters");
    require(snd_pcm_sw_params_set_avail_min(raw, software, period), "Set ALSA refill quantum");
    require(snd_pcm_sw_params_set_tstamp_mode(raw, software, SND_PCM_TSTAMP_ENABLE),
            "Enable ALSA timestamps");
    require(snd_pcm_sw_params_set_tstamp_type(raw, software, SND_PCM_TSTAMP_TYPE_MONOTONIC),
            "Set ALSA timestamp clock");
    require(snd_pcm_sw_params_set_start_threshold(raw, software, length),
            "Set ALSA start watermark");
    require(snd_pcm_sw_params_set_stop_threshold(raw, software, length), "Set ALSA stop watermark");
    require(snd_pcm_sw_params(raw, software), "Apply ALSA software parameters");
    require(snd_pcm_prepare(raw), "Prepare ALSA PCM");
    epoch_started = monotonic_ns();
    const auto count = snd_pcm_poll_descriptors_count(raw);
    require(count, "Query ALSA poll descriptors");
    if (count == 0) { throw std::runtime_error("ALSA has no playback descriptor"); }
    descriptors.resize(static_cast<std::size_t>(count) + 1);
    descriptors.front() = {.fd = wake.fd, .events = POLLIN, .revents = 0};
    if (snd_pcm_poll_descriptors(raw, descriptors.data() + 1, static_cast<unsigned>(count)) !=
        count) {
      throw std::runtime_error("Read ALSA poll descriptors");
    }
  }

  void replace_epoch() {
    if (epoch == std::numeric_limits<std::uint64_t>::max() || !position.clear()) {
      throw std::overflow_error("ALSA PCM epoch exhausted");
    }
    started.store(false, std::memory_order_release);
    require(snd_pcm_drop(pcm.get()), "Stop old ALSA PCM epoch");
    buffer.discard();
    require(snd_pcm_prepare(pcm.get()), "Prepare new ALSA PCM epoch");
    ++epoch;
    epoch_started = monotonic_ns();
    written = consumed = timestamp = 0;
    delay_us.store(unknown_delay, std::memory_order_relaxed);
    reset_done.release();
  }

  void submit() {
    const std::size_t frame_bytes = std::size_t{channels} * sample_bytes;
    for (auto frames = buffer.readable(); !frames.empty(); frames = buffer.readable()) {
      for (std::size_t n = 0; n < frames.size(); ++n) {
        for (std::size_t ch = 0; ch < stereo; ++ch) {
          const auto bits = std::bit_cast<std::uint32_t>(frames[n][ch]) << sample_shift;
          const auto offset = (n * frame_bytes) + (ch * sample_bytes);
          for (unsigned b = 0; b < sample_bytes; ++b) {
            packed[offset + b] = static_cast<std::uint8_t>(bits >> (b * 8U));
          }
        }
      }
      const auto count = snd_pcm_writei(pcm.get(), packed.data(), frames.size());
      if (count == -EAGAIN || count == 0) { return; }
      require(static_cast<int>(count), "Submit ALSA PCM");
      const auto accepted = static_cast<std::size_t>(count);
      if (accepted > std::numeric_limits<std::uint64_t>::max() - written ||
          !buffer.consume(accepted)) {
        throw std::runtime_error("Invalid ALSA write progress");
      }
      written += accepted;
      submitted.fetch_add(accepted, std::memory_order_relaxed);
    }
  }

  void observe(snd_pcm_status_t *status) {
    if (written < length) { return; }
    const auto before = monotonic_ns();
    require(snd_pcm_status(pcm.get(), status), "Observe ALSA PCM");
    const auto after = monotonic_ns();
    const auto state = snd_pcm_status_get_state(status);
    if (state == SND_PCM_STATE_XRUN) { require(-EPIPE, "ALSA playback underrun"); }
    if (state != SND_PCM_STATE_RUNNING) { throw std::runtime_error("ALSA PCM stopped"); }
    const auto available = snd_pcm_status_get_avail(status);
    // The status can retain RUNNING from before ALSA updated its hardware pointer.
    if (available >= length) { require(-EPIPE, "ALSA playback ring exhausted"); }
    const auto next = written - (length - available);
    const auto delay = snd_pcm_status_get_delay(status);
    if (next < consumed || delay < 0 || delay > static_cast<snd_pcm_sframes_t>(rate)) {
      throw std::runtime_error("Invalid ALSA playback position or delay");
    }
    delay_us.store(delay * us_per_second / rate, std::memory_order_relaxed);
    started.store(true, std::memory_order_release);
    if (next == consumed) { return; }
    consumed = next;
    snd_htimestamp_t stamp{};
    snd_pcm_status_get_htstamp(status, &stamp);
    const auto point = stamp_ns(stamp);
    if (point == timestamp) { return; }
    if (after < before || point < epoch_started || point > after || point < timestamp) {
      throw std::runtime_error("Invalid ALSA monotonic playback timestamp");
    }
    const auto uncertainty = after - std::min(before, point);
    if (uncertainty > query_bound_ns) { return; }
    if (!position.publish({.epoch = epoch,
                           .first_frame = consumed - 1,
                           .host_ns = point,
                           .frames = 1,
                           .uncertainty_ns = uncertainty})) {
      throw std::overflow_error("ALSA PCM position exhausted");
    }
    timestamp = point;
  }

  void run(const std::stop_token &stop_token) noexcept {
    const std::stop_callback cancel(stop_token, [this] { wake.signal(); });
    try {
      open();
      ready.store(true, std::memory_order_release);
      ready.notify_all();
      snd_pcm_status_t *status;
      snd_pcm_status_alloca(&status);
      while (!stop_token.stop_requested() && !buffer.closed()) {
        if (reset_pending.exchange(false, std::memory_order_acq_rel)) { replace_epoch(); }
        submit();
        observe(status);
        const auto count = buffer.readable().empty() ? 1U : descriptors.size();
        const auto *timeout = started.load(std::memory_order_relaxed) ? &status_interval : nullptr;
        if (ppoll(descriptors.data(), count, timeout, nullptr) < 0 && errno != EINTR) {
          throw std::runtime_error("Poll ALSA playback descriptors");
        }
        if ((descriptors.front().revents & POLLIN) != 0) { wake.drain(); }
      }
    } catch (...) {
      if (!stop_token.stop_requested()) {
        failure = std::current_exception();
        failed.store(true, std::memory_order_release);
      }
    }
    static_cast<void>(position.clear());
    pcm.reset();
    started.store(false, std::memory_order_release);
    buffer.close();
    ready.store(true, std::memory_order_release);
    ready.notify_all();
  }
};

AlsaPlayback::AlsaPlayback(const unsigned rate, const std::string_view device)
    : impl_(std::make_unique<Impl>(rate, device)) {}

AlsaPlayback::~AlsaPlayback() = default;

void AlsaPlayback::stop() noexcept {
  impl_->stop();
}

void AlsaPlayback::check() const {
  impl_->check();
}

const Grants &AlsaPlayback::grants() const noexcept {
  return impl_->granted;
}

bool AlsaPlayback::write(std::span<const Frame> frames, const std::stop_token &stop) {
  while (!frames.empty() && !stop.stop_requested() && !impl_->buffer.closed()) {
    const auto count = impl_->buffer.produce(frames);
    if (count != 0) {
      frames = frames.subspan(count);
      impl_->wake.signal();
    } else if (!impl_->buffer.wait_space(stop)) {
      return false;
    }
  }
  return frames.empty() && !stop.stop_requested() && !impl_->buffer.closed();
}

bool AlsaPlayback::reset(const std::stop_token &stop) {
  if (stop.stop_requested() || impl_->buffer.closed()) { return false; }
  impl_->reset_pending.store(true, std::memory_order_release);
  impl_->wake.signal();
  while (!stop.stop_requested() && !impl_->buffer.closed()) {
    if (impl_->reset_done.try_acquire_for(std::chrono::milliseconds(1))) { return true; }
  }
  impl_->buffer.close();
  impl_->wake.signal();
  return false;
}

PlaybackStatus AlsaPlayback::status() const noexcept {
  const auto delay = impl_->delay_us.load(std::memory_order_relaxed);
  const auto active = impl_->started.load(std::memory_order_acquire);
  const auto failed = impl_->failed.load(std::memory_order_acquire);
  return {.submitted_frames = impl_->submitted.load(std::memory_order_relaxed),
          .underruns = impl_->underruns.load(std::memory_order_relaxed),
          .queued_frames = impl_->buffer.queued(),
          .server_buffer_frames = impl_->granted.target_frames,
          .latency_us = delay == unknown_delay ? std::nullopt : std::optional(delay),
          .started = active,
          .failed = failed,
          .position = active && !failed ? impl_->position.read() : std::nullopt};
}
}
