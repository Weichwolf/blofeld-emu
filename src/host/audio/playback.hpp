// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/pcm_frame.hpp"
#include "audio/render_position.hpp"

#if defined(__linux__)
#include "midi/ump.hpp"

#include <pthread.h>
#include <sched.h>
#endif

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <stop_token>
#include <string>
#include <string_view>

namespace blofeld::host::audio {
struct Grants {
  unsigned rate = 0, sink_rate = 0;
  std::size_t target_frames = 0, maximum_frames = 0;
  std::optional<std::size_t> minimum_request_frames;
  std::optional<std::size_t> position_history_frames;
  std::optional<std::uint64_t> maximum_position_uncertainty_ns;
  std::string sink, sink_format;
};

struct PlaybackStatus {
  std::uint64_t submitted_frames = 0, underruns = 0;
  std::size_t queued_frames = 0, server_buffer_frames = 0;
  std::optional<std::int64_t> latency_us;
  bool started = false, failed = false;
  // Render submission clock; unavailable before priming or on an unqualified backend.
  std::optional<RenderPosition> position;
};

// Push-mode native client. One producer supplies complete stereo frames. The
// I/O owner handles device events; render callbacks only consume prepared frames.
class Playback {
public:
  // Darwin permanently opts the dedicated producer thread out of QoS scheduling.
  class Producer {
  public:
    ~Producer();
    Producer(Producer &&) noexcept;
    Producer(const Producer &) = delete;
    Producer &operator=(const Producer &) = delete;
    Producer &operator=(Producer &&) = delete;
    void begin_work();
    void end_work();

  private:
    friend class Playback;
    explicit Producer(std::chrono::nanoseconds period);
    struct Impl;
    std::unique_ptr<Impl> impl_;
  };

  explicit Playback(unsigned rate, std::string_view name = "Virtual instrument");
  ~Playback();
  Playback(const Playback &) = delete;
  Playback &operator=(const Playback &) = delete;
  [[nodiscard]] bool write(std::span<const Frame> frames, const std::stop_token &stop);
  [[nodiscard]] bool reset(const std::stop_token &stop);
  void stop() noexcept;
  void check() const;
  [[nodiscard]] const Grants &grants() const noexcept;
  [[nodiscard]] PlaybackStatus status() const noexcept;
  [[nodiscard]] Producer producer() const;
  [[nodiscard]] bool fills_render_quantum(std::size_t pending) const noexcept;
#if defined(__APPLE__)
  [[nodiscard]] std::chrono::nanoseconds device_period() const noexcept;
#elif defined(__linux__)
  [[nodiscard]] bool pin_callback(const cpu_set_t &mask) noexcept;
  [[nodiscard]] bool callback_preempts(pthread_t producer) const noexcept;
  [[nodiscard]] std::uint64_t jack_xruns() const noexcept;
  // Frame zero starts the first live JACK cycle. The frontier starts the next cycle.
  [[nodiscard]] std::uint64_t midi_frontier() const noexcept;
  [[nodiscard]] std::optional<midi::Packet> read_midi(std::uint64_t before) noexcept;
  void write_midi(std::span<const std::uint8_t> message, std::uint64_t frame);
#endif

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
