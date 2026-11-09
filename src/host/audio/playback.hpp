// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/buffer.hpp"

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
  std::string sink, sink_format;
};

struct PlaybackStatus {
  std::uint64_t submitted_frames = 0, underruns = 0;
  std::size_t queued_frames = 0, server_buffer_frames = 0;
  std::optional<std::int64_t> latency_us;
  bool started = false, failed = false;
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

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
