// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <optional>
#include <pthread.h>
#include <sched.h>

namespace blofeld::host::audio {
class Playback;
}

namespace blofeld::app {
struct PhysicalCore {
  cpu_set_t allowed, siblings;
};

class DspPlacement {
public:
  void prepare(PhysicalCore primary, PhysicalCore spare, PhysicalCore callback);
  void attach() noexcept;
  void update(host::audio::Playback &audio);

private:
  using Clock = std::chrono::steady_clock;
  using Runtimes = std::array<std::uint64_t, CPU_SETSIZE>;

  struct Move {
    std::size_t destination;
    unsigned observations;
  };

  static constexpr std::size_t callback_core = 2;
  static Runtimes runtimes();
  [[nodiscard]] std::optional<std::uint64_t> runtime(const Runtimes &values,
                                                     const cpu_set_t &mask) const;
  [[nodiscard]] std::optional<Move> select(host::audio::Playback &audio,
                                           const Runtimes &values,
                                           Clock::duration elapsed,
                                           std::uint64_t owner_elapsed) const;
  bool move(host::audio::Playback &audio, Move target);

  std::array<PhysicalCore, callback_core + 1> cores_{};
  std::atomic<pthread_t> thread_;
  clockid_t owner_clock_{};
  std::uint64_t previous_owner_ = 0;
  Runtimes previous_{};
  Clock::time_point last_sample_, next_move_;
  std::size_t current_ = 0, pending_ = 0;
  unsigned busy_samples_ = 0;
};
}
