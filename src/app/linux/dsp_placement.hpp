// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <ctime>
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
  void prepare(PhysicalCore primary, PhysicalCore spare);
  void attach() noexcept;
  void update(host::audio::Playback &audio);

private:
  using Clock = std::chrono::steady_clock;
  using Runtimes = std::array<std::uint64_t, CPU_SETSIZE>;
  static Runtimes runtimes();
  bool move(host::audio::Playback &audio);

  std::array<PhysicalCore, 2> cores_{};
  std::atomic<pthread_t> thread_;
  clockid_t owner_clock_{};
  std::uint64_t previous_owner_ = 0;
  Runtimes previous_{};
  Clock::time_point last_sample_, next_move_;
  unsigned current_ = 0, busy_samples_ = 0;
};
}
