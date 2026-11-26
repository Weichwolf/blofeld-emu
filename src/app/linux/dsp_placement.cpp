// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_placement.hpp"

#include "audio/playback.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <optional>
#include <pthread.h>
#include <ratio>
#include <sched.h>
#include <sstream>
#include <string>
// NOLINTNEXTLINE(modernize-deprecated-headers) POSIX declares clock_gettime here.
#include <time.h>

namespace blofeld::app {
DspPlacement::Runtimes DspPlacement::runtimes() {
  std::ifstream stat("/proc/schedstat");
  Runtimes values{};
  for (std::string line; std::getline(stat, line);) {
    if (!line.starts_with("cpu")) { continue; }
    std::istringstream fields(line.substr(3));
    int cpu = 0;
    std::array<std::uint64_t, 7> counters{};
    if (!(fields >> cpu) || cpu < 0 || cpu >= CPU_SETSIZE) { continue; }
    for (auto &counter : counters) { fields >> counter; }
    // The seventh CPU field is cumulative scheduler runtime in nanoseconds.
    if (fields) { values[static_cast<std::size_t>(cpu)] = counters.back(); }
  }
  return values;
}

void DspPlacement::prepare(const PhysicalCore primary, const PhysicalCore spare) {
  if (CPU_COUNT(&primary.siblings) >= 2 && CPU_COUNT(&spare.allowed) != 0) {
    cores_ = {primary, spare};
  }
}

void DspPlacement::attach() noexcept {
  const auto thread = ::pthread_self();
  if (CPU_COUNT(&cores_[1].allowed) != 0 && ::pthread_getcpuclockid(thread, &owner_clock_) == 0) {
    thread_.store(thread, std::memory_order_release);
  }
}

bool DspPlacement::move(host::audio::Playback &audio) {
  constexpr unsigned sustained_samples = 3;
  busy_samples_ = std::min(busy_samples_ + 1U, sustained_samples);
  const auto capacity = audio.grants().maximum_frames;
  if (busy_samples_ < sustained_samples || audio.status().queued_frames < capacity - capacity / 4) {
    return false;
  }
  const auto destination = current_ ^ 1U;
  const auto &mask = cores_[destination].allowed;
  const auto thread = thread_.load(std::memory_order_relaxed);
  if (::pthread_setaffinity_np(thread, sizeof(mask), &mask) == 0) { current_ = destination; }
  busy_samples_ = 0;
  return true;
}

void DspPlacement::update(host::audio::Playback &audio) {
  if (thread_.load(std::memory_order_acquire) == 0U) { return; }
  timespec owner{};
  if (::clock_gettime(owner_clock_, &owner) != 0) { return; }
  constexpr std::uint64_t ns_per_second = 1'000'000'000;
  const auto owner_time = (static_cast<std::uint64_t>(owner.tv_sec) * ns_per_second) +
                          static_cast<std::uint64_t>(owner.tv_nsec);
  const auto now = Clock::now();
  const auto elapsed = now - last_sample_;
  const auto values = runtimes();
  const auto load = [&](const cpu_set_t &mask, const bool total) -> std::optional<std::uint64_t> {
    std::uint64_t used = 0;
    for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
      if (!CPU_ISSET(cpu, &mask)) { continue; }
      const auto index = static_cast<std::size_t>(cpu);
      if (previous_[index] == 0 || values[index] < previous_[index]) { return std::nullopt; }
      const auto delta = values[index] - previous_[index];
      used = total ? used + delta : std::max(used, delta);
    }
    return used;
  };
  constexpr auto minimum_interval = std::chrono::milliseconds(5);
  constexpr auto maximum_interval = std::chrono::milliseconds(40);
  constexpr double busy_sibling = 0.45;
  constexpr double idle_destination = 0.30;
  const auto current_load = load(cores_[current_].siblings, true);
  const auto spare_load = load(cores_[current_ ^ 1U].siblings, false);
  const auto wall_ns = std::chrono::duration<double, std::nano>(elapsed).count();
  if (elapsed >= minimum_interval && elapsed <= maximum_interval && now >= next_move_ &&
      owner_time >= previous_owner_ && current_load && spare_load &&
      static_cast<double>(*current_load) - static_cast<double>(owner_time - previous_owner_) >=
          busy_sibling * wall_ns &&
      static_cast<double>(*spare_load) <= idle_destination * wall_ns) {
    if (move(audio)) { next_move_ = now + std::chrono::seconds(1); }
  } else {
    busy_samples_ = 0;
  }
  previous_ = values;
  previous_owner_ = owner_time;
  last_sample_ = now;
}
}
