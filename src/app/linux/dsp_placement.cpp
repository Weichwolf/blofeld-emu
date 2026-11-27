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

void DspPlacement::prepare(const PhysicalCore primary,
                           const PhysicalCore spare,
                           const PhysicalCore callback) {
  if (CPU_COUNT(&primary.siblings) >= 2) { cores_ = {primary, spare, callback}; }
}

void DspPlacement::attach() noexcept {
  const auto thread = ::pthread_self();
  if (CPU_COUNT(&cores_[callback_core].allowed) != 0 &&
      ::pthread_getcpuclockid(thread, &owner_clock_) == 0) {
    thread_.store(thread, std::memory_order_release);
  }
}

bool DspPlacement::move(host::audio::Playback &audio, const Move target) {
  if (pending_ != target.destination) {
    pending_ = target.destination;
    busy_samples_ = 0;
  }
  busy_samples_ = std::min(busy_samples_ + 1U, target.observations);
  const auto capacity = audio.grants().maximum_frames;
  if (busy_samples_ < target.observations ||
      audio.status().queued_frames < capacity - capacity / 4) {
    return false;
  }
  const auto &mask = cores_[target.destination].allowed;
  const auto thread = thread_.load(std::memory_order_relaxed);
  if (::pthread_setaffinity_np(thread, sizeof(mask), &mask) == 0) { current_ = target.destination; }
  busy_samples_ = 0;
  return true;
}

std::optional<std::uint64_t>
DspPlacement::runtime(const Runtimes &values, const cpu_set_t &mask, const bool total) const {
  std::uint64_t used = 0;
  for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
    if (!CPU_ISSET(cpu, &mask)) { continue; }
    const auto index = static_cast<std::size_t>(cpu);
    if (previous_[index] == 0 || values[index] < previous_[index]) { return std::nullopt; }
    const auto delta = values[index] - previous_[index];
    used = total ? used + delta : std::max(used, delta);
  }
  return used;
}

std::optional<DspPlacement::Move> DspPlacement::select(host::audio::Playback &audio,
                                                       const Runtimes &values,
                                                       const Clock::duration elapsed,
                                                       const std::uint64_t owner_elapsed) const {
  constexpr double busy_sibling = 0.45;
  constexpr double saturated_sibling = 0.75;
  constexpr double idle_destination = 0.30;
  constexpr double relative_destination = 0.60;
  const auto current_load = runtime(values, cores_[current_].siblings, true);
  if (!current_load) { return {}; }
  const auto wall_ns = std::chrono::duration<double, std::nano>(elapsed).count();
  const auto other = static_cast<double>(*current_load) - static_cast<double>(owner_elapsed);
  if (other < busy_sibling * wall_ns) { return {}; }
  const auto quiet = [&](const std::size_t index) {
    if (index == current_ || CPU_COUNT(&cores_[index].allowed) == 0) { return false; }
    const auto used = runtime(values, cores_[index].siblings, false);
    return used && static_cast<double>(*used) <= idle_destination * wall_ns;
  };
  const bool saturated = other >= saturated_sibling * wall_ns;
  const unsigned observations = saturated ? 1U : 3U;
  for (std::size_t index = 0; index < callback_core; ++index) {
    if (quiet(index)) { return Move{.destination = index, .observations = observations}; }
  }
  // Borrow the callback's core only under contention and with verified RT priority.
  if (quiet(callback_core) && audio.callback_preempts(thread_.load(std::memory_order_relaxed))) {
    return Move{.destination = callback_core, .observations = observations};
  }
  // A substantially quieter spare may still be useful when every core is busy.
  if (saturated) {
    for (std::size_t index = 0; index < callback_core; ++index) {
      if (index == current_ || CPU_COUNT(&cores_[index].allowed) == 0) { continue; }
      const auto used = runtime(values, cores_[index].siblings, true);
      if (used && static_cast<double>(*used) <= relative_destination * other) {
        return Move{.destination = index, .observations = 3U};
      }
    }
  }
  return {};
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
  constexpr auto minimum_interval = std::chrono::milliseconds(5);
  constexpr auto maximum_interval = std::chrono::milliseconds(40);
  const bool eligible = elapsed >= minimum_interval && elapsed <= maximum_interval &&
                        now >= next_move_ && owner_time >= previous_owner_;
  const auto target =
      eligible ? select(audio, values, elapsed, owner_time - previous_owner_) : std::nullopt;
  if (!target) {
    busy_samples_ = 0;
  } else if (move(audio, *target)) {
    next_move_ = now + std::chrono::seconds(1);
  }
  previous_ = values;
  previous_owner_ = owner_time;
  last_sample_ = now;
}
}
