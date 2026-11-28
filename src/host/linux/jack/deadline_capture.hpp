// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "process/configuration.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace blofeld::host::audio {
class DeadlineCapture {
public:
  static std::unique_ptr<DeadlineCapture> open() {
    const auto path = inherited_setting("BLOFELD_JACK_TRACE");
    return path.empty() ? nullptr : std::make_unique<DeadlineCapture>(std::string(path));
  }

  explicit DeadlineCapture(const std::string &path)
      : file_(path, std::ios::out | std::ios::noreplace) {
    if (!file_) { throw std::runtime_error("Cannot create the JACK deadline capture"); }
  }

  [[nodiscard]] std::uint64_t start() const noexcept {
    return required_.load(std::memory_order_relaxed) == 0 ? clock(CLOCK_MONOTONIC) : 0;
  }

  void missed(const std::uint32_t frame,
              const std::uint64_t tail,
              const std::uint64_t debt,
              const std::size_t missing,
              const std::uint64_t before) noexcept {
    if (before == 0) { return; }
    const auto after = clock(CLOCK_MONOTONIC);
    miss_ = {.frame = frame,
             .tail = tail,
             .needed = tail + debt,
             .missing = missing,
             .before = before,
             .after = after};
    required_.store(miss_.needed, std::memory_order_release);
  }

  void published(const std::size_t count) noexcept {
    if (count == 0 || frozen_) { return; }
    const auto now = clock(CLOCK_MONOTONIC);
    const auto cpu = clock(CLOCK_THREAD_CPUTIME_ID);
    if (tid_ == 0) { tid_ = ::gettid(); }
    head_ += count;
    publications_[written_ % publications_.size()] = {.head = head_, .now = now, .cpu = cpu};
    ++written_;
    const auto needed = required_.load(std::memory_order_acquire);
    frozen_ = needed != 0 && head_ >= needed;
  }

  // Both audio owners are stopped before this report is written.
  void save(const std::uint64_t xruns) noexcept {
    try {
      file_ << "JACK-DEADLINE 1 " << tid_ << ' ' << xruns << '\n';
      if (miss_.needed != 0) {
        file_ << "C " << miss_.frame << ' ' << miss_.tail << ' ' << miss_.needed << ' '
              << miss_.before << ' ' << miss_.after << ' ' << miss_.missing << '\n';
      }
      const auto kept = std::min(written_, publications_.size());
      for (auto i = written_ - kept; i < written_; ++i) {
        const auto &entry = publications_[i % publications_.size()];
        file_ << "P " << entry.head << ' ' << entry.now << ' ' << entry.cpu << '\n';
      }
      file_.flush();
      if (file_) { return; }
    } catch (...) {
      static_cast<void>(std::fputs("Could not write the JACK deadline capture\n", stderr));
      return;
    }
    static_cast<void>(std::fputs("Could not write the complete JACK deadline capture\n", stderr));
  }

private:
  static std::uint64_t clock(const clockid_t id) noexcept {
    timespec ts{};
    if (::clock_gettime(id, &ts) != 0) { return 0; }
    constexpr auto ns_per_second = std::chrono::nanoseconds::period::den;
    return (static_cast<std::uint64_t>(ts.tv_sec) * ns_per_second) +
           static_cast<std::uint64_t>(ts.tv_nsec);
  }

  struct Publication {
    std::uint64_t head, now, cpu;
  };

  struct Miss {
    std::uint32_t frame = 0;
    std::uint64_t tail = 0, needed = 0;
    std::size_t missing = 0;
    std::uint64_t before = 0, after = 0;
  };

  static constexpr std::size_t history = 4096;
  static_assert(std::atomic<std::uint64_t>::is_always_lock_free);

  std::ofstream file_;
  std::atomic<std::uint64_t> required_{0};
  Miss miss_;
  std::array<Publication, history> publications_{};
  std::size_t written_ = 0;
  std::uint64_t head_ = 0;
  pid_t tid_ = 0;
  bool frozen_ = false;
};
}
