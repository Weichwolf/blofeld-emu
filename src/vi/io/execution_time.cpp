// SPDX-License-Identifier: GPL-3.0-only
#include "io/execution_time.hpp"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <stop_token>

namespace blofeld::vi {
namespace {
static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
static_assert(std::atomic<bool>::is_always_lock_free);

struct Waiting {
  std::atomic<bool> &flag;

  ~Waiting() { flag.store(false, std::memory_order_seq_cst); }
};
}

bool ExecutionTime::publish_mcu(const std::uint64_t ticks) noexcept {
  const auto before = mcu_.load(std::memory_order_seq_cst);
  if (ticks < before) { return false; }
  if (ticks == before) { return true; }
  // One MCU owner publishes progress. SC publication and wait registration
  // ensure either the waiter sees this value or the publisher wakes it.
  mcu_.store(ticks, std::memory_order_seq_cst);
  if (waiting_.load(std::memory_order_seq_cst)) {
    const std::lock_guard lock(mutex_);
    changed_.notify_all();
  }
  return true;
}

bool ExecutionTime::enter_mcu(const std::uint64_t ticks, const std::stop_token &stop) {
  std::unique_lock lock(mutex_);
  if (pending_ || ticks < mcu_.load(std::memory_order_seq_cst) || stop.stop_requested()) {
    return false;
  }
  mcu_.store(ticks, std::memory_order_seq_cst);
  pending_ = true;
  granted_ = held_;
  changed_.notify_all();
  if (!changed_.wait(lock, stop, [&] { return granted_ || held_; }) || stop.stop_requested()) {
    pending_ = granted_ = false;
    changed_.notify_all();
    return false;
  }
  return true;
}

void ExecutionTime::leave_mcu() noexcept {
  const std::lock_guard lock(mutex_);
  pending_ = granted_ = false;
  changed_.notify_all();
}

std::optional<std::uint64_t> ExecutionTime::dsp_limit(const std::uint64_t ticks,
                                                      const std::stop_token &stop) {
  std::unique_lock lock(mutex_);
  if (ticks < dsp_) {
    throw std::logic_error("DSP execution time moved backwards within its epoch");
  }
  dsp_ = ticks;
  while (!stop.stop_requested() && !held_) {
    const auto frontier = mcu_.load(std::memory_order_seq_cst);
    if (pending_ && ticks >= frontier) {
      granted_ = true;
      changed_.notify_all();
      if (!changed_.wait(lock, stop, [&] { return !pending_ || !granted_ || held_; })) {
        return {};
      }
    } else if (frontier > ticks) {
      return frontier;
    } else {
      waiting_.store(true, std::memory_order_seq_cst);
      const Waiting registered{waiting_};
      if (!changed_.wait(lock, stop, [&] {
            return pending_ || held_ || mcu_.load(std::memory_order_seq_cst) > ticks;
          })) {
        return {};
      }
    }
  }
  return {};
}

void ExecutionTime::hold_dsp() noexcept {
  const std::lock_guard lock(mutex_);
  held_ = true;
  changed_.notify_all();
}

void ExecutionTime::release_dsp(const std::uint64_t origin) noexcept {
  const std::lock_guard lock(mutex_);
  origin_ = dsp_ = origin;
  held_ = false;
  changed_.notify_all();
}

std::uint64_t ExecutionTime::dsp_origin() const noexcept {
  const std::lock_guard lock(mutex_);
  return origin_;
}
}
