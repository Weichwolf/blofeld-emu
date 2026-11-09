// SPDX-License-Identifier: GPL-3.0-only
#include "io/execution_time.hpp"

#include <atomic>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <stop_token>

namespace blofeld::vi {
namespace {
static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
static_assert(std::atomic<bool>::is_always_lock_free);
constexpr std::uint64_t phase_mask = 3, requested = 1, granted = 2, next_generation = 4;

struct Waiting {
  std::atomic<bool> &flag;

  explicit Waiting(std::atomic<bool> &value) : flag(value) {
    flag.store(true, std::memory_order_seq_cst);
  }

  ~Waiting() { flag.store(false, std::memory_order_seq_cst); }
};

void relax() noexcept {
#if defined(__x86_64__)
  __builtin_ia32_pause();
#elif defined(__aarch64__)
  asm volatile("yield");
#else
  std::atomic_signal_fence(std::memory_order_seq_cst);
#endif
}
}

void ExecutionTime::signal() noexcept {
  changed_.fetch_add(1, std::memory_order_seq_cst);
  changed_.notify_all();
}

void ExecutionTime::notify() noexcept {
  if (waiting_mcu_.load(std::memory_order_seq_cst) ||
      waiting_dsp_.load(std::memory_order_seq_cst)) {
    signal();
  }
}

template <typename Ready>
bool ExecutionTime::await(std::atomic<bool> &waiting, const std::stop_token &stop, Ready ready) {
  constexpr unsigned spin_attempts = 64;
  for (unsigned attempt = 0; attempt < spin_attempts; ++attempt) {
    if (stop.stop_requested()) { return false; }
    if (ready()) { return true; }
    relax();
  }
  // SC registration + predicate recheck: either observe a publication or wake.
  const Waiting registered{waiting};
  const std::stop_callback cancelled(stop, [this] { signal(); });
  for (;;) {
    const auto observed = changed_.load(std::memory_order_seq_cst);
    if (stop.stop_requested()) { return false; }
    if (ready()) { return true; }
    changed_.wait(observed, std::memory_order_acquire);
  }
}

bool ExecutionTime::publish_mcu(const std::uint64_t ticks) noexcept {
  const auto before = mcu_.load(std::memory_order_seq_cst);
  if (ticks < before) { return false; }
  if (ticks == before) { return true; }
  mcu_.store(ticks, std::memory_order_seq_cst);
  notify();
  return true;
}

bool ExecutionTime::enter_mcu(const std::uint64_t ticks, const std::stop_token &stop) {
  const auto previous = transaction_.load(std::memory_order_seq_cst);
  if ((previous & phase_mask) != 0 || ticks < mcu_.load(std::memory_order_seq_cst) ||
      previous > std::numeric_limits<std::uint64_t>::max() - next_generation ||
      stop.stop_requested()) {
    return false;
  }
  mcu_.store(ticks, std::memory_order_seq_cst);
  // Generations distinguish consecutive transactions even at the same tick.
  const auto generation = previous + next_generation;
  const auto request = generation | requested;
  transaction_.store(request, std::memory_order_seq_cst);
  notify();
  if (!await(waiting_mcu_, stop, [&] {
        return held_.load(std::memory_order_seq_cst) ||
               transaction_.load(std::memory_order_seq_cst) == (generation | granted);
      })) {
    leave_mcu();
    return false;
  }
  return true;
}

void ExecutionTime::leave_mcu() noexcept {
  const auto current = transaction_.load(std::memory_order_seq_cst);
  transaction_.store(current & ~phase_mask, std::memory_order_seq_cst);
  notify();
}

std::optional<std::uint64_t> ExecutionTime::dsp_limit(const std::uint64_t ticks,
                                                      const std::stop_token &stop) {
  if (ticks < dsp_.load(std::memory_order_seq_cst)) {
    throw std::logic_error("DSP execution time moved backwards within its epoch");
  }
  dsp_.store(ticks, std::memory_order_seq_cst);
  while (!stop.stop_requested() && !held_.load(std::memory_order_seq_cst)) {
    auto transaction = transaction_.load(std::memory_order_seq_cst);
    const auto frontier = mcu_.load(std::memory_order_seq_cst);
    if ((transaction & phase_mask) != 0 && ticks >= frontier) {
      if ((transaction & phase_mask) == requested) {
        const auto grant = (transaction & ~phase_mask) | granted;
        // A stale observation must never grant a newer MCU transaction.
        if (!transaction_.compare_exchange_strong(transaction, grant, std::memory_order_seq_cst)) {
          continue;
        }
        transaction = grant;
        notify();
      }
      if (!await(waiting_dsp_, stop, [&] {
            return transaction_.load(std::memory_order_seq_cst) != transaction ||
                   held_.load(std::memory_order_seq_cst);
          })) {
        return {};
      }
    } else if (frontier > ticks) {
      return frontier;
    } else if (!await(waiting_dsp_, stop, [&] {
                 return (transaction_.load(std::memory_order_seq_cst) & phase_mask) != 0 ||
                        held_.load(std::memory_order_seq_cst) ||
                        mcu_.load(std::memory_order_seq_cst) > ticks;
               })) {
      return {};
    }
  }
  return {};
}

void ExecutionTime::hold_dsp() noexcept {
  held_.store(true, std::memory_order_seq_cst);
  notify();
}

void ExecutionTime::release_dsp(const std::uint64_t origin) noexcept {
  origin_.store(origin, std::memory_order_seq_cst);
  dsp_.store(origin, std::memory_order_seq_cst);
  held_.store(false, std::memory_order_seq_cst);
  notify();
}

std::uint64_t ExecutionTime::dsp_origin() const noexcept {
  return origin_.load(std::memory_order_seq_cst);
}
}
