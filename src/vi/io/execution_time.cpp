// SPDX-License-Identifier: GPL-3.0-only
#include "io/execution_time.hpp"

#if defined(BLOFELD_CENSUS)
#include "census.hpp"
#endif

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <format>
#include <limits>
#include <optional>
#include <stdexcept>
#include <stop_token>

namespace blofeld::vi {
namespace {
static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
static_assert(std::atomic<bool>::is_always_lock_free);
constexpr std::uint64_t phase_mask = 3, requested = 1, granted = 2, next_generation = 4;
constexpr std::uint64_t window_mask = 7, window_stride = 8, running = 3, forecasted = 4;

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

void ExecutionTime::Cancel::operator()() const noexcept {
  owner->signal();
}

void ExecutionTime::Cancellation::bind(const std::stop_token &stop, ExecutionTime &owner) {
  if (token == stop) { return; }
  callback.reset();
  token = stop;
  if (token.stop_possible()) { callback.emplace(token, Cancel{.owner = &owner}); }
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
  auto &cancellation = &waiting == &waiting_mcu_ ? mcu_cancellation_ : dsp_cancellation_;
  cancellation.bind(stop, *this);
  for (;;) {
    const auto observed = changed_.load(std::memory_order_seq_cst);
    if (stop.stop_requested()) { return false; }
    if (ready()) { return true; }
#if defined(BLOFELD_CENSUS)
    const Census::Wait observed_wait;
#endif
    changed_.wait(observed, std::memory_order_acquire);
  }
}

void ExecutionTime::publish_mcu_wait(const std::uint64_t ticks) noexcept {
  if (waited_.load(std::memory_order_relaxed)) { return; }
  first_wait_.store(ticks, std::memory_order_relaxed);
  waited_.store(true, std::memory_order_release);
}

std::optional<std::uint64_t> ExecutionTime::mcu_wait_time() const noexcept {
  if (!waited_.load(std::memory_order_acquire)) { return {}; }
  return first_wait_.load(std::memory_order_relaxed);
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

bool ExecutionTime::wait_for_dsp(const std::uint64_t ticks, const std::stop_token &stop) {
  if (!publish_mcu(ticks)) {
    throw std::logic_error("MCU interrupt admission time moved backwards");
  }
  return await(waiting_mcu_, stop, [&] {
    return held_.load(std::memory_order_seq_cst) || dsp_.load(std::memory_order_seq_cst) >= ticks;
  });
}

bool ExecutionTime::settle_dsp(const std::uint64_t ticks, const std::stop_token &stop) {
  if (ticks == std::numeric_limits<std::uint64_t>::max() ||
      (idle_.load(std::memory_order_seq_cst) & window_mask) != 0 ||
      (transaction_.load(std::memory_order_seq_cst) & phase_mask) != 0 ||
      settle_until_.load(std::memory_order_seq_cst) != 0 || stop.stop_requested()) {
    return false;
  }
  if (!publish_mcu(ticks)) { return false; }
  const auto settled = [&] {
    return held_.load(std::memory_order_seq_cst) ||
           dsp_known_.load(std::memory_order_seq_cst) > ticks;
  };
  if (settled()) { return true; }
  settle_until_.store(ticks + 1, std::memory_order_seq_cst);
  notify();
  const auto ready = await(waiting_mcu_, stop, settled);
  settle_until_.store(0, std::memory_order_seq_cst);
  notify();
  return ready;
}

std::optional<std::uint64_t> ExecutionTime::idle_mcu(const std::uint64_t now,
                                                     const std::uint64_t until,
                                                     const std::stop_token &stop,
                                                     const McuWork work) {
  const auto previous = idle_.load(std::memory_order_seq_cst);
  const auto completed_mcu = mcu_.load(std::memory_order_seq_cst);
  const auto held = held_.load(std::memory_order_seq_cst);
  const auto completed_dsp = dsp_.load(std::memory_order_seq_cst);
  if ((previous & window_mask) != 0 || until <= now ||
      previous > std::numeric_limits<std::uint64_t>::max() - window_stride ||
      now != completed_mcu || (!held && completed_dsp < now)) {
    throw std::logic_error(
        std::format("Invalid MCU idle lease: now={} until={} MCU={} DSP={} held={} generation={}",
                    now,
                    until,
                    completed_mcu,
                    completed_dsp,
                    held,
                    previous));
  }
  if (stop.stop_requested()) { return {}; }
  const auto generation = previous + window_stride;
  const auto request = generation | (work == McuWork::running ? running : requested);
  idle_until_.store(until, std::memory_order_seq_cst);
  idle_.store(request, std::memory_order_seq_cst);
  notify();
  const auto ready = await(waiting_mcu_, stop, [&] {
    return idle_.load(std::memory_order_seq_cst) != request ||
           held_.load(std::memory_order_seq_cst) || dsp_.load(std::memory_order_seq_cst) >= until;
  });
  const auto state = idle_.load(std::memory_order_seq_cst);
  const bool signalled = state == (generation | granted) || state == (generation | forecasted);
  const auto result = signalled ? wake_at_.load(std::memory_order_seq_cst) : until;
  idle_.store(generation, std::memory_order_seq_cst);
  notify();
  if (!ready) { return {}; }
  if (result < now || result > until) {
    throw std::logic_error("DSP wake lies outside its MCU idle lease");
  }
  return result;
}

bool ExecutionTime::wake_mcu(const std::uint64_t ticks) noexcept {
  auto request = idle_.load(std::memory_order_seq_cst);
  const auto phase = request & window_mask;
  if ((phase != requested && phase != running) ||
      ticks >= idle_until_.load(std::memory_order_seq_cst)) {
    return false;
  }
  wake_at_.store(ticks, std::memory_order_seq_cst);
  const auto wake = (request & ~window_mask) | granted;
  if (!idle_.compare_exchange_strong(request, wake, std::memory_order_seq_cst)) { return false; }
  notify();
  return true;
}

std::optional<ExecutionTime::IdleWindow> ExecutionTime::idle_window() const noexcept {
  const auto state = idle_.load(std::memory_order_seq_cst);
  const auto phase = state & window_mask;
  if (phase != requested && phase != running) { return {}; }
  const auto until = idle_until_.load(std::memory_order_seq_cst);
  if (idle_.load(std::memory_order_seq_cst) != state) { return {}; }
  return IdleWindow{.until = until, .generation = state};
}

bool ExecutionTime::publish_dsp_knowledge(const std::uint64_t until) noexcept {
  if (until <= dsp_known_.load(std::memory_order_seq_cst)) { return false; }
  dsp_known_.store(until, std::memory_order_seq_cst);
  return true;
}

void ExecutionTime::publish_dsp(const std::uint64_t ticks, const DspBoundary boundary) {
  const bool settled = boundary != DspBoundary::exact;
  if (settled && ticks == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("DSP current-instant knowledge exceeds the instrument domain");
  }
  // Same-tick input ends quiescence after the receiver's atomic operation.
  const auto previous = dsp_.load(std::memory_order_seq_cst);
  if (ticks < previous) {
    throw std::logic_error("DSP execution time moved backwards within its epoch");
  }
  if (ticks != previous) { dsp_.store(ticks, std::memory_order_seq_cst); }
  const bool extended = publish_dsp_knowledge(ticks + unsigned{settled});
  if (ticks != previous || extended) { notify(); }
}

std::optional<ExecutionTime::DspGrant> ExecutionTime::forecast_grant(const IdleWindow window,
                                                                     const std::uint64_t ticks,
                                                                     const DspForecast forecast,
                                                                     const std::stop_token &stop) {
  if (forecast.query == nullptr || (window.generation & window_mask) != running) {
    return DspGrant{.until = window.until};
  }
  const auto until = std::min(window.until, forecast.query(forecast.owner));
  if (stop.stop_requested() || held_.load(std::memory_order_seq_cst)) { return {}; }
  if (idle_.load(std::memory_order_seq_cst) != window.generation) { return {}; }
  if (until <= ticks) { return DspGrant{.until = window.until}; }
  wake_at_.store(until, std::memory_order_seq_cst);
  auto expected = window.generation;
  const auto offered = (expected & ~window_mask) | forecasted;
  if (!idle_.compare_exchange_strong(expected, offered, std::memory_order_seq_cst)) { return {}; }
  static_cast<void>(publish_dsp_knowledge(until));
  notify();
  return DspGrant{.until = until};
}

std::optional<ExecutionTime::DspGrant> ExecutionTime::execution_grant(const std::uint64_t ticks,
                                                                      const DspBoundary boundary,
                                                                      const DspForecast forecast,
                                                                      const std::stop_token &stop) {
  const auto frontier = mcu_.load(std::memory_order_seq_cst);
  if (frontier > ticks) { return DspGrant{.until = frontier}; }
  const auto through = settle_until_.load(std::memory_order_seq_cst);
  if (boundary == DspBoundary::exact && through > ticks) {
    return DspGrant{.until = through, .kind = DspGrant::Kind::current};
  }
  const auto idle = idle_window();
  return idle && idle->until > ticks ? forecast_grant(*idle, ticks, forecast, stop) : std::nullopt;
}

bool ExecutionTime::dsp_ready(const std::uint64_t ticks,
                              const DspBoundary boundary) const noexcept {
  const auto idle = idle_window();
  return (transaction_.load(std::memory_order_seq_cst) & phase_mask) != 0 ||
         held_.load(std::memory_order_seq_cst) || mcu_.load(std::memory_order_seq_cst) > ticks ||
         (boundary == DspBoundary::exact &&
          settle_until_.load(std::memory_order_seq_cst) > ticks) ||
         (idle && idle->until > ticks);
}

std::optional<ExecutionTime::DspGrant> ExecutionTime::dsp_limit(const std::uint64_t ticks,
                                                                const std::stop_token &stop,
                                                                const DspBoundary boundary,
                                                                const DspForecast forecast) {
  publish_dsp(ticks, boundary);
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
    } else if (const auto grant = execution_grant(ticks, boundary, forecast, stop)) {
      return grant;
    } else if (!await(waiting_dsp_, stop, [&] { return dsp_ready(ticks, boundary); })) {
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
  dsp_known_.store(origin, std::memory_order_seq_cst);
  held_.store(false, std::memory_order_seq_cst);
  notify();
}

std::uint64_t ExecutionTime::dsp_origin() const noexcept {
  return origin_.load(std::memory_order_seq_cst);
}
}
