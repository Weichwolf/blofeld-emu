// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <chrono>
#include <mutex>

namespace blofeld::vi {
class PreparationClock {
public:
  using Clock = std::chrono::steady_clock;
  using Source = Clock::time_point (*)() noexcept;

  explicit PreparationClock(Source source = Clock::now);
  [[nodiscard]] Clock::duration elapsed() const;

  class Scope {
  public:
    explicit Scope(PreparationClock *clock);
    ~Scope();
    Scope(const Scope &) = delete;
    Scope &operator=(const Scope &) = delete;

  private:
    PreparationClock *clock_;
  };

private:
  void begin();
  void end();
  Source source_;
  const Clock::time_point origin_;
  mutable std::mutex mutex_;
  Clock::time_point begun_;
  Clock::duration excluded_{};
  unsigned active_ = 0;
};
}
