// SPDX-License-Identifier: GPL-3.0-only
#include "preparation_clock.hpp"

#include <cassert>
#include <mutex>
#include <stdexcept>

namespace blofeld::vi {
PreparationClock::PreparationClock(const Source source)
    : source_(source != nullptr ? source
                                : throw std::invalid_argument("Missing preparation clock")),
      origin_(source_()) {}

PreparationClock::Clock::duration PreparationClock::elapsed() const {
  const std::scoped_lock lock(mutex_);
  const auto now = source_();
  const auto current = active_ != 0 ? now - begun_ : Clock::duration{};
  return now - origin_ - excluded_ - current;
}

void PreparationClock::begin() {
  const std::scoped_lock lock(mutex_);
  assert(active_ < 2);
  if (active_ == 0) { begun_ = source_(); }
  ++active_;
}

void PreparationClock::end() {
  const std::scoped_lock lock(mutex_);
  assert(active_ != 0);
  --active_;
  if (active_ == 0) { excluded_ += source_() - begun_; }
}

PreparationClock::Scope::Scope(PreparationClock *const clock) : clock_(clock) {
  if (clock_ != nullptr) { clock_->begin(); }
}

PreparationClock::Scope::~Scope() {
  if (clock_ != nullptr) { clock_->end(); }
}
}
