// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "mcu/epoch.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace blofeld::vi::mcu {
class InputHistory {
public:
  using Values = std::array<std::uint64_t, static_cast<std::size_t>(Epoch::Signal::count)>;

  explicit InputHistory(const std::span<Epoch::Read> storage, const Values initial = {}) noexcept
      : storage_(storage), baseline_(initial), current_(initial) {}

  [[nodiscard]] bool write(Epoch::Read change) noexcept {
    if (error_ != Epoch::Error::none) { return false; }
    if (change.ticks < origin_ || change.ticks < source_ || change.signal >= Epoch::Signal::count) {
      return fail(Epoch::Error::time);
    }
    source_ = change.ticks;
    auto &current = current_[static_cast<std::size_t>(change.signal)];
    change.value = (current & ~change.mask) | (change.value & change.mask);
    if (change.value == current) { return true; }
    if (size_ == storage_.size()) { return fail(Epoch::Error::exhausted); }
    current = change.value;
    storage_[(first_ + size_) % storage_.size()] = change;
    ++size_;
    return true;
  }

  [[nodiscard]] std::optional<std::uint64_t> at(const std::uint64_t ticks,
                                                const Epoch::Signal signal) const noexcept {
    if (error_ != Epoch::Error::none || ticks < origin_ || signal >= Epoch::Signal::count) {
      return {};
    }
    const auto selected = static_cast<std::size_t>(signal);
    if (ticks >= source_) { return current_[selected]; }
    auto value = baseline_[selected];
    for (std::size_t index = 0; index < size_; ++index) {
      const auto &change = storage_[(first_ + index) % storage_.size()];
      if (change.ticks > ticks) { break; }
      if (change.signal == signal) { value = change.value; }
    }
    return value;
  }

  [[nodiscard]] bool retire(const std::uint64_t ticks) noexcept {
    if (error_ != Epoch::Error::none) { return false; }
    if (ticks < origin_) { return fail(Epoch::Error::time); }
    while (size_ != 0 && storage_[first_].ticks <= ticks) {
      const auto &change = storage_[first_];
      baseline_[static_cast<std::size_t>(change.signal)] = change.value;
      first_ = (first_ + 1) % storage_.size();
      --size_;
    }
    origin_ = ticks;
    return true;
  }

  [[nodiscard]] Epoch::Error error() const noexcept { return error_; }

private:
  bool fail(const Epoch::Error error) noexcept {
    if (error_ == Epoch::Error::none) { error_ = error; }
    return false;
  }

  std::span<Epoch::Read> storage_;
  Values baseline_, current_;
  std::size_t first_ = 0, size_ = 0;
  std::uint64_t origin_ = 0, source_ = 0;
  Epoch::Error error_ = Epoch::Error::none;
};
}
