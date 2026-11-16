// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <utility>

namespace blofeld::vi::mcu {
class Epoch {
public:
  enum class Signal : std::uint8_t { data, interrupt, mode, reply, count };
  enum class Error : std::uint8_t { none, exhausted, time, phase, unsettled };
  enum class Phase : std::uint8_t { speculative, invalidated, atomic };
  enum class Refusal : std::uint8_t { device, epoch_invalidated };

  struct Read {
    std::uint64_t ticks = 0, value = 0, mask = 0;
    Signal signal = Signal::data;
  };

  struct Census {
    std::array<std::uint64_t, static_cast<std::size_t>(Signal::count)> reads{}, failed{};
    std::uint64_t rollbacks = 0, destructive = 0;
  };

  explicit Epoch(const std::span<Read> storage, const std::size_t reserve) noexcept
      : storage_(storage), reserve_(reserve) {
    if (storage.size() <= reserve) { error_ = Error::exhausted; }
  }

  [[nodiscard]] bool record(const Read read) noexcept {
    if (error_ != Error::none) { return false; }
    if (phase_ == Phase::invalidated) { return refuse(); }
    if (read.ticks < position_ || read.signal >= Signal::count) { return fail(Error::time); }
    position_ = read.ticks;
    if (phase_ == Phase::atomic && !settled(read.ticks)) { return fail(Error::unsettled); }
#if defined(BLOFELD_CENSUS)
    ++census_.reads[static_cast<std::size_t>(read.signal)];
#endif
    if (settled(read.ticks)) { return true; }
    if (size_ == storage_.size()) { return fail(Error::exhausted); }
    storage_[(first_ + size_) % storage_.size()] = read;
    ++size_;
    return true;
  }

  [[nodiscard]] bool changed(const Read change) noexcept {
    if (error_ != Error::none) { return false; }
    if (change.ticks < attained_ || change.ticks < source_ || change.signal >= Signal::count) {
      return fail(Error::time);
    }
    source_ = change.ticks;
    for (std::size_t index = 0; index < size_; ++index) {
      const auto &read = storage_[(first_ + index) % storage_.size()];
      if (read.ticks < change.ticks || read.signal != change.signal ||
          ((read.value ^ change.value) & read.mask & change.mask) == 0) {
        continue;
      }
      if (phase_ != Phase::invalidated) {
        conflict_ = change.ticks;
#if defined(BLOFELD_CENSUS)
        ++census_.failed[static_cast<std::size_t>(read.signal)];
#endif
      }
      phase_ = Phase::invalidated;
      return false;
    }
    return phase_ != Phase::invalidated;
  }

  [[nodiscard]] bool attained(const std::uint64_t exclusive) noexcept {
    if (error_ != Error::none || phase_ == Phase::invalidated) { return false; }
    if (exclusive < attained_ || exclusive < source_) { return fail(Error::time); }
    attained_ = exclusive;
    while (size_ != 0 && storage_[first_].ticks < attained_) {
      first_ = (first_ + 1) % storage_.size();
      --size_;
    }
    return true;
  }

  template <typename Action>
  [[nodiscard]] bool consume(const std::uint64_t ticks, Action &&action) noexcept {
    if (error_ != Error::none) { return false; }
    if (phase_ == Phase::invalidated) { return refuse(); }
    if (ticks < position_) { return fail(Error::time); }
    if (!settled(ticks)) { return fail(Error::unsettled); }
    position_ = ticks;
    phase_ = Phase::atomic;
    if (!std::forward<Action>(action)()) { return false; }
#if defined(BLOFELD_CENSUS)
    ++census_.destructive;
#endif
    return true;
  }

  [[nodiscard]] bool checkpoint(const std::uint64_t ticks) noexcept {
    if (error_ != Error::none || phase_ == Phase::invalidated) { return false; }
    if (ticks < position_ || !settled(ticks) || size_ != 0) { return fail(Error::unsettled); }
    checkpoint_ = position_ = ticks;
    phase_ = Phase::speculative;
    refusal_ = Refusal::device;
    return true;
  }

  [[nodiscard]] bool rollback() noexcept {
    if (error_ != Error::none || phase_ != Phase::invalidated) { return fail(Error::phase); }
    first_ = size_ = 0;
    position_ = checkpoint_;
    phase_ = Phase::speculative;
    refusal_ = Refusal::device;
    conflict_.reset();
#if defined(BLOFELD_CENSUS)
    ++census_.rollbacks;
#endif
    return true;
  }

  [[nodiscard]] bool yield() const noexcept { return storage_.size() - size_ <= reserve_; }

  [[nodiscard]] bool cancelled() const noexcept {
    return phase_ == Phase::invalidated && error_ == Error::none;
  }

  [[nodiscard]] bool refuse() noexcept {
    refusal_ = cancelled() ? Refusal::epoch_invalidated : Refusal::device;
    return false;
  }

  [[nodiscard]] Refusal refusal() const noexcept {
    return cancelled() ? refusal_ : Refusal::device;
  }

  [[nodiscard]] bool atomic() const noexcept { return phase_ == Phase::atomic; }

  [[nodiscard]] Error error() const noexcept { return error_; }

  [[nodiscard]] std::optional<std::uint64_t> conflict() const noexcept { return conflict_; }

  [[nodiscard]] const Census &census() const noexcept { return census_; }

private:
  [[nodiscard]] bool settled(const std::uint64_t ticks) const noexcept { return ticks < attained_; }

  bool fail(const Error error) noexcept {
    if (error_ == Error::none) { error_ = error; }
    return false;
  }

  std::span<Read> storage_;
  std::size_t reserve_, first_ = 0, size_ = 0;
  std::uint64_t checkpoint_ = 0, position_ = 0, attained_ = 0, source_ = 0;
  Phase phase_ = Phase::speculative;
  Error error_ = Error::none;
  Refusal refusal_ = Refusal::device;
  std::optional<std::uint64_t> conflict_;
  Census census_;
};
}
