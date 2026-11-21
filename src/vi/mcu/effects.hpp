// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace blofeld::vi::mcu {
class Effects {
public:
  enum class Kind : std::uint8_t {
    data,
    strobe,
    select,
    word,
    midi,
    reset_hold,
    reset_release,
    wait,
    acknowledge
  };
  enum class Error : std::uint8_t { none, exhausted, time, prefix, phase, sink };

  struct Entry {
    std::uint64_t ticks = 0, value = 0;
    Kind kind = Kind::data;
    bool operator==(const Entry &) const = default;
  };

  Effects(const std::span<Entry> storage, const std::size_t reserve) noexcept
      : storage_(storage), reserve_(reserve) {
    if (storage.size() <= reserve) { error_ = Error::exhausted; }
  }

  struct Diagnostic {
    Entry expected, actual;
    std::uint64_t attained;
    std::size_t written, applied, replayed;
  };

  [[nodiscard]] Diagnostic diagnostic() const noexcept {
    return {.expected = expected_,
            .actual = refused_,
            .attained = attained_,
            .written = written_,
            .applied = applied_,
            .replayed = replayed_};
  }

  [[nodiscard]] bool append(const Entry entry) noexcept {
    if (error_ != Error::none) { return false; }
    if (entry.ticks < position_) { return fail(Error::time); }
    position_ = entry.ticks;
    if (replaying_) {
      if (storage_[replayed_] != entry) {
        expected_ = storage_[replayed_];
        refused_ = entry;
        return fail(Error::prefix);
      }
      ++replayed_;
      replaying_ = replayed_ != applied_;
      return true;
    }
    if (entry.ticks < attained_) {
      refused_ = entry;
      return fail(Error::prefix);
    }
    if (written_ == storage_.size()) { return fail(Error::exhausted); }
    storage_[written_++] = entry;
    return true;
  }

  template <typename Sink>
  [[nodiscard]] bool apply_before(const std::uint64_t exclusive, Sink &&sink) {
    if (error_ != Error::none) { return false; }
    if (replaying_) { return fail(Error::phase); }
    if (exclusive < attained_) { return fail(Error::time); }
    while (applied_ != written_ && storage_[applied_].ticks < exclusive) {
      if (!sink(storage_[applied_])) { return fail(Error::sink); }
      ++applied_;
    }
    attained_ = exclusive;
    return true;
  }

  [[nodiscard]] std::span<const Entry> entries() const noexcept { return storage_.first(written_); }

  [[nodiscard]] bool acknowledge(const std::size_t count, const std::uint64_t exclusive) noexcept {
    if (error_ != Error::none) { return false; }
    if (count < applied_ || count > written_ || exclusive < attained_) {
      return fail(Error::phase);
    }
    for (auto index = applied_; index < count; ++index) {
      if (storage_[index].ticks >= exclusive) { return fail(Error::time); }
    }
    applied_ = count;
    attained_ = exclusive;
    return true;
  }

  struct Promotion {
    std::uint64_t ticks, known;
    std::size_t position, received;
  };

  [[nodiscard]] bool promote(const Promotion promotion) noexcept {
    const auto [ticks, known, position, received] = promotion;
    if (error_ != Error::none) { return false; }
    if (replaying_ || position < first_ || position > received || received > written_) {
      return fail(Error::phase);
    }
    if (ticks < origin_ || ticks >= known ||
        (position != first_ && storage_[position - 1].ticks > ticks) ||
        (position != written_ && storage_[position].ticks < ticks)) {
      return fail(Error::time);
    }
    first_ = replayed_ = position;
    applied_ = applied_ < position ? position : applied_;
    origin_ = ticks;
    position_ = position_ < ticks ? ticks : position_;
    return true;
  }

  [[nodiscard]] bool rollback() noexcept {
    if (error_ != Error::none) { return false; }
    if (replaying_) { return fail(Error::phase); }
    written_ = applied_;
    replayed_ = first_;
    position_ = origin_;
    replaying_ = applied_ != first_;
    return true;
  }

  [[nodiscard]] bool finish_replay() noexcept {
    if (error_ != Error::none) { return false; }
    return !replaying_ || fail(Error::prefix);
  }

  [[nodiscard]] bool checkpoint(const std::uint64_t ticks) noexcept {
    if (error_ != Error::none) { return false; }
    if (replaying_ || applied_ != written_) { return fail(Error::phase); }
    if (ticks < position_ || ticks >= attained_) { return fail(Error::time); }
    origin_ = position_ = ticks;
    attained_ = ticks;
    first_ = written_ = applied_ = replayed_ = 0;
    return true;
  }

  [[nodiscard]] bool yield() const noexcept { return storage_.size() - written_ <= reserve_; }

  [[nodiscard]] Error error() const noexcept { return error_; }

  [[nodiscard]] std::size_t size() const noexcept { return written_; }

private:
  friend class Output;

  bool fail(const Error error) noexcept {
    if (error_ == Error::none) { error_ = error; }
    return false;
  }

  Entry expected_{}, refused_{};
  std::span<Entry> storage_;
  std::size_t reserve_, first_ = 0, written_ = 0, applied_ = 0, replayed_ = 0;
  std::uint64_t origin_ = 0, position_ = 0, attained_ = 0;
  Error error_ = Error::none;
  bool replaying_ = false;
};
}
