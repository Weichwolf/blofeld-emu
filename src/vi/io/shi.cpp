// SPDX-License-Identifier: GPL-3.0-only
#include "io/shi.hpp"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>
#include <stop_token>

namespace blofeld::vi {
namespace {
constexpr std::uint32_t clock_address = 0xffff90;
constexpr std::uint32_t status_address = 0xffff91;
constexpr std::uint32_t transmit_address = 0xffff93;
constexpr std::uint32_t receive_address = 0xffff94;
constexpr std::uint32_t enabled = 1, i2c = 2, fifo_enabled = 0x20, master = 0x40;
constexpr std::uint32_t idle = 0x200, transmit_empty = 0x8000, underrun = 0x4000;
constexpr std::uint32_t receive_not_empty = 0x20000, receive_full = 0x80000;
constexpr std::uint32_t overrun = 0x100000, busy = 0x400000;
constexpr std::uint32_t control_mask = 0x3fff, clock_mask = 0x7ff, word_mask = 0xffffff;
constexpr std::uint32_t word_size_mask = 0xc;
constexpr unsigned word_bits = 24;
constexpr std::uint32_t receive_mode_mask = enabled | i2c | word_size_mask | fifo_enabled | master;

unsigned width(const std::uint32_t control) noexcept {
  return 8 * (((control & word_size_mask) >> 2U) + 1);
}
}

void Shi::publish() noexcept {
  const auto capacity = (control_ & fifo_enabled) != 0 ? fifo_.size() : 1;
  auto value = control_ | idle | errors_;
  if (!transmit_) { value |= transmit_empty; }
  if (count_ != 0) { value |= receive_not_empty; }
  if (count_ >= capacity) { value |= receive_full; }
  if (selected_ && (control_ & enabled) != 0) { value |= busy; }
  status_.store(value, std::memory_order_release);
}

void Shi::prime_transmit() noexcept {
  // CPHA=0 loads only while deselected; CPHA=1 can refill between words.
  if (!shifting_ && transmit_ && (control_ & enabled) != 0 && ((clock_ & 1) != 0 || !selected_)) {
    shifting_ = transmit_;
    transmit_.reset();
  }
}

void Shi::reset() noexcept {
  const std::lock_guard lock(mutex_);
  ++generation_;
  head_ = count_ = 0;
  control_ = errors_ = 0;
  // DSP56371UM 7.4.5: CPHA resets set; all other HCKR fields reset clear.
  clock_ = 1;
  observed_overrun_ = false;
  observed_underrun_ = phase_zero_complete_ = false;
  transmit_.reset();
  shifting_.reset();
  publish();
  ready_.notify_all();
}

bool Shi::read(const std::uint32_t address, std::uint32_t &value) noexcept {
  if (address == status_address) {
    value = status_.load(std::memory_order_acquire);
    if ((value & overrun) != 0) { observed_overrun_ = true; }
    if ((value & underrun) != 0) { observed_underrun_ = true; }
    return true;
  }
  const std::lock_guard lock(mutex_);
  if (address == clock_address) {
    value = clock_;
    return true;
  }
  if (address != receive_address || count_ == 0) { return false; }
  value = fifo_[head_];
  head_ = (head_ + 1) % fifo_.size();
  --count_;
  if (observed_overrun_) {
    errors_ &= ~overrun;
    observed_overrun_ = false;
  }
  publish();
  ready_.notify_one();
  return true;
}

bool Shi::write(const Write access) noexcept {
  const std::lock_guard lock(mutex_);
  if (access.address == clock_address) {
    const auto next = access.value & clock_mask;
    if (clock_ != next) { ++generation_; }
    clock_ = next;
    prime_transmit();
    publish();
    ready_.notify_all();
    return true;
  }
  if (access.address == transmit_address) {
    if ((control_ & enabled) == 0 || access.value > word_mask) { return false; }
    // An unpolled write overwrites HTX, not the word already in IOSR (7.4.2).
    transmit_ = access.value;
    if (observed_underrun_) {
      errors_ &= ~underrun;
      observed_underrun_ = false;
    }
    prime_transmit();
    publish();
    return true;
  }
  if (access.address != status_address) { return false; }
  const auto next = access.value & control_mask;
  if ((next & enabled) != 0 &&
      ((next & (i2c | master)) != 0 || (next & word_size_mask) == word_size_mask)) {
    return false;
  }
  if (((control_ ^ next) & receive_mode_mask) != 0) { ++generation_; }
  control_ = next;
  if ((control_ & enabled) == 0) {
    head_ = count_ = 0;
    errors_ = 0;
    observed_overrun_ = false;
    observed_underrun_ = phase_zero_complete_ = false;
    transmit_.reset();
    shifting_.reset();
  }
  publish();
  ready_.notify_all();
  return true;
}

std::optional<Shi::Mode> Shi::select(const bool active) noexcept {
  const std::lock_guard lock(mutex_);
  if (selected_ != active) { ++generation_; }
  selected_ = active;
  if (!active) { phase_zero_complete_ = false; }
  prime_transmit();
  publish();
  ready_.notify_all();
  return mode();
}

std::optional<Shi::Mode> Shi::mode() const noexcept {
  if (!selected_ || (control_ & enabled) == 0) { return {}; }
  // SPI samples the leading edge for CPHA=0, trailing for CPHA=1.
  const bool phase = (clock_ & 1) != 0;
  const bool polarity = (clock_ & 2) != 0;
  return Mode{.bits = width(control_), .falling = phase != polarity, .continuous = phase};
}

std::optional<Shi::Mode> Shi::wait_mode(const std::stop_token &stop) {
  std::unique_lock lock(mutex_);
  if (!ready_.wait(lock, stop, [&] { return mode().has_value(); }) || stop.stop_requested()) {
    return {};
  }
  return mode();
}

Shi::Transfer Shi::transfer(const std::uint32_t word) noexcept {
  const std::lock_guard lock(mutex_);
  return transfer_locked(word);
}

Shi::Transfer Shi::submit(const std::uint32_t word, const std::stop_token &stop) {
  std::unique_lock lock(mutex_);
  if (!valid_transfer(word) || stop.stop_requested()) { return {}; }
  const auto generation = generation_;
  const auto ready = [&] {
    const auto capacity = (control_ & fifo_enabled) != 0 ? fifo_.size() : 1;
    return generation_ != generation || !selected_ || (control_ & enabled) == 0 ||
           count_ < capacity;
  };
  if ((!ready() && !ready_.wait(lock, stop, ready)) || stop.stop_requested() ||
      generation_ != generation) {
    return {};
  }
  return transfer_locked(word);
}

bool Shi::valid_transfer(const std::uint32_t word) const noexcept {
  return selected_ && (control_ & enabled) != 0 && ((clock_ & 1) != 0 || !phase_zero_complete_) &&
         word <= (word_mask >> (word_bits - width(control_)));
}

Shi::Transfer Shi::transfer_locked(const std::uint32_t word) noexcept {
  if (!valid_transfer(word)) { return {}; }
  const auto padding = word_bits - width(control_);
  Transfer result{.accepted = true, .output = {}};
  if (shifting_) {
    result.output = *shifting_ >> padding;
    shifting_.reset();
  } else {
    errors_ |= underrun;
  }
  phase_zero_complete_ = (clock_ & 1) == 0;
  prime_transmit();
  const auto capacity = (control_ & fifo_enabled) != 0 ? fifo_.size() : 1;
  if (count_ >= capacity) {
    errors_ |= overrun;
  } else {
    fifo_[(head_ + count_) % fifo_.size()] = (word << padding) & word_mask;
    ++count_;
  }
  publish();
  return result;
}
}
