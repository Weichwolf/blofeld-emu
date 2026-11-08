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
constexpr std::uint32_t receive_address = 0xffff94;
constexpr std::uint32_t enabled = 1, i2c = 2, fifo_enabled = 0x20, master = 0x40;
constexpr std::uint32_t idle = 0x200, transmit_empty = 0x8000, underrun = 0x4000;
constexpr std::uint32_t receive_not_empty = 0x20000, receive_full = 0x80000;
constexpr std::uint32_t overrun = 0x100000, busy = 0x400000;
constexpr std::uint32_t control_mask = 0x3fff, clock_mask = 0x7ff, word_mask = 0xffffff;
constexpr std::uint32_t word_size_mask = 0xc;

unsigned width(const std::uint32_t control) noexcept {
  return 8 * (((control & word_size_mask) >> 2U) + 1);
}
}

void Shi::publish() noexcept {
  const auto capacity = (control_ & fifo_enabled) != 0 ? fifo_.size() : 1;
  auto value = control_ | idle | transmit_empty | errors_;
  if (count_ != 0) { value |= receive_not_empty; }
  if (count_ >= capacity) { value |= receive_full; }
  if (selected_ && (control_ & enabled) != 0) { value |= busy; }
  status_.store(value, std::memory_order_release);
}

void Shi::reset() noexcept {
  const std::lock_guard lock(mutex_);
  head_ = count_ = 0;
  control_ = errors_ = 0;
  // DSP56371UM 7.4.5: CPHA resets set; all other HCKR fields reset clear.
  clock_ = 1;
  observed_overrun_ = false;
  publish();
  ready_.notify_all();
}

bool Shi::read(const std::uint32_t address, std::uint32_t &value) noexcept {
  if (address == status_address) {
    value = status_.load(std::memory_order_acquire);
    if ((value & overrun) != 0) { observed_overrun_ = true; }
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
  return true;
}

bool Shi::write(const Write access) noexcept {
  const std::lock_guard lock(mutex_);
  if (access.address == clock_address) {
    clock_ = access.value & clock_mask;
    return true;
  }
  if (access.address != status_address) { return false; }
  const auto next = access.value & control_mask;
  if ((next & enabled) != 0 &&
      ((next & (i2c | master)) != 0 || (next & word_size_mask) == word_size_mask)) {
    return false;
  }
  control_ = next;
  if ((control_ & enabled) == 0) {
    head_ = count_ = 0;
    errors_ = 0;
    observed_overrun_ = false;
  }
  publish();
  ready_.notify_all();
  return true;
}

std::optional<Shi::Mode> Shi::select(const bool active) noexcept {
  const std::lock_guard lock(mutex_);
  selected_ = active;
  publish();
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

bool Shi::receive(const std::uint32_t word) noexcept {
  const std::lock_guard lock(mutex_);
  if (!selected_ || (control_ & enabled) == 0 || word > word_mask) { return false; }
  const auto padding = 24 - width(control_);
  if (word > (word_mask >> padding)) { return false; }
  const auto capacity = (control_ & fifo_enabled) != 0 ? fifo_.size() : 1;
  // The receive-only binding has no initialized HTX word: the actual SPI
  // transaction sets HTUE. HTX/MISO remain explicit unsupported accesses.
  errors_ |= underrun;
  if (count_ >= capacity) {
    errors_ |= overrun;
  } else {
    fifo_[(head_ + count_) % fifo_.size()] = (word << padding) & word_mask;
    ++count_;
  }
  publish();
  return true;
}
}
