// SPDX-License-Identifier: GPL-3.0-only
#include "io/sci.hpp"

#include <algorithm>
#include <bit>
#include <cstdint>
#include <limits>
#include <optional>

namespace blofeld::vi {
namespace {
constexpr std::uint16_t first = 0xc8, last = 0xcf;
constexpr std::uint16_t baud_high = first, baud_low = first + 1, control1 = first + 2;
constexpr std::uint16_t control2 = first + 3, status1 = first + 4, status2 = first + 5;
constexpr std::uint16_t data_high = first + 6, data_low = last, vector = 0xffd6;
constexpr std::uint16_t byte_mask = 0xff, ninth_bit = 0x100;
constexpr std::uint8_t empty = 0x80, complete = 0x40, full = 0x20, idle_flag = 0x10;
constexpr std::uint8_t overrun = 8, noise = 4, framing = 2, parity = 1;
constexpr std::uint8_t tx_flags = 0xc0, rx_flags = 0x3f, wait_disable = 0x40;
constexpr std::uint8_t loop = 0x80, nine = 0x10, wake_address = 8, idle_after_stop = 4;
constexpr std::uint8_t parity_enable = 2, odd = 1, tx_enable = 8, rx_enable = 4;
constexpr std::uint8_t wakeup = 2, send_break = 1, interrupt_mask = 0xf0;
constexpr std::uint8_t receive_irq = 0x20, idle_irq = 0x10, high_mask = 0x1f, control3_mask = 0x0a;
constexpr unsigned samples = 16, stop_transfer = 9;
constexpr auto maximum = std::numeric_limits<Sci::Cycle>::max();

void earlier(std::optional<Sci::Cycle> &next, const Sci::Cycle candidate) noexcept {
  next = next ? std::min(*next, candidate) : candidate;
}
}

bool Sci::contains(const std::uint16_t offset) noexcept {
  return offset >= first && offset <= last;
}

bool Sci::active() const noexcept {
  return activity_.clocked && (!activity_.waiting || (control1_ & wait_disable) == 0);
}

unsigned Sci::bits() const noexcept {
  return (control1_ & nine) != 0 ? 9U : 8U;
}

unsigned Sci::bit_period() const noexcept {
  return unsigned{baud_} * samples;
}

unsigned Sci::character_period() const noexcept {
  return (bits() + 2) * bit_period();
}

std::optional<Sci::Cycle> Sci::event() const noexcept {
  std::optional<Cycle> next;
  if (transmitting_) {
    earlier(next, transmitting_->end);
    if (!transmitting_->checked) { earlier(next, transmitting_->transfer); }
  }
  if (receiving_) { earlier(next, receiving_->end); }
  if (launch_) { earlier(next, *launch_); }
  if (idle_) { earlier(next, *idle_); }
  return next;
}

std::optional<Sci::Cycle> Sci::deadline() const noexcept {
  return active() && baud_ != 0 ? event() : std::nullopt;
}

bool Sci::interrupts_enabled() const noexcept {
  return (control2_ & interrupt_mask) != 0;
}

std::optional<std::uint16_t> Sci::pending() const noexcept {
  const bool tx = (status_ & control2_ & tx_flags) != 0;
  const bool rx = (control2_ & receive_irq) != 0 && (status_ & (full | overrun)) != 0;
  const bool idle_ready = (control2_ & idle_irq & status_) != 0;
  return tx || ((control2_ & wakeup) == 0 && (rx || idle_ready)) ? std::optional{vector}
                                                                 : std::nullopt;
}

Sci::Payload Sci::data() const noexcept {
  const auto width = bits();
  const auto has_parity = (control1_ & parity_enable) != 0;
  const auto data_bits = width - (has_parity ? 1U : 0U);
  auto word = unsigned{transmit_} & ((1U << data_bits) - 1U);
  if (has_parity) {
    const auto bit =
        (static_cast<unsigned>(std::popcount(word)) & 1U) ^ ((control1_ & odd) != 0 ? 1U : 0U);
    word |= bit << data_bits;
  }
  return {.word = static_cast<std::uint16_t>(word), .bits = width, .data = true};
}

bool Sci::start(const Payload payload) noexcept {
  const Cycle duration = Cycle{payload.bits + 2U} * bit_period();
  if (duration > maximum - time_) { return false; }
  const Cycle transfer = (Cycle{payload.bits + 1U} * samples + stop_transfer) * baud_;
  transmitting_ =
      Transmit{.payload = payload, .transfer = time_ + transfer, .end = time_ + duration};
  status_ &= static_cast<std::uint8_t>(~complete);
  return true;
}

bool Sci::schedule() noexcept {
  if (transmitting_ || launch_ || (control2_ & tx_enable) == 0 || (status_ & empty) != 0 ||
      baud_ == 0) {
    return true;
  }
  const auto delay = bit_period() - phase_;
  if (delay > maximum - time_) { return false; }
  launch_ = time_ + delay;
  return true;
}

bool Sci::complete_transmit(const Payload payload) noexcept {
  if (payload.data) {
    if (count_ == output_.size()) {
      error_ = "SCI output queue is full";
      return false;
    }
    output_[(head_ + count_) % output_.size()] = {
        .cycle = time_, .word = payload.word, .bits = payload.bits};
    ++count_;
  }
  transmitting_.reset();
  if (prepared_) {
    const auto next = *prepared_;
    prepared_.reset();
    return start(next);
  }
  if ((status_ & empty) != 0) { status_ |= complete; }
  return schedule();
}

void Sci::complete_receive(const Received frame) noexcept {
  receiving_.reset();
  const auto width = bits();
  if ((control2_ & wakeup) != 0 && (control1_ & wake_address) != 0) {
    if ((frame.word & (1U << (width - 1))) == 0) { return; }
    control2_ &= static_cast<std::uint8_t>(~wakeup);
  }
  if ((control2_ & wakeup) == 0) {
    if ((status_ & full) != 0) {
      status_ |= overrun;
    } else {
      received_ = frame.word;
      auto errors = frame.errors;
      if ((control1_ & parity_enable) != 0 && (static_cast<unsigned>(std::popcount(frame.word)) &
                                               1U) != ((control1_ & odd) != 0 ? 1U : 0U)) {
        errors |= parity;
      }
      status_ = static_cast<std::uint8_t>(status_ | full | errors);
      idle_armed_ = true;
    }
  }
}

void Sci::idle() noexcept {
  idle_.reset();
  rx_active_ = false;
  if ((control2_ & wakeup) != 0 && (control1_ & wake_address) == 0) {
    control2_ &= static_cast<std::uint8_t>(~wakeup);
    idle_armed_ = false;
  } else if (idle_armed_ && (control2_ & wakeup) == 0) {
    status_ |= idle_flag;
  }
}

bool Sci::events() noexcept {
  if (receiving_ && receiving_->end == time_) { complete_receive(receiving_->frame); }
  if (transmitting_ && !transmitting_->checked && transmitting_->transfer == time_) {
    transmitting_->checked = true;
    if ((control2_ & tx_enable) != 0) {
      if (preamble_) {
        prepared_ = Payload{.bits = bits()};
        preamble_ = false;
      } else if ((status_ & empty) == 0) {
        prepared_ = data();
        status_ |= empty;
      }
    }
  }
  if (transmitting_ && transmitting_->end == time_ && !complete_transmit(transmitting_->payload)) {
    return false;
  }
  if (launch_ && *launch_ == time_) {
    launch_.reset();
    if (preamble_) {
      preamble_ = false;
      if (!start({.bits = bits()})) { return false; }
    } else {
      if (!start(data())) { return false; }
      status_ |= empty;
    }
  }
  if (idle_ && *idle_ == time_) { idle(); }
  return true;
}

bool Sci::postpone(const Cycle cycles) noexcept {
  if ((transmitting_ && transmitting_->end > maximum - cycles) ||
      (receiving_ && receiving_->end > maximum - cycles) ||
      (launch_ && *launch_ > maximum - cycles) || (idle_ && *idle_ > maximum - cycles)) {
    return false;
  }
  if (transmitting_) {
    transmitting_->transfer += cycles;
    transmitting_->end += cycles;
  }
  if (receiving_) { receiving_->end += cycles; }
  if (launch_) { *launch_ += cycles; }
  if (idle_) { *idle_ += cycles; }
  time_ += cycles;
  return true;
}

bool Sci::advance(const Cycle now) noexcept {
  if (error_ != nullptr) { return false; }
  if (now < time_) {
    error_ = "SCI device time moved backwards";
    return false;
  }
  if (!active() || baud_ == 0) {
    if (postpone(now - time_)) { return true; }
    error_ = "SCI paused event timestamp cannot be represented";
    return false;
  }
  while (true) {
    const auto next = event();
    const auto until = next && *next <= now ? *next : now;
    if (generator_started_) {
      phase_ = static_cast<unsigned>((phase_ + ((until - time_) % bit_period())) % bit_period());
    }
    time_ = until;
    if (!next || *next > now) { return true; }
    if (!events()) {
      if (error_ == nullptr) { error_ = "SCI event timestamp cannot be represented"; }
      return false;
    }
  }
}

bool Sci::receive(const Received frame) noexcept {
  if ((frame.word >> bits()) != 0 || (frame.errors & ~(noise | framing)) != 0) { return false; }
  if ((control2_ & rx_enable) == 0 || !active() || baud_ == 0 || (status_ & framing) != 0) {
    return true;
  }
  if (receiving_) { return false; }
  const auto duration = character_period();
  // Completion is a decoded-character boundary, with no stop-bit sampling model.
  unsigned tail = 0;
  if ((control1_ & idle_after_stop) == 0 && (frame.errors & framing) == 0) {
    tail = 1;
    for (unsigned bit = bits(); bit != 0 && (frame.word & (1U << (bit - 1))) != 0; --bit) {
      ++tail;
    }
  }
  const Cycle idle_delay = Cycle{bits() + 2U - tail} * bit_period();
  if (duration > maximum - time_ || idle_delay > maximum - time_ - duration) { return false; }
  receiving_ = Receive{.frame = frame, .end = time_ + duration};
  idle_ = time_ + duration + idle_delay;
  rx_active_ = true;
  return true;
}

std::optional<Sci::Output> Sci::take_output() noexcept {
  if (count_ == 0) { return {}; }
  const auto result = output_[head_];
  head_ = (head_ + 1) % output_.size();
  --count_;
  return result;
}

bool Sci::read_byte(const std::uint16_t offset, std::uint8_t &value) noexcept {
  switch (offset) {
    case baud_high: value = static_cast<std::uint8_t>(baud_ >> 8U); return !baud_pending_;
    case baud_low: value = static_cast<std::uint8_t>(baud_ & byte_mask); return true;
    case control1: value = control1_; return true;
    case control2: value = control2_; return true;
    case status1:
      value = status_;
      seen_tx_ = status_ & tx_flags;
      seen_rx_ = status_ & rx_flags;
      return true;
    case status2:
      value = static_cast<std::uint8_t>(control3_ | (rx_active_ ? 1U : 0U));
      return true;
    case data_high:
      value = static_cast<std::uint8_t>(((received_ & ninth_bit) >> 1U) |
                                        ((transmit_ & ninth_bit) >> 2U));
      return true;
    case data_low:
      value = static_cast<std::uint8_t>(received_ & byte_mask);
      if ((seen_rx_ & idle_flag) != 0) { idle_armed_ = false; }
      status_ &= static_cast<std::uint8_t>(~seen_rx_);
      seen_rx_ = 0;
      return true;
    default: return false;
  }
}

bool Sci::write_control(const std::uint8_t value) noexcept {
  if ((value & send_break) != 0) { return false; }
  const auto previous = control2_;
  const auto toggled = ((previous ^ value) & tx_enable) != 0;
  if (toggled && prepared_) { return false; } // Late TE toggles need a separate pipeline contract.
  control2_ = value;
  if ((value & (tx_enable | rx_enable)) != 0) { generator_started_ = true; }
  if ((value & tx_enable) != 0 && (previous & tx_enable) == 0) {
    launch_.reset();
    if (transmitting_ || baud_ == 0) {
      preamble_ = true;
    } else {
      preamble_ = false;
      if (!start({.bits = bits()})) { return false; }
    }
  }
  if ((value & tx_enable) == 0) { launch_.reset(); }
  if ((value & rx_enable) == 0) {
    receiving_.reset();
    idle_.reset();
    rx_active_ = false;
  } else if ((previous & rx_enable) == 0 && baud_ != 0) {
    if (character_period() > maximum - time_) { return false; }
    idle_armed_ = true;
    idle_ = time_ + character_period();
  }
  return schedule();
}

bool Sci::write_byte(const Write access) noexcept {
  const auto value = static_cast<std::uint8_t>(access.value);
  switch (access.offset) {
    case baud_high:
      baud_high_ = value & high_mask;
      baud_pending_ = true;
      return true;
    case baud_low:
      if (transmitting_ || receiving_) { return false; }
      baud_ = static_cast<std::uint16_t>((unsigned{baud_high_} << 8U) | value);
      baud_pending_ = false;
      phase_ = 0;
      if (idle_) {
        if (character_period() > maximum - time_) { return false; }
        idle_ = baud_ == 0 ? std::nullopt : std::optional{time_ + character_period()};
      }
      if (preamble_ && (control2_ & tx_enable) != 0 && baud_ != 0) {
        preamble_ = false;
        return start({.bits = bits()});
      }
      return schedule();
    case control1:
      if ((value & loop) != 0 ||
          (((value ^ control1_) & ~wait_disable) != 0 && (transmitting_ || receiving_))) {
        return false;
      }
      control1_ = value;
      return true;
    case control2: return write_control(value);
    case status1: return true;
    case status2: control3_ = value & control3_mask; return true;
    case data_high:
      transmit_ = static_cast<std::uint16_t>((transmit_ & byte_mask) | ((value & complete) << 2U));
      return true;
    case data_low:
      transmit_ = static_cast<std::uint16_t>((transmit_ & ninth_bit) | value);
      status_ &= static_cast<std::uint8_t>(~seen_tx_);
      seen_tx_ = 0;
      if ((status_ & empty) == 0) { status_ &= static_cast<std::uint8_t>(~complete); }
      return schedule();
    default: return false;
  }
}

bool Sci::read(const Read access, std::uint16_t &value) noexcept {
  if (access.bytes != 1 && access.bytes != 2) { return false; }
  std::uint8_t high = 0;
  std::uint8_t low = 0;
  if (!read_byte(access.offset, high)) { return false; }
  value = high;
  if (access.bytes == 1) { return true; }
  if (!read_byte(static_cast<std::uint16_t>(access.offset + 1), low)) { return false; }
  value = static_cast<std::uint16_t>((unsigned{high} << 8U) | low);
  return true;
}

bool Sci::write(const Write access) noexcept {
  if (access.bytes == 1) { return access.value <= byte_mask && write_byte(access); }
  if (access.bytes != 2) { return false; }
  return write_byte(
             {.offset = access.offset, .value = static_cast<std::uint16_t>(access.value >> 8U)}) &&
         write_byte({.offset = static_cast<std::uint16_t>(access.offset + 1),
                     .value = static_cast<std::uint16_t>(access.value & byte_mask)});
}
}
