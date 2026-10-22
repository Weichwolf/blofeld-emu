// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/serial_pcm.hpp"

#include <array>
#include <cstddef>

namespace blofeld {
// Single-owner FIFO; producer and consumer run at separate operation boundaries.
// No allocation, synchronization, dropping or sample fabrication; copies are snapshots.
template <std::size_t Capacity> class PcmQueue {
  static_assert(Capacity > 0 && Capacity <= 65536);

public:
  [[nodiscard]] std::size_t size() const noexcept { return size_; }

  [[nodiscard]] std::size_t available() const noexcept { return Capacity - size_; }

  [[nodiscard]] const SerialPcmFrame *peek(const std::size_t offset = 0) const noexcept {
    return offset < size_ ? &frames_[(head_ + offset) % Capacity] : nullptr;
  }

  bool push(const SerialPcmFrame &frame) noexcept {
    if (size_ == Capacity) { return false; }
    frames_[(head_ + size_) % Capacity] = frame;
    ++size_;
    return true;
  }

  bool pop(const std::size_t count = 1) noexcept {
    if (count > size_) { return false; }
    head_ = (head_ + count) % Capacity;
    size_ -= count;
    return true;
  }

  bool operator==(const PcmQueue &) const = default;

private:
  std::array<SerialPcmFrame, Capacity> frames_{};
  std::size_t head_ = 0;
  std::size_t size_ = 0;
};
}
