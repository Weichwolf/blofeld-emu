// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/serial_pcm.hpp"

#include <atomic>
#include <cstddef>
#include <vector>

namespace blofeld {
// One producer and one consumer. Allocate only at construction. A failed push
// retains caller ownership; a failed pop leaves the destination untouched.
class PcmQueue {
public:
  static constexpr std::size_t maximum_capacity = 65536;
  explicit PcmQueue(std::size_t capacity);
  PcmQueue(const PcmQueue &) = delete;
  PcmQueue &operator=(const PcmQueue &) = delete;
  PcmQueue(PcmQueue &&) = delete;
  PcmQueue &operator=(PcmQueue &&) = delete;
  [[nodiscard]] bool try_push(const SerialPcmFrame &frame) noexcept;
  [[nodiscard]] bool try_pop(SerialPcmFrame &frame) noexcept;

private:
  [[nodiscard]] std::size_t next(std::size_t cursor) const noexcept;
  std::vector<SerialPcmFrame> storage_;
  std::atomic<std::size_t> head_{0};
  std::atomic<std::size_t> tail_{0};
};
}
