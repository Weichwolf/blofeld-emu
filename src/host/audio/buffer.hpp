// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stop_token>

namespace blofeld::host::audio {
using Frame = std::array<std::int32_t, 2>; // Signed 24-bit PCM in native 32-bit containers.

// One producer and one consumer. Borrowed readable spans remain owned by the
// consumer until consume(); no allocation, locks, or per-frame notifications.
class Buffer {
public:
  static constexpr std::size_t capacity = 32;
  [[nodiscard]] std::size_t produce(std::span<const Frame> frames) noexcept;
  [[nodiscard]] std::span<const Frame> readable() const noexcept;
  [[nodiscard]] bool consume(std::size_t frames) noexcept;
  void discard() noexcept;
  [[nodiscard]] bool wait_space(const std::stop_token &stop) noexcept;
  void close() noexcept;

  [[nodiscard]] bool closed() const noexcept { return closed_.load(std::memory_order_acquire); }

  [[nodiscard]] std::size_t queued() const noexcept;

private:
  void signal() noexcept;
  std::array<Frame, capacity> frames_{};
  std::atomic<std::uint64_t> head_{0}, tail_{0}, changed_{0};
  std::atomic<bool> closed_{false};
};
}
