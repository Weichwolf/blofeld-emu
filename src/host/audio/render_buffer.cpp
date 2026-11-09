// SPDX-License-Identifier: GPL-3.0-only
#include "audio/render_buffer.hpp"

#include "audio/buffer.hpp"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <stdexcept>

namespace blofeld::host::audio {
static_assert(std::atomic<std::uint64_t>::is_always_lock_free);

RenderBuffer::RenderBuffer(const std::size_t capacity) : frames_(capacity) {
  if (frames_.empty()) { throw std::invalid_argument("Render buffer must have capacity"); }
}

std::size_t RenderBuffer::produce(const std::span<const Frame> frames) {
  const auto head = head_.load(std::memory_order_relaxed);
  const auto tail = tail_.load(std::memory_order_acquire);
  const auto begin = static_cast<std::size_t>(head % capacity());
  const auto count = std::min(
      {frames.size(), capacity() - static_cast<std::size_t>(head - tail), capacity() - begin});
  if (count > std::numeric_limits<std::uint64_t>::max() - head) {
    throw std::overflow_error("Render frame counter exhausted");
  }
  std::ranges::copy(frames.first(count), frames_.begin() + static_cast<std::ptrdiff_t>(begin));
  head_.store(head + count, std::memory_order_release);
  return count;
}

std::size_t RenderBuffer::render(const std::span<FloatFrame> output) noexcept
    [[clang::nonblocking]] {
  const auto tail = tail_.load(std::memory_order_relaxed);
  const auto head = head_.load(std::memory_order_acquire);
  const auto count = std::min(output.size(), static_cast<std::size_t>(head - tail));
  constexpr float scale = 1.0F / 8388608.0F;
  std::size_t copied = 0;
  while (copied < count) {
    const auto begin = static_cast<std::size_t>((tail + copied) % capacity());
    const auto chunk = std::min(count - copied, capacity() - begin);
    for (std::size_t index = 0; index < chunk; ++index) {
      const auto &frame = frames_[begin + index];
      output[copied + index] = {static_cast<float>(frame[0]) * scale,
                                static_cast<float>(frame[1]) * scale};
    }
    copied += chunk;
  }
  std::ranges::fill(output.subspan(count), FloatFrame{});
  tail_.store(tail + count, std::memory_order_release);
  return count;
}

std::size_t RenderBuffer::queued() const noexcept {
  const auto tail = tail_.load(std::memory_order_acquire);
  const auto head = head_.load(std::memory_order_acquire);
  return std::min(capacity(), static_cast<std::size_t>(head - tail));
}

void RenderBuffer::discard() noexcept {
  tail_.store(head_.load(std::memory_order_acquire), std::memory_order_release);
}
}
