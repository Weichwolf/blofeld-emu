// SPDX-License-Identifier: GPL-3.0-only
#include "audio/render_buffer.hpp"

#include "audio/pcm_frame.hpp"

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
  const auto available = capacity() - static_cast<std::size_t>(head - tail);
  if (available == 0 || frames.empty()) { return 0; }
  const auto begin = static_cast<std::size_t>(head % capacity());
  const auto count = std::min({frames.size(), available, capacity() - begin});
  if (count > std::numeric_limits<std::uint64_t>::max() - head) {
    throw std::overflow_error("Render frame counter exhausted");
  }
  std::ranges::copy(frames.first(count), frames_.begin() + static_cast<std::ptrdiff_t>(begin));
  head_.store(head + count, std::memory_order_release);
  return count;
}

template <typename Store>
std::size_t RenderBuffer::render_frames(const std::size_t frames,
                                        const Store store,
                                        std::size_t *const late) noexcept {
  auto tail = tail_.load(std::memory_order_relaxed);
  const auto head = head_.load(std::memory_order_acquire);
  if (late != nullptr) {
    const auto expired = std::min(*late, static_cast<std::size_t>(head - tail));
    tail += expired;
    *late -= expired;
  }
  const auto count = std::min(frames, static_cast<std::size_t>(head - tail));
  constexpr float scale = 1.0F / 8388608.0F;
  std::size_t copied = 0;
  while (copied < count) {
    const auto begin = static_cast<std::size_t>((tail + copied) % capacity());
    const auto chunk = std::min(count - copied, capacity() - begin);
    for (std::size_t index = 0; index < chunk; ++index) {
      const auto &frame = frames_[begin + index];
      store(copied + index,
            FloatFrame{static_cast<float>(frame[0]) * scale, static_cast<float>(frame[1]) * scale});
    }
    copied += chunk;
  }
  for (std::size_t index = count; index < frames; ++index) { store(index, FloatFrame{}); }
  tail_.store(tail + count, std::memory_order_release);
  if (late != nullptr) { *late += frames - count; }
  return count;
}

std::size_t RenderBuffer::render(const std::span<FloatFrame> output,
                                 std::size_t *const late) noexcept [[clang::nonblocking]] {
  return render_frames(
      output.size(),
      [output](std::size_t index, FloatFrame frame) noexcept { output[index] = frame; },
      late);
}

std::size_t RenderBuffer::render(const std::span<float> left,
                                 const std::span<float> right,
                                 std::size_t *const late) noexcept [[clang::nonblocking]] {
  return render_frames(
      left.size(),
      [left, right](std::size_t index, FloatFrame frame) noexcept {
        left[index] = frame[0];
        right[index] = frame[1];
      },
      late);
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
