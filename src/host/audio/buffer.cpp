// SPDX-License-Identifier: GPL-3.0-only
#include "audio/buffer.hpp"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <stop_token>

namespace blofeld::host::audio {
std::size_t Buffer::produce(const std::span<const Frame> frames) noexcept {
  if (closed()) { return 0; }
  const auto head = head_.load(std::memory_order_relaxed);
  const auto tail = tail_.load(std::memory_order_acquire);
  const auto used = head - tail;
  const auto begin = static_cast<std::size_t>(head % capacity);
  const auto count =
      std::min({frames.size(), capacity - static_cast<std::size_t>(used), capacity - begin});
  if (count > std::numeric_limits<std::uint64_t>::max() - head) {
    close();
    return 0;
  }
  std::ranges::copy(frames.first(count), frames_.begin() + static_cast<std::ptrdiff_t>(begin));
  head_.store(head + count, std::memory_order_release);
  return count;
}

std::span<const Frame> Buffer::readable() const noexcept {
  const auto tail = tail_.load(std::memory_order_relaxed);
  const auto head = head_.load(std::memory_order_acquire);
  const auto begin = static_cast<std::size_t>(tail % capacity);
  return std::span(frames_).subspan(
      begin, std::min(static_cast<std::size_t>(head - tail), capacity - begin));
}

void Buffer::signal() noexcept {
  changed_.fetch_add(1, std::memory_order_release);
  changed_.notify_all();
}

bool Buffer::consume(const std::size_t frames) noexcept {
  if (frames > readable().size()) { return false; }
  tail_.fetch_add(frames, std::memory_order_release);
  signal();
  return true;
}

void Buffer::discard() noexcept {
  tail_.store(head_.load(std::memory_order_acquire), std::memory_order_release);
  signal();
}

std::size_t Buffer::queued() const noexcept {
  const auto tail = tail_.load(std::memory_order_acquire);
  const auto head = head_.load(std::memory_order_acquire);
  return std::min(capacity, static_cast<std::size_t>(head - tail));
}

bool Buffer::wait_space(const std::stop_token &stop) noexcept {
  const std::stop_callback cancel(stop, [this] { signal(); });
  for (;;) {
    const auto change = changed_.load(std::memory_order_acquire);
    if (closed() || stop.stop_requested()) { return false; }
    if (queued() < capacity) { return true; }
    changed_.wait(change, std::memory_order_acquire);
  }
}

void Buffer::close() noexcept {
  closed_.store(true, std::memory_order_release);
  signal();
}
}
