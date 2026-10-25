// SPDX-License-Identifier: GPL-3.0-only
#include "audio/pcm_queue.hpp"

#include "audio/serial_pcm.hpp"
#include "common/bus.hpp"

#include <atomic>
#include <cstddef>
#include <type_traits>

namespace blofeld {
namespace {
std::size_t storage_size(const std::size_t capacity) {
  if (capacity == 0 || capacity > PcmQueue::maximum_capacity) {
    throw Fault("Invalid bounded PCM handoff capacity");
  }
  return capacity + 1;
}
}

PcmQueue::PcmQueue(const std::size_t capacity) : storage_(storage_size(capacity)) {
  static_assert(std::atomic<std::size_t>::is_always_lock_free);
  static_assert(std::is_nothrow_copy_assignable_v<SerialPcmFrame>);
}

std::size_t PcmQueue::next(const std::size_t cursor) const noexcept {
  return cursor + 1 == storage_.size() ? 0 : cursor + 1;
}

bool PcmQueue::try_push(const SerialPcmFrame &frame) noexcept {
  const auto tail = tail_.load(std::memory_order_relaxed);
  const auto following = next(tail);
  if (following == head_.load(std::memory_order_acquire)) { return false; }
  storage_[tail] = frame;
  tail_.store(following, std::memory_order_release);
  return true;
}

bool PcmQueue::try_pop(SerialPcmFrame &frame) noexcept {
  const auto head = head_.load(std::memory_order_relaxed);
  if (head == tail_.load(std::memory_order_acquire)) { return false; }
  frame = storage_[head];
  head_.store(next(head), std::memory_order_release);
  return true;
}
}
