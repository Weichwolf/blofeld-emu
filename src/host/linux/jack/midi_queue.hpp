// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace blofeld::host::audio {
// One producer and one consumer. The producer publishes a complete JACK event
// with one release store, including a SysEx message that wraps the byte ring.
class JackMidiQueue {
public:
  static constexpr std::size_t capacity = 128UL * 1024;

  struct Message {
    std::uint64_t frame;
    std::span<const std::uint8_t> first, second;

    [[nodiscard]] std::size_t size() const noexcept { return first.size() + second.size(); }
  };

  [[nodiscard]] bool push(std::uint64_t frame, std::span<const std::uint8_t> bytes) noexcept {
    const auto head = head_.load(std::memory_order_relaxed);
    const auto tail = tail_.load(std::memory_order_acquire);
    if (head - tail == entries_.size() || bytes.empty() ||
        bytes.size() >
            storage_.size() -
                static_cast<std::size_t>(written_ - tail_bytes_.load(std::memory_order_acquire))) {
      return false;
    }
    const auto offset = static_cast<std::size_t>(written_ % storage_.size());
    const auto first = std::min(bytes.size(), storage_.size() - offset);
    std::ranges::copy(bytes.first(first), storage_.data() + offset);
    std::ranges::copy(bytes.subspan(first), storage_.data());
    entries_[head % entries_.size()] = {.frame = frame, .end = written_ + bytes.size()};
    written_ += bytes.size();
    head_.store(head + 1, std::memory_order_release);
    return true;
  }

  [[nodiscard]] std::optional<Message> peek() const noexcept {
    const auto tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) { return {}; }
    const auto &entry = entries_[tail % entries_.size()];
    const auto offset = static_cast<std::size_t>(read_ % storage_.size());
    const auto bytes = static_cast<std::size_t>(entry.end - read_);
    const auto first = std::min(bytes, storage_.size() - offset);
    const std::span data(storage_);
    return Message{.frame = entry.frame,
                   .first = data.subspan(offset, first),
                   .second = data.first(bytes - first)};
  }

  // A DIN consumer can retire only the prefix it can admit on the firmware timeline.
  void consume(std::size_t bytes) noexcept {
    const auto tail = tail_.load(std::memory_order_relaxed);
    read_ += bytes;
    tail_bytes_.store(read_, std::memory_order_release);
    if (read_ == entries_[tail % entries_.size()].end) {
      tail_.store(tail + 1, std::memory_order_release);
    }
  }

private:
  struct Entry {
    std::uint64_t frame = 0, end = 0;
  };

  static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
  std::array<Entry, 1024> entries_{};
  std::array<std::uint8_t, capacity> storage_{};
  std::atomic<std::uint64_t> head_{0}, tail_{0}, tail_bytes_{0};
  std::uint64_t written_ = 0, read_ = 0;
};
}
