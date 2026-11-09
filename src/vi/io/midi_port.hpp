// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace blofeld::vi {
// One host owner and one MCU owner. DIN character starts use the common 60 MHz
// timeline; the receiver still applies its genuine SCI format/status/IRQ effects.
class MidiPort {
public:
  struct Event {
    std::uint64_t ticks = 0;
    std::uint8_t byte = 0;
    bool operator==(const Event &) const = default;
  };

  static constexpr std::size_t capacity = 1024;
  static constexpr std::uint64_t byte_ticks = 19200; // 8N1 at 31,250 baud.

  // Host owner: packets are serialized on this one DIN input, without reordering.
  [[nodiscard]] bool write_input(std::span<const std::uint8_t> bytes, std::uint64_t ticks) noexcept;
  [[nodiscard]] std::optional<Event> read_output() noexcept;

  [[nodiscard]] std::uint64_t time() const noexcept {
    return time_.load(std::memory_order_acquire);
  }

  // MCU owner: only actual SCI character completions publish output.
  [[nodiscard]] std::optional<Event> peek_input() const noexcept;
  void pop_input() noexcept;
  [[nodiscard]] bool write_output(Event event) noexcept;

  void publish_time(std::uint64_t ticks) noexcept { time_.store(ticks, std::memory_order_release); }

  [[nodiscard]] const char *error() const noexcept {
    return error_.load(std::memory_order_acquire);
  }

private:
  struct Queue {
    std::array<Event, capacity> events{};
    std::atomic<std::uint64_t> read{0}, written{0};
    [[nodiscard]] std::optional<Event> peek() const noexcept;
    void pop() noexcept;
  };

  [[nodiscard]] bool fail(const char *message) noexcept;
  Queue input_, output_;
  std::uint64_t next_input_ = 0, last_input_ = 0, last_output_ = 0;
  std::atomic<std::uint64_t> time_{0};
  std::atomic<const char *> error_{nullptr};
};
}
