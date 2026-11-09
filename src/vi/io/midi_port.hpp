// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace blofeld::vi {
// One host owner; MCU SCI and DSP Timer0 observe the same DIN input independently.
// Input slots remain immutable until both observers retire them. Only SCI sends
// DIN output. Character starts use the common 60 MHz instrument timeline.
class MidiPort {
public:
  enum class Receiver : std::uint8_t { mcu, dsp };

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

  // Each receiver owner retires only its own input observations.
  [[nodiscard]] std::optional<Event> peek_input(Receiver receiver = Receiver::mcu) const noexcept;
  void pop_input(Receiver receiver = Receiver::mcu) noexcept;
  // MCU owner: only actual SCI character completions publish output.
  [[nodiscard]] bool write_output(Event event) noexcept;

  void publish_time(std::uint64_t ticks) noexcept { time_.store(ticks, std::memory_order_release); }

  [[nodiscard]] const char *error() const noexcept {
    return error_.load(std::memory_order_acquire);
  }

private:
  struct Input {
    std::array<Event, capacity> events{};
    std::atomic<std::uint64_t> mcu{0}, dsp{0}, written{0};
    [[nodiscard]] std::optional<Event> peek(Receiver receiver) const noexcept;
    void pop(Receiver receiver) noexcept;
  };

  struct Queue {
    std::array<Event, capacity> events{};
    std::atomic<std::uint64_t> read{0}, written{0};
    [[nodiscard]] std::optional<Event> peek() const noexcept;
    void pop() noexcept;
  };

  [[nodiscard]] bool fail(const char *message) noexcept;
  Input input_;
  Queue output_;
  std::uint64_t next_input_ = 0, last_input_ = 0, last_output_ = 0;
  std::atomic<std::uint64_t> time_{0};
  std::atomic<const char *> error_{nullptr};
};
}
