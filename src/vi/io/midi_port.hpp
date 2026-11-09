// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <stop_token>

namespace blofeld::vi {
// Separate host input/output owners; MCU SCI and DSP Timer0 observe the same DIN input
// independently. Input slots remain immutable until both observers retire them. Only SCI sends DIN
// output. Character starts use the common 60 MHz instrument timeline.
class MidiPort {
public:
  enum class Receiver : std::uint8_t { mcu, dsp };
  enum class InputMode : std::uint8_t { live, stream };

  struct Event {
    std::uint64_t ticks = 0;
    std::uint8_t byte = 0;
    bool operator==(const Event &) const = default;
  };

  static constexpr std::size_t capacity = 1024;
  static constexpr std::uint32_t reference_hz = 60000000;
  static constexpr std::uint64_t byte_ticks = 19200; // 8N1 at 31,250 baud.

  explicit MidiPort(InputMode input = InputMode::live) noexcept : input_mode_(input) {}

  // Input producer: packets are serialized on this DIN input, without reordering.
  [[nodiscard]] bool write_input(std::span<const std::uint8_t> bytes, std::uint64_t ticks) noexcept;
  [[nodiscard]] bool write_input(std::span<const std::uint8_t> bytes,
                                 std::uint64_t ticks,
                                 const std::stop_token &stop) noexcept;
  [[nodiscard]] bool advance_input(std::uint64_t ticks) noexcept;
  [[nodiscard]] bool finish_input() noexcept;
  void abort_input() noexcept;

  // A separate host owner may drain replies while input admission waits for credit.
  [[nodiscard]] std::optional<Event> read_output() noexcept;
  [[nodiscard]] bool await_output(const std::stop_token &stop) noexcept;

  [[nodiscard]] std::optional<std::uint64_t> input_limit() const noexcept;

  [[nodiscard]] std::uint64_t time() const noexcept {
    return time_.load(std::memory_order_acquire);
  }

  // Each receiver owner retires only its own input observations.
  [[nodiscard]] std::optional<Event> peek_input(Receiver receiver = Receiver::mcu) const noexcept;
  void pop_input(Receiver receiver = Receiver::mcu) noexcept;
  // MCU owner: only actual SCI character completions publish output.
  [[nodiscard]] bool write_output(Event event) noexcept;
  [[nodiscard]] bool await_input(std::uint64_t ticks, const std::stop_token &stop) noexcept;

  void publish_time(std::uint64_t ticks) noexcept { time_.store(ticks, std::memory_order_release); }

  [[nodiscard]] const char *error() const noexcept {
    return error_.load(std::memory_order_acquire);
  }

private:
  enum class Phase : std::uint8_t { active, ended };

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

  template <typename Ready>
  [[nodiscard]] bool
  await(std::atomic<bool> &waiting, const std::stop_token &stop, Ready ready) noexcept;
  void signal() noexcept;
  void notify() noexcept;
  [[nodiscard]] bool admit(std::span<const std::uint8_t> bytes,
                           std::uint64_t ticks,
                           const std::stop_token *stop) noexcept;
  [[nodiscard]] bool fail(const char *message) noexcept;
  const InputMode input_mode_;
  std::atomic<Phase> phase_{Phase::active};
  Input input_;
  Queue output_;
  std::uint64_t next_input_ = 0, last_input_ = 0, last_output_ = 0;
  std::atomic<std::uint64_t> input_until_{0}, changed_{0};
  std::atomic<bool> input_waiting_{false}, space_waiting_{false}, output_waiting_{false};
  std::atomic<std::uint64_t> time_{0};
  std::atomic<const char *> error_{nullptr};
};
}
