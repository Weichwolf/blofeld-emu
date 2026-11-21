// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <stop_token>

namespace blofeld::vi {
struct CheckpointWait {
  const std::uint64_t &generation;
  std::uint64_t observed, command;
};

// Separate host input/output owners; MCU SCI and DSP Timer0 observe the same DIN input
// independently. Input slots remain immutable until both observers retire them. Only SCI sends DIN
// output. Character starts use the common 60 MHz instrument timeline.
class MidiPort {
public:
  enum class Receiver : std::uint8_t { mcu, dsp };
  enum class InputMode : std::uint8_t { live, stream, audio };
  enum class InputStatus : std::uint8_t { admitted, full, refused };

  struct Admission {
    InputStatus status = InputStatus::refused;
    std::uint64_t tick = 0;
  };

  struct Event {
    std::uint64_t ticks = 0;
    std::uint8_t byte = 0;
    bool operator==(const Event &) const = default;
  };

  static constexpr std::size_t capacity = 1024;
  static constexpr std::uint32_t reference_hz = 60000000;
  static constexpr std::uint64_t byte_ticks = 19200; // 8N1 at 31,250 baud.

  explicit MidiPort(InputMode input = InputMode::live) noexcept : input_mode_(input) {}

  void wake(void *owner, void (*notify)(void *) noexcept) noexcept {
    wake_owner_ = owner;
    wake_notify_ = notify;
  }

  struct AudioSource {
    void *owner = nullptr;
    bool (*refill)(void *, std::uint64_t, const std::stop_token &) = nullptr;
    void (*before_mcu_wait)(void *, std::uint64_t) noexcept = nullptr;
    bool (*before_checkpoint_wait)(void *, CheckpointWait) noexcept = nullptr;
  };

  void audio_source(AudioSource source) noexcept { audio_source_ = source; }

  void before_mcu_wait(std::uint64_t tick) const noexcept {
    if (audio_source_.before_mcu_wait != nullptr) {
      audio_source_.before_mcu_wait(audio_source_.owner, tick);
    }
  }

  [[nodiscard]] bool audio_driven() const noexcept { return input_mode_ == InputMode::audio; }

  [[nodiscard]] bool before_checkpoint_wait(const CheckpointWait wait) const noexcept {
    return audio_source_.before_checkpoint_wait != nullptr &&
           audio_source_.before_checkpoint_wait(audio_source_.owner, wait);
  }

  void begin_rendering(std::uint64_t origin) noexcept {
    render_origin_.store(origin, std::memory_order_release);
  }

  [[nodiscard]] bool rendering_at(std::uint64_t tick) const noexcept {
    return tick >= render_origin_.load(std::memory_order_acquire);
  }

  [[nodiscard]] bool prepare_audio(std::uint64_t ticks, const std::stop_token &stop);

  // Input producer: packets are serialized on this DIN input, without reordering.
  [[nodiscard]] bool write_input(std::span<const std::uint8_t> bytes, std::uint64_t ticks) noexcept;
  [[nodiscard]] bool write_input(std::span<const std::uint8_t> bytes,
                                 std::uint64_t ticks,
                                 const std::stop_token &stop) noexcept;
  // Audio owner stages packets without advancing the exclusive input frontier.
  [[nodiscard]] Admission stage_input(std::span<const std::uint8_t> bytes,
                                      std::uint64_t ticks) noexcept;
  [[nodiscard]] bool advance_input(std::uint64_t ticks) noexcept;
  [[nodiscard]] bool finish_input() noexcept;
  void abort_input() noexcept;

  // A separate host owner may drain replies while input admission waits for credit.
  [[nodiscard]] std::optional<Event> read_output() noexcept;
  [[nodiscard]] bool await_output(const std::stop_token &stop) noexcept;

  [[nodiscard]] std::optional<std::uint64_t> input_limit() const noexcept;
  [[nodiscard]] std::uint64_t input_knowledge() const noexcept;

  [[nodiscard]] std::uint64_t time() const noexcept {
    return time_.load(std::memory_order_acquire);
  }

  // Each receiver owner retires only its own input observations.
  [[nodiscard]] std::optional<Event> peek_input(Receiver receiver = Receiver::mcu,
                                                std::uint64_t offset = 0) const noexcept;
  void pop_input(Receiver receiver = Receiver::mcu) noexcept;
  // MCU owner: only actual SCI character completions publish output.
  [[nodiscard]] bool write_output(Event event) noexcept;
  [[nodiscard]] bool await_input(std::uint64_t ticks, const std::stop_token &stop) noexcept;
  void request_input(std::uint64_t ticks) noexcept;

  [[nodiscard]] std::uint64_t requested_input() const noexcept {
    return input_requested_.load(std::memory_order_acquire);
  }

  void publish_time(std::uint64_t ticks) noexcept {
    if (ticks > time_.load(std::memory_order_relaxed)) {
      time_.store(ticks, std::memory_order_release);
    }
  }

  [[nodiscard]] const char *error() const noexcept {
    return error_.load(std::memory_order_acquire);
  }

private:
  enum class Phase : std::uint8_t { active, ended };

  struct Input {
    std::array<Event, capacity> events{};
    std::atomic<std::uint64_t> mcu{0}, dsp{0}, written{0};
    [[nodiscard]] std::optional<Event> peek(Receiver receiver, std::uint64_t offset) const noexcept;
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
  await(std::atomic<unsigned> &waiters, const std::stop_token &stop, Ready ready) noexcept;
  void signal() noexcept;
  void notify() noexcept;
  void signal_output() noexcept;
  [[nodiscard]] Admission admit(std::span<const std::uint8_t> bytes,
                                std::uint64_t ticks,
                                const std::stop_token *stop,
                                bool staged) noexcept;
  [[nodiscard]] bool fail(const char *message) noexcept;
  void *wake_owner_ = nullptr;
  void (*wake_notify_)(void *) noexcept = nullptr;
  const InputMode input_mode_;
  AudioSource audio_source_;
  std::atomic<Phase> phase_{Phase::active};
  Input input_;
  Queue output_;
  std::uint64_t next_input_ = 0, last_input_ = 0, last_output_ = 0;
  std::atomic<std::uint64_t> input_until_{0}, changed_{0};
  std::atomic<unsigned> input_waiters_{0}, space_waiters_{0};
  std::atomic<std::uint64_t> output_changed_{0};
  std::atomic<std::uint64_t> time_{0};
  std::atomic<const char *> error_{nullptr};
  std::atomic<std::uint64_t> input_requested_{0};
  std::atomic<std::uint64_t> render_origin_{std::numeric_limits<std::uint64_t>::max()};
};
}
