// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace blofeld::vi {
// UF32 SCI: character transactions, register effects and nominal bus deadlines.
// All operations, including receive/take_output, belong to the MCU owner. A host
// adapter must deliver ordered frame starts at device time, never mutate registers.
class Sci {
public:
  using Cycle = std::uint64_t;
  static constexpr std::size_t output_capacity = 64;

  struct Read {
    std::uint16_t offset = 0;
    std::uint8_t bytes = 1;
  };

  struct Write {
    std::uint16_t offset = 0, value = 0;
    std::uint8_t bytes = 1;
  };

  struct Received {
    std::uint16_t word = 0;  // Includes the received parity bit when enabled.
    std::uint8_t errors = 0; // External noise/framing flags; parity is checked here.
  };

  struct Output {
    Cycle cycle = 0; // Complete character, including its stop bit, in device time.
    std::uint16_t word = 0;
    unsigned bits = 0; // Eight or nine data/parity bits; excludes start/stop.
  };

  struct Activity {
    bool waiting = false;
    bool clocked = true;
  };

  [[nodiscard]] static bool contains(std::uint16_t offset) noexcept;
  [[nodiscard]] bool read(Read access, std::uint16_t &value) noexcept;
  [[nodiscard]] bool write(Write access) noexcept;
  [[nodiscard]] bool advance(Cycle now) noexcept;
  [[nodiscard]] bool receive(Received frame) noexcept;
  [[nodiscard]] std::optional<Output> take_output() noexcept;

  void activity(Activity state) noexcept { activity_ = state; }

  [[nodiscard]] std::optional<std::uint16_t> pending() const noexcept;
  [[nodiscard]] std::optional<Cycle> deadline() const noexcept;
  [[nodiscard]] bool interrupts_enabled() const noexcept;

  [[nodiscard]] const char *error() const noexcept { return error_; }

private:
  struct Payload {
    std::uint16_t word = 0;
    unsigned bits = 8;
    bool data = false;
  };

  struct Transmit {
    Payload payload;
    Cycle transfer = 0, end = 0;
    bool checked = false;
  };

  struct Receive {
    Received frame;
    Cycle end = 0;
  };

  [[nodiscard]] bool active() const noexcept;
  [[nodiscard]] unsigned bits() const noexcept;
  [[nodiscard]] unsigned bit_period() const noexcept;
  [[nodiscard]] unsigned character_period() const noexcept;
  [[nodiscard]] std::optional<Cycle> event() const noexcept;
  [[nodiscard]] Payload data() const noexcept;
  bool start(Payload payload) noexcept;
  bool schedule() noexcept;
  bool events() noexcept;
  bool complete_transmit(Payload payload) noexcept;
  void complete_receive(Received frame) noexcept;
  bool postpone(Cycle cycles) noexcept;
  bool write_control(std::uint8_t value) noexcept;
  bool write_byte(Write access) noexcept;
  bool read_byte(std::uint16_t offset, std::uint8_t &value) noexcept;
  void idle() noexcept;
  std::array<Output, output_capacity> output_{};
  std::size_t head_ = 0, count_ = 0;
  std::optional<Transmit> transmitting_;
  std::optional<Receive> receiving_;
  std::optional<Payload> prepared_;
  std::optional<Cycle> launch_, idle_;
  Cycle time_ = 0;
  std::uint16_t baud_ = 4, received_ = 0, transmit_ = 0;
  unsigned phase_ = 0;
  static constexpr std::uint8_t reset_status = 0xc0;
  std::uint8_t baud_high_ = 0, control1_ = 0, control2_ = 0, control3_ = 0;
  std::uint8_t status_ = reset_status, seen_tx_ = 0, seen_rx_ = 0;
  bool baud_pending_ = false, generator_started_ = false, preamble_ = false, rx_active_ = false;
  bool idle_armed_ = true;
  Activity activity_;
  const char *error_ = nullptr;
};
}
