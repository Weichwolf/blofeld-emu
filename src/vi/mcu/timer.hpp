// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstdint>
#include <optional>

namespace blofeld::vi::mcu {
// TIM16B8CV1: free-running counter, capture/compare flags and IRQ requests.
// All operations belong to the MCU owner. Time is absolute nominal bus cycles.
class Timer {
public:
  using Cycle = std::uint64_t;
  enum class Activity : std::uint8_t { running, waiting, stopped };

  struct Read {
    std::uint16_t offset = 0;
    std::uint8_t bytes = 1;
  };

  struct Write {
    std::uint16_t offset = 0, value = 0;
    std::uint8_t bytes = 1;
  };

  struct Capture {
    unsigned channel = 0;
    bool rising = false;
  };

  [[nodiscard]] static bool contains(std::uint16_t offset) noexcept;
  [[nodiscard]] bool advance(Cycle now) noexcept;
  [[nodiscard]] bool read(Read access, std::uint16_t &value) noexcept;
  [[nodiscard]] bool write(Write access) noexcept;
  [[nodiscard]] bool capture(Capture edge) noexcept;
  void activity(Activity state) noexcept;
  [[nodiscard]] std::optional<std::uint16_t> pending() const noexcept;
  [[nodiscard]] std::optional<Cycle> deadline() const noexcept;
  [[nodiscard]] bool interrupts_enabled() const noexcept;

private:
  [[nodiscard]] bool active() const noexcept;
  void count(Cycle ticks) noexcept;
  void elapsed(Cycle cycles) noexcept;
  [[nodiscard]] Cycle clocks(Cycle ticks) const noexcept;
  [[nodiscard]] bool read_byte(std::uint16_t offset, std::uint8_t &value) noexcept;
  [[nodiscard]] bool write_byte(Write access) noexcept;
  [[nodiscard]] bool write_control(unsigned index, std::uint8_t value) noexcept;
  void clear_channel(unsigned channel, bool writing) noexcept;
  std::array<std::uint8_t, 16> registers_{};
  std::array<std::uint16_t, 8> channels_{};
  Cycle time_ = 0;
  std::uint16_t counter_ = 0, pulse_count_ = 0;
  unsigned phase_ = 0, prescale_ = 0;
  std::uint8_t pulse_control_ = 0;
  Activity activity_ = Activity::running;
};
}
