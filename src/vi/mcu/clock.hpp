// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <optional>

namespace blofeld::vi::mcu {
// UF32 CRG_U and USB clock control, using the specified 12 MHz reference and
// nominal 60 MHz PHY output. Time is advanced analytically by the MCU owner.
// A stable-source qualification window is functional, not an analog PLL model.
class Clock {
public:
  using Cycle = std::uint64_t;
  enum class Activity : std::uint8_t { running, waiting, stopped };

  struct Write {
    std::uint16_t offset = 0, value = 0;
    std::uint8_t bytes = 1;
  };

  [[nodiscard]] static bool contains(std::uint16_t offset) noexcept;
  [[nodiscard]] bool read(std::uint16_t offset, std::uint8_t &value) const noexcept;
  [[nodiscard]] bool write(Write access) noexcept;
  [[nodiscard]] bool advance(Cycle now) noexcept;
  void activity(Activity state) noexcept;
  [[nodiscard]] std::optional<std::uint16_t> pending() const noexcept;
  [[nodiscard]] std::optional<Cycle> deadline() const noexcept;
  [[nodiscard]] bool interrupts_enabled() const noexcept;
  [[nodiscard]] bool bus_running() const noexcept;
  // Exact nominal periods/timestamps in 60 MHz units, independent of host time.
  [[nodiscard]] unsigned bus_period() const noexcept;

  [[nodiscard]] std::uint16_t usb_module_control() const noexcept { return usb_; }

  [[nodiscard]] Cycle ticks() const noexcept { return ticks_; }

  [[nodiscard]] Cycle cycles() const noexcept { return time_; }

  // Current bus-period projection; callers also honor the next clock event.
  [[nodiscard]] std::optional<Cycle> at_ticks(Cycle ticks) const noexcept;

private:
  [[nodiscard]] Cycle rti_period() const noexcept;
  [[nodiscard]] bool rti_running() const noexcept;
  bool elapsed(Cycle cycles) noexcept;
  bool write_byte(Write access) noexcept;
  bool write_usb(std::uint16_t value) noexcept;
  void lock(bool ready) noexcept;
  Cycle time_ = 0, ticks_ = 0, rti_phase_ = 0;
  std::optional<Cycle> qualification_;
  static constexpr std::uint16_t usb_reset = 0x0820;
  static constexpr std::uint8_t power_on = 0x40;
  std::uint16_t usb_ = usb_reset;
  std::uint8_t divider_ = 0, flags_ = power_on, select_ = 0, enable_ = 0, rti_ = 0, cop_ = 0;
  bool cop_written_ = false, cop_wait_written_ = false;
  Activity activity_ = Activity::running;
};
}
