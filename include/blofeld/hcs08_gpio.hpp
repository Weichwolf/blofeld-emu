// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/gpio_pads.hpp"

#include <array>
#include <cstdint>

namespace blofeld {
enum class Hcs08GpioContract : std::uint8_t { unqualified, candidate_gt8a };
enum class Hcs08GpioPort : std::uint8_t { a, b, c, d, e };

struct Hcs08GpioRegisters {
  std::uint8_t latch = 0;
  std::uint8_t pull_enable = 0;
  std::uint8_t slew_enable = 0;
  std::uint8_t direction = 0;
  bool operator==(const Hcs08GpioRegisters &) const = default;
};

// Candidate GT GPIO registers A..E. Board nets and peripheral pin ownership
// remain outside this component. Input reads require resolved observations.
class Hcs08Gpio {
public:
  explicit Hcs08Gpio(Hcs08GpioContract contract);
  void reset();
  void attach_pad_source(Hcs08GpioPort port, GpioPadReader reader);
  [[nodiscard]] Hcs08GpioRegisters state(Hcs08GpioPort port) const;
  // Only the explicitly GPIO-owned pins contribute drivers/biases. Other
  // peripheral drivers and KBI pull-down selection need their own owner.
  [[nodiscard]] GpioNetDriver driver(Hcs08GpioPort port, std::uint8_t gpio_owned) const;
  [[nodiscard]] std::uint8_t read(std::uint16_t address, ExecutionContext context) const;
  void write(ByteWrite access, ExecutionContext context);
  [[nodiscard]] static bool contains(std::uint16_t address);

private:
  static constexpr unsigned ports = 5;
  std::array<Hcs08GpioRegisters, ports> registers_{};
  std::array<GpioPadBinding, ports> pads_{};
};
} // namespace blofeld
