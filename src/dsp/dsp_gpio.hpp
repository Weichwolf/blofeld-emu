// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "runtime/gpio_pads.hpp"

#include <cstdint>
#include <string>

namespace blofeld {

enum class DspPort : std::uint8_t { c, e, f, d };
enum class DspGpioRegister : std::uint8_t { data, direction, control };
enum class DspGpioContract : std::uint8_t { strict, logical_pins };

struct DspGpioWrite {
  static constexpr std::uint32_t word_mask = 0xffffff;
  DspGpioRegister reg = DspGpioRegister::data;
  std::uint32_t value = 0;
  std::uint32_t mask = word_mask;
};

struct DspGpioDrive {
  std::uint32_t mask = 0;
  std::uint32_t levels = 0;
  // Explicitly undriven pins; absent mask/floating bits remain unobserved.
  std::uint32_t floating = 0;
  std::uint32_t unknown = 0; // Strong peer drivers with an unobserved level.
};

struct DspGpioSample {
  std::uint32_t known = 0;
  std::uint32_t levels = 0;
  std::uint32_t conflicts = 0;
};

struct DspGpioOutputs {
  std::uint32_t mask = 0;
  std::uint32_t known = 0;
  std::uint32_t levels = 0;
  std::uint32_t peripheral = 0;
};

// Logical DSP56371 GPIO. External levels are observations, not board defaults.
class DspGpio {
public:
  explicit DspGpio(DspPort port);
  void reset();
  [[nodiscard]] std::uint32_t read(DspGpioRegister reg,
                                   std::uint32_t mask = DspGpioWrite::word_mask) const;
  void write(DspGpioWrite access);
  // Functional pins: no upper-bit storage and no BIU wait-state qualification.
  void set_contract(DspGpioContract contract);

  [[nodiscard]] DspGpioContract contract() const noexcept { return contract_; }

  void set_drive(DspGpioDrive drive);
  // Replace only this source's declared pins, retaining unrelated observations.
  void update_drive(std::uint32_t mask, DspGpioDrive drive);
  void attach_pad_source(GpioPadReader reader);
  [[nodiscard]] DspGpioOutputs outputs() const;
  [[nodiscard]] DspGpioSample observe_pads(std::uint32_t mask) const;
  void validate_peripheral_drive(DspGpioDrive drive) const;

private:
  [[nodiscard]] std::uint32_t pin_mask() const;
  [[nodiscard]] std::string address(DspGpioRegister reg) const;
  [[nodiscard]] DspGpioSample resolve(std::uint32_t mask) const;
  [[nodiscard]] DspGpioSample resolve_local(std::uint32_t mask) const;
  void write_checked(DspGpioWrite access);
  DspPort port_;
  DspGpioContract contract_ = DspGpioContract::strict;
  std::uint32_t control_ = 0;
  std::uint32_t direction_ = 0;
  std::uint32_t latch_ = 0;
  std::uint32_t latch_known_ = 0;
  DspGpioDrive external_;
  GpioPadBinding pads_;
};

}
