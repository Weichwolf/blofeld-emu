// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <string>

namespace blofeld {

enum class DspPort : std::uint8_t { c, e, f };
enum class DspGpioRegister : std::uint8_t { data, direction, control };

struct DspGpioWrite {
  static constexpr std::uint32_t word_mask = 0xffffff;
  DspGpioRegister reg = DspGpioRegister::data;
  std::uint32_t value = 0;
  std::uint32_t mask = word_mask;
};

struct DspGpioDrive {
  std::uint32_t mask = 0;
  std::uint32_t levels = 0;
};

struct DspGpioSample {
  std::uint32_t known = 0;
  std::uint32_t levels = 0;
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
  void set_drive(DspGpioDrive drive);
  [[nodiscard]] DspGpioOutputs outputs() const;
  [[nodiscard]] DspGpioSample observe_pads(std::uint32_t mask) const;
  void validate_peripheral_drive(DspGpioDrive drive) const;

private:
  [[nodiscard]] std::uint32_t pin_mask() const;
  [[nodiscard]] std::string address(DspGpioRegister reg) const;
  [[nodiscard]] DspGpioSample resolve(std::uint32_t mask) const;
  void write_checked(DspGpioWrite access);
  DspPort port_;
  std::uint32_t control_ = 0;
  std::uint32_t direction_ = 0;
  std::uint32_t latch_ = 0;
  std::uint32_t latch_known_ = 0;
  DspGpioDrive external_;
};

}
