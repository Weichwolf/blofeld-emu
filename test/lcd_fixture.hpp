// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "display/st7565.hpp"
#include "display/st7565_parallel.hpp"

#include <cstdint>

namespace blofeld {
class McuBus;
struct BusEvent;

namespace test {
struct LcdCounts {
  unsigned commands = 0;
  unsigned writes = 0;
  unsigned reads = 0;
};

// Inferred routing is an explicit test fixture, not a default board profile.
class LcdFixture {
public:
  explicit LcdFixture(McuBus &bus);
  // Value-copy the controlled external fixture while rebinding its bus observer.
  LcdFixture(McuBus &bus, const LcdFixture &source);
  void observe(const BusEvent &event);

  void withdraw_data() { withdraw_ = true; }

  [[nodiscard]] LcdCounts counts() const { return counts_; }

  [[nodiscard]] const St7565 &controller() const { return device_.controller(); }

private:
  static bool gpio_address(std::uint16_t address);
  [[nodiscard]] LcdHostPins sample() const;
  McuBus &bus_;
  St7565Parallel device_;
  LcdCounts counts_;
  bool armed_ = false;
  bool withdraw_ = false;
};
}
}
