// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "display/st7565.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {

namespace st7565_pin {
constexpr unsigned cs1_n = 1;
constexpr unsigned cs2 = 2;
constexpr unsigned enable = 4;
constexpr unsigned read = 8;
constexpr unsigned data = 16;
constexpr unsigned reset_n = 32;
constexpr unsigned all = 63;
}

struct LcdHostPins {
  std::uint8_t control = 0;
  std::uint8_t known = 0;
  std::uint8_t drive = 0;
  LcdByte data{};
};

struct LcdCycle {
  bool read = false;
  bool data = false;
  LcdByte byte{};
};

struct LcdBusResult {
  std::uint8_t drive = 0;
  LcdByte data{};
  std::optional<LcdCycle> completed;
};

// Logical 6800 bus samples with stable input phases, separate from PCB routing.
// Elapsed access/setup/hold/reset timings still require external qualification.
class St7565Parallel {
public:
  LcdBusResult sample(LcdHostPins pins);

  [[nodiscard]] const St7565 &controller() const { return controller_; }

private:
  static void validate(LcdHostPins pins);
  [[nodiscard]] static bool enabled(LcdHostPins pins);
  void begin_cycle(LcdHostPins pins);
  [[nodiscard]] LcdCycle finish_cycle(LcdHostPins pins);
  [[nodiscard]] LcdBusResult output(std::optional<LcdCycle> completed) const;
  St7565 controller_;
  LcdHostPins previous_;
  LcdByte output_;
  bool active_ = false;
  bool reset_seen_ = false;
  bool in_reset_ = false;
};

}
