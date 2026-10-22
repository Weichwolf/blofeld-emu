// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <iosfwd>
#include <memory>

namespace blofeld {
class McuMachine;

namespace test {
class LcdFixture;

struct ApplicationRun {
  unsigned instructions = 0;
  unsigned interrupts = 0;
  unsigned oscillator_periods = 0;
};

// Conditional native reset/application experiment; no calibrated board timing.
ApplicationRun start_application(McuMachine &machine,
                                 std::unique_ptr<LcdFixture> &display,
                                 std::ostream &trace,
                                 bool pm3_high);
}
}
