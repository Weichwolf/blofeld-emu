// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <iosfwd>

namespace blofeld {
class Hc12;
class McuBus;

namespace test {
class LcdFixture;

struct ApplicationRun {
  unsigned instructions = 0;
  unsigned interrupts = 0;
  unsigned oscillator_periods = 0;
};

// Conditional native reset/application experiment; no calibrated board timing.
ApplicationRun
start_application(McuBus &bus, Hc12 &cpu, LcdFixture &display, std::ostream &trace, bool pm3_high);
}
}
