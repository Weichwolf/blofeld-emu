// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <functional>
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
  unsigned before_input_instructions = 0;
  unsigned before_input_interrupts = 0;
};

using ApplicationBoundary = std::function<void(const McuMachine &, const LcdFixture &)>;

// Caller-bounded, conditional startup for a supplied unit image. The observer
// cannot mutate CPU/bus/display state. Unknown PM3 still faults and is recovered
// jointly before its explicitly declared level is observed.
ApplicationRun start_application_reference(McuMachine &machine,
                                           std::unique_ptr<LcdFixture> &display,
                                           std::ostream &trace,
                                           bool pm3_high,
                                           unsigned maximum_steps,
                                           const ApplicationBoundary &observer = {});

// Conditional native reset/application experiment; no calibrated board timing.
// Retains the exact erased-unit instruction and LCD comparison contract.
ApplicationRun start_application(McuMachine &machine,
                                 std::unique_ptr<LcdFixture> &display,
                                 std::ostream &trace,
                                 bool pm3_high);
}
}
