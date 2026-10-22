// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <iosfwd>
#include <memory>

namespace blofeld {
struct Firmware;
class McuMachine;

namespace test {
class LcdFixture;
// Continue natural application WAI through two native XIRQ/RAM-helper requests.
// Explicit parallel peer observations; no DSP peer or elapsed board time.
void verify_application_update(McuMachine &machine,
                               std::unique_ptr<LcdFixture> &display,
                               const Firmware &firmware,
                               std::ostream &trace);
}
}
