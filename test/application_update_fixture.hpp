// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <iosfwd>

namespace blofeld {
struct Firmware;
class Hc12;
class McuBus;

namespace test {
// Continue natural application WAI through two native XIRQ/RAM-helper requests.
// Explicit parallel peer observations; no DSP peer or elapsed board time.
void verify_application_update(McuBus &bus,
                               Hc12 &cpu,
                               const Firmware &firmware,
                               std::ostream &trace);
}
}
