// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <iosfwd>

namespace blofeld {
class Hc12;
class McuBus;

namespace test {
void verify_application_midi(McuBus &bus, Hc12 &cpu, std::ostream &trace);
}
}
