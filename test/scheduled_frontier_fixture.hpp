// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "scheduled_coupling_fixture.hpp"

#include <cstdint>
#include <filesystem>
#include <sstream>

namespace blofeld::test::scheduled {
void verify_scheduled_frontier(McuMachine &mcu,
                               DspMachine &dsp,
                               ClockedLink &link,
                               Progress &progress,
                               std::ostringstream &trace,
                               const std::filesystem::path &prefix,
                               std::uint64_t epoch);
}
