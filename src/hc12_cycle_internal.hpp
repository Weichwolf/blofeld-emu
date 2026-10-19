// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/hc12_cycles.hpp"

#include <cstdint>

namespace blofeld::detail {
[[nodiscard]] bool hc12_cycle_short_index(std::uint8_t post);
[[nodiscard]] bool hc12_cycle_branch(unsigned condition, ExecutionContext context);
[[nodiscard]] Hc12CyclePlan hc12_page2_cycle_plan(const Bus &bus, ExecutionContext context);
}
