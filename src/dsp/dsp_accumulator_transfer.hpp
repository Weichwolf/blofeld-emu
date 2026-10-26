// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>

namespace blofeld {
// Qualified standalone full A/B sources to X/Y/R destinations only.
[[nodiscard]] bool accumulator_transfer_source(std::uint32_t opcode);
[[nodiscard]] std::uint8_t accumulator_transfer_dependency(std::uint32_t producer,
                                                           std::uint32_t consumer);
}
