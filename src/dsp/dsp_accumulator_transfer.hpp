// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>

namespace blofeld {
struct DspInstructionDependency;
// Qualified standalone or parallel signed-MPY full A/B sources to X/Y/R only.
[[nodiscard]] bool accumulator_transfer_source(std::uint32_t opcode);
// Raw middle words preserve all24 bits and bypass full-accumulator limiting.
[[nodiscard]] bool middle_accumulator_transfer_source(std::uint32_t opcode);
[[nodiscard]] std::uint8_t accumulator_transfer_dependency(DspInstructionDependency instructions);
}
