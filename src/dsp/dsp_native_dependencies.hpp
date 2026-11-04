// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_interlocks.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
// Pure encoded dependencies for selected transfer, ALU, shift and INSERT forms.
// The caller owns base costs. Missing descriptions never imply no dependencies.
[[nodiscard]] std::optional<DspInterlockInstruction>
inspect_native_dependencies(std::uint32_t opcode, unsigned native_units);
}
