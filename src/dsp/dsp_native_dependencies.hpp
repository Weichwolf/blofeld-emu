// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_interlocks.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
// Pure encoded dependencies for qualified MOVE, MPY/CLR and INSERT families.
// The caller owns base costs. Missing descriptions never imply no dependencies.
[[nodiscard]] std::optional<DspInterlockInstruction>
inspect_native_dependencies(std::uint32_t opcode, unsigned native_units);
}
