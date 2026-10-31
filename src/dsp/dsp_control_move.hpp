// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/transfers.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
// Qualifies only the supported control-register sources, never the value moved.
[[nodiscard]] std::optional<unsigned> decode_control_move(std::uint32_t opcode,
                                                          dsp56300::RegisterId destination);
}
