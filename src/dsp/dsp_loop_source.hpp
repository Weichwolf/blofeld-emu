// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <optional>

namespace blofeld {
enum class DspLoopCountSource : std::uint8_t { immediate, x0, r0, a, a0 };
// Clocked forms with independently retained source/producer timing evidence.
[[nodiscard]] std::optional<DspLoopCountSource> automatic_loop_source(std::uint32_t opcode);
}
