// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>

namespace blofeld {
// Functional protocol edges do not assert calibrated pad/setup/hold intervals.
// NAND command, cell, ownership and scheduled operation semantics remain separate.
enum class NandTiming : std::uint8_t { unqualified, candidate_minima, functional_protocol };

[[nodiscard]] constexpr bool valid_nand_timing(const NandTiming timing) {
  return timing == NandTiming::candidate_minima || timing == NandTiming::functional_protocol;
}

[[nodiscard]] constexpr std::uint32_t nand_interval_ns(const NandTiming timing,
                                                       const std::uint32_t candidate) {
  return timing == NandTiming::functional_protocol ? 0U : candidate;
}
}
