// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <optional>

namespace blofeld {
struct DspOmrMove {
  unsigned words = 0;
};

// A decoded control write; the actual native retirement owns its OMR value.
[[nodiscard]] std::optional<DspOmrMove> decode_omr_move(std::uint32_t opcode);
}
