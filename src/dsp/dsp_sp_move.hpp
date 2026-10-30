// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <cstdint>
#include <optional>

namespace blofeld {
struct DspSpMove {
  unsigned words = 0;
};

// Decoded nonmemory control write; only actual native retirement supplies SP.
[[nodiscard]] std::optional<DspSpMove> decode_sp_move(std::uint32_t opcode);
}
