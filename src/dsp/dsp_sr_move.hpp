// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <cstdint>
#include <optional>

namespace blofeld {
struct DspSrMove {
  unsigned words = 0;
};

// Decoded nonmemory control write; only actual native retirement supplies SR.
[[nodiscard]] std::optional<DspSrMove> decode_sr_move(std::uint32_t opcode);
}
