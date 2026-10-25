// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_reference_execution.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
enum class DspMoveDecode : std::uint8_t { all, nonmemory };

struct DspMoveWritebackState {
  std::optional<DspMoveWriteback> pending;
  DspMoveFields published;
};

[[nodiscard]] DspMoveFields decode_move_writeback(std::uint32_t opcode,
                                                  DspMoveDecode scope = DspMoveDecode::all);
}
