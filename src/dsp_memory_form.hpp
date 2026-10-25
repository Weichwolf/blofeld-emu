// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_bus.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
struct DspMemoryForm {
  DspSpace space = DspSpace::x;
  DspMemoryOperation operation = DspMemoryOperation::read;
  unsigned mode = 0;
  unsigned reg = 0;
};

[[nodiscard]] std::optional<DspMemoryForm> decode_memory_stage(std::uint32_t opcode);
}
