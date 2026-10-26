// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_bus.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
// Port operand only; destination writeback and pipeline timing require their own contracts.
struct DspPeripheralReadForm {
  DspAddress target;
  std::optional<unsigned> bit = std::nullopt;
  bool set = false;
  unsigned words = 1;
};

[[nodiscard]] std::optional<DspPeripheralReadForm> decode_peripheral_read(std::uint32_t opcode);
}
