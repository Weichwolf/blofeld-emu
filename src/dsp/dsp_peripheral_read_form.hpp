// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_bus.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
// Owned fixed X/Y read operand:AA6 bit branches and short-I/O forms.
// Destination writeback and pipeline timing require their own contracts.
struct DspPeripheralReadForm {
  DspAddress target;
  std::optional<unsigned> bit = std::nullopt;
  bool set = false;
  bool relative = false;
  unsigned words = 1;
};

[[nodiscard]] std::optional<DspPeripheralReadForm> decode_peripheral_read(std::uint32_t opcode);
}
