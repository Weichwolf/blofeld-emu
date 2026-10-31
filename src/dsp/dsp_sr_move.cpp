// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_sr_move.hpp"

#include "dsp/dsp_control_move.hpp"
#include "dsp56300/transfers.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
std::optional<DspSrMove> decode_sr_move(const std::uint32_t opcode) {
  const auto words = decode_control_move(opcode, dsp56300::RegisterId::sr);
  if (!words) { return std::nullopt; }
  return DspSrMove{.words = *words};
}
}
