// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_sp_move.hpp"

#include "dsp/dsp_control_move.hpp"
#include "dsp56300/transfers.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
std::optional<DspSpMove> decode_sp_move(const std::uint32_t opcode) {
  // FM13-131 fixes bit6 to zero; reject the legacy undocumented alias.
  constexpr unsigned undocumented_alias = 0x40;
  if ((opcode & undocumented_alias) != 0) { return std::nullopt; }
  const auto words = decode_control_move(opcode, dsp56300::RegisterId::sp);
  if (!words) { return std::nullopt; }
  return DspSpMove{.words = *words};
}
}
