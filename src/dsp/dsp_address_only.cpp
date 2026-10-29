// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_address_only.hpp"

#include "dsp56kEmu/opcodes.h"
#include "dsp56kEmu/opcodetypes.h"

#include <cstdint>
#include <optional>

namespace blofeld {
std::optional<DspAddressOnlyForm> decode_address_only(const std::uint32_t opcode) {
  constexpr unsigned word_mask = 0xffffff;
  if (opcode > word_mask) { return std::nullopt; }
  static const dsp56k::Opcodes opcodes;
  auto first = dsp56k::Invalid;
  auto second = dsp56k::Invalid;
  if (opcodes.getInstructionTypes(opcode, first, second) == 0 ||
      (second != dsp56k::Move_ea && first != dsp56k::Move_ea)) {
    return std::nullopt;
  }
  return DspAddressOnlyForm{
      .mode = dsp56k::getFieldValue(dsp56k::Move_ea, dsp56k::Field_MM, opcode),
      .reg = dsp56k::getFieldValue(dsp56k::Move_ea, dsp56k::Field_RRR, opcode)};
}
}
