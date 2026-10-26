// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_branch_form.hpp"

#include "dsp56kEmu/opcodeanalysis.h"
#include "dsp56kEmu/opcodes.h"
#include "dsp56kEmu/opcodetypes.h"

#include <cstdint>
#include <optional>

namespace blofeld {
std::optional<DspBranchForm> decode_branch_form(const std::uint32_t opcode) {
  static const dsp56k::Opcodes opcodes;
  auto first = dsp56k::Invalid;
  auto second = dsp56k::Invalid;
  if (opcodes.getInstructionTypes(opcode, first, second) == 0 || second != dsp56k::Invalid) {
    return std::nullopt;
  }
  if (first == dsp56k::Jmp_ea) {
    constexpr unsigned direct = 4;
    if (dsp56k::getFieldValue(first, dsp56k::Field_MMM, opcode) != direct) { return std::nullopt; }
    const auto reg = dsp56k::getFieldValue(first, dsp56k::Field_RRR, opcode);
    return DspBranchForm{.address_register = static_cast<std::uint8_t>(reg)};
  }
  if (first == dsp56k::Bra_xxx) {
    constexpr unsigned sign = 0x100;
    constexpr unsigned high = 0xfffe00;
    const auto value =
        dsp56k::getFieldValue(first, dsp56k::Field_aaaa, dsp56k::Field_aaaaa, opcode);
    return DspBranchForm{.displacement = value < sign ? value : (value | high)};
  }
  if ((first != dsp56k::Jset_S && first != dsp56k::Jclr_S) ||
      dsp56k::getRegister(first, dsp56k::Field_DDDDDD, opcode) != dsp56k::Register::OMR) {
    return std::nullopt;
  }
  constexpr unsigned bits = 24;
  const auto bit = dsp56k::getFieldValue(first, dsp56k::Field_bbbbb, opcode);
  if (bit >= bits) { return std::nullopt; }
  return DspBranchForm{.bit = bit, .set = first == dsp56k::Jset_S};
}
}
