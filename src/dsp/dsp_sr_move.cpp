// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_sr_move.hpp"

#include "dsp56kEmu/opcodeanalysis.h"
#include "dsp56kEmu/opcodes.h"
#include "dsp56kEmu/opcodetypes.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>

namespace blofeld {
std::optional<DspSrMove> decode_sr_move(const std::uint32_t opcode) {
  static const dsp56k::Opcodes opcodes;
  auto first = dsp56k::Invalid;
  auto second = dsp56k::Invalid;
  constexpr unsigned word_mask = 0xffffff;
  if (opcode > word_mask || opcodes.getInstructionTypes(opcode, first, second) == 0 ||
      second != dsp56k::Invalid) {
    return std::nullopt;
  }
  if (first != dsp56k::Movec_S1D2 && first != dsp56k::Movec_xx && first != dsp56k::Movec_ea) {
    return std::nullopt;
  }
  if (dsp56k::getRegister(first, dsp56k::Field_DDDDD, opcode) != dsp56k::Register::SR) {
    return std::nullopt;
  }
  if (first == dsp56k::Movec_xx) { return DspSrMove{.words = 1}; }
  if (dsp56k::getFieldValue(first, dsp56k::Field_W, opcode) == 0) { return std::nullopt; }
  if (first == dsp56k::Movec_ea) {
    constexpr unsigned extended = 6;
    constexpr unsigned literal = 4;
    if (dsp56k::getFieldValue(first, dsp56k::Field_MMM, opcode) == extended &&
        dsp56k::getFieldValue(first, dsp56k::Field_RRR, opcode) == literal) {
      return DspSrMove{.words = 2};
    }
    return std::nullopt;
  }
  using Reg = dsp56k::Register;
  constexpr std::array sources{Reg::X0,
                               Reg::X1,
                               Reg::Y0,
                               Reg::Y1,
                               Reg::R0,
                               Reg::R1,
                               Reg::R2,
                               Reg::R3,
                               Reg::R4,
                               Reg::R5,
                               Reg::R6,
                               Reg::R7};
  if (std::ranges::contains(sources, dsp56k::getRegister(first, dsp56k::Field_eeeeee, opcode))) {
    return DspSrMove{.words = 1};
  }
  return std::nullopt;
}
}
