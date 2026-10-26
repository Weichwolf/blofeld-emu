// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_move_writeback.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp56kEmu/opcodeanalysis.h"
#include "dsp56kEmu/opcodes.h"
#include "dsp56kEmu/opcodetypes.h"

#include <cstdint>

namespace blofeld {
namespace {
using Mask = dsp56k::RegisterMask;
using Type = dsp56k::Instruction;

bool peripheral(const Type type) {
  return type == dsp56k::Movep_Spp || type == dsp56k::Movep_SXqq || type == dsp56k::Movep_SYqq;
}

bool memory(const Type type) {
  return peripheral(type) || type == dsp56k::Movex_ea || type == dsp56k::Movex_aa ||
         type == dsp56k::Movey_ea || type == dsp56k::Movey_aa || type == dsp56k::Movec_ea ||
         type == dsp56k::Movec_aa || type == dsp56k::Movel_ea || type == dsp56k::Movel_aa ||
         type == dsp56k::Movexy;
}

bool transfer(const Type type) {
  return memory(type) || type == dsp56k::Move_xx || type == dsp56k::Mover ||
         type == dsp56k::Movec_xx || type == dsp56k::Movec_S1D2;
}

bool memory_operand(const Type type, const std::uint32_t opcode) {
  if (!memory(type)) { return false; }
  if (type == dsp56k::Movex_ea || type == dsp56k::Movey_ea || type == dsp56k::Movec_ea) {
    constexpr unsigned extended = 6;
    constexpr unsigned immediate = 4;
    if (dsp56k::getFieldValue(type, dsp56k::Field_MMM, opcode) == extended &&
        dsp56k::getFieldValue(type, dsp56k::Field_RRR, opcode) == immediate) {
      return false;
    }
  }
  return true;
}

Mask operand(const Type type, const dsp56k::Field field, const std::uint32_t opcode) {
  const auto reg = dsp56k::getRegister(type, field, opcode);
  if (reg == dsp56k::Register::Invalid) {
    throw Fault("Undefined native move-writeback destination");
  }
  return dsp56k::convert(reg);
}

Mask destination(const Type type, const std::uint32_t opcode) {
  if (peripheral(type)) {
    if (dsp56k::getFieldValue(type, dsp56k::Field_W, opcode) != 0) { return Mask::None; }
    const auto reg = dsp56k::getRegister(type, dsp56k::Field_dddddd, opcode);
    if (reg == dsp56k::Register::A || reg == dsp56k::Register::B) {
      throw Fault("Unqualified full accumulator peripheral-load writeback");
    }
    return operand(type, dsp56k::Field_dddddd, opcode);
  }

  if (type == dsp56k::Movexy) {
    const auto x = dsp56k::getFieldValue(type, dsp56k::Field_W, opcode) != 0
                       ? operand(type, dsp56k::Field_ee, opcode)
                       : Mask::None;
    const auto y = dsp56k::getFieldValue(type, dsp56k::Field_w, opcode) != 0
                       ? operand(type, dsp56k::Field_ff, opcode)
                       : Mask::None;
    return x | y;
  }
  if (memory(type)) {
    if (dsp56k::getFieldValue(type, dsp56k::Field_W, opcode) == 0) { return Mask::None; }
    if (type == dsp56k::Movel_ea || type == dsp56k::Movel_aa) {
      return operand(type, dsp56k::Field_LL, opcode);
    }
    const bool control = type == dsp56k::Movec_ea || type == dsp56k::Movec_aa;
    return operand(type, control ? dsp56k::Field_DDDDD : dsp56k::Field_ddd, opcode);
  }
  if (type == dsp56k::Move_xx || type == dsp56k::Mover) {
    return operand(type, dsp56k::Field_ddddd, opcode);
  }
  if (type == dsp56k::Movec_xx) { return operand(type, dsp56k::Field_DDDDD, opcode); }
  const bool to_control = dsp56k::getFieldValue(type, dsp56k::Field_W, opcode) != 0;
  return operand(type, to_control ? dsp56k::Field_DDDDD : dsp56k::Field_eeeeee, opcode);
}

Mask source(const Type type, const std::uint32_t opcode) {
  if (peripheral(type)) {
    return dsp56k::getFieldValue(type, dsp56k::Field_W, opcode) != 0
               ? operand(type, dsp56k::Field_dddddd, opcode)
               : Mask::None;
  }

  if (type == dsp56k::Movexy) {
    const auto x = dsp56k::getFieldValue(type, dsp56k::Field_W, opcode) == 0
                       ? operand(type, dsp56k::Field_ee, opcode)
                       : Mask::None;
    const auto y = dsp56k::getFieldValue(type, dsp56k::Field_w, opcode) == 0
                       ? operand(type, dsp56k::Field_ff, opcode)
                       : Mask::None;
    return x | y;
  }
  if (memory(type)) {
    if (dsp56k::getFieldValue(type, dsp56k::Field_W, opcode) != 0) { return Mask::None; }
    if (type == dsp56k::Movel_ea || type == dsp56k::Movel_aa) {
      return operand(type, dsp56k::Field_LL, opcode);
    }
    const bool control = type == dsp56k::Movec_ea || type == dsp56k::Movec_aa;
    return operand(type, control ? dsp56k::Field_DDDDD : dsp56k::Field_ddd, opcode);
  }
  if (type == dsp56k::Mover) { return operand(type, dsp56k::Field_eeeee, opcode); }
  if (type == dsp56k::Movec_S1D2) {
    const bool to_control = dsp56k::getFieldValue(type, dsp56k::Field_W, opcode) != 0;
    return operand(type, to_control ? dsp56k::Field_eeeeee : dsp56k::Field_DDDDD, opcode);
  }
  return Mask::None;
}

std::uint8_t group(const Mask mask, const dsp56k::Register first, const unsigned count) {
  const auto bits = static_cast<std::uint64_t>(mask) >> static_cast<unsigned>(first);
  return static_cast<std::uint8_t>(bits & ((std::uint64_t{1} << count) - 1));
}

DspMoveFields fields(const Mask mask) {
  constexpr auto allowed =
      static_cast<std::uint64_t>(Mask::X) | static_cast<std::uint64_t>(Mask::Y) |
      static_cast<std::uint64_t>(Mask::AB) | static_cast<std::uint64_t>(Mask::R) |
      static_cast<std::uint64_t>(Mask::N) | static_cast<std::uint64_t>(Mask::M);
  if ((static_cast<std::uint64_t>(mask) & ~allowed) != 0) {
    throw Fault("Unqualified native move-writeback control destination");
  }
  using Reg = dsp56k::Register;
  return {.x = group(mask, Reg::X0, 2),
          .y = group(mask, Reg::Y0, 2),
          .a = group(mask, Reg::A0, 3),
          .b = group(mask, Reg::B0, 3),
          .r = group(mask, Reg::R0, 8),
          .n = group(mask, Reg::N0, 8),
          .m = group(mask, Reg::M0, 8)};
}
}

DspMoveFields decode_move_writeback(const std::uint32_t opcode, const DspMoveDecode scope) {
  static const dsp56k::Opcodes opcodes;
  auto first = dsp56k::Invalid;
  auto second = dsp56k::Invalid;
  if (opcodes.getInstructionTypes(opcode, first, second) == 0) {
    throw Fault("Undefined native move-writeback opcode=" + hex(opcode, 6));
  }
  const auto move = transfer(first) ? first : second;
  if (!transfer(move)) {
    if (second != dsp56k::Invalid && second != dsp56k::Move_Nop) {
      throw Fault("Unqualified native move-writeback format opcode=" + hex(opcode, 6));
    }
    return {};
  }
  if (scope == DspMoveDecode::nonmemory && memory_operand(move, opcode)) { return {}; }
  if (dsp56k::any(source(move, opcode), Mask::SSH)) {
    throw Fault("Unqualified native move-writeback SSH source side effects");
  }
  const auto mask = destination(move, opcode);
  const auto result = fields(mask);
  if (move != first && first != dsp56k::Nop) {
    auto written = Mask::None;
    auto read = Mask::None;
    dsp56k::getRegisters(written, read, first, opcode);
    if ((result.a != 0 && dsp56k::any(written, Mask::A)) ||
        (result.b != 0 && dsp56k::any(written, Mask::B))) {
      throw Fault("Unqualified overlapping native ALU/move-writeback accumulator");
    }
  }
  return result;
}
}
