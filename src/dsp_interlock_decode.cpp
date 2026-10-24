// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "dsp56kEmu/opcodeanalysis.h"
#include "dsp56kEmu/opcodes.h"
#include "dsp56kEmu/opcodetypes.h"
#include "dsp_interlocks.hpp"

#include <cstdint>

namespace blofeld {
namespace {
using Mask = dsp56k::RegisterMask;
using Instruction = dsp56k::Instruction;
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned register_set_mask = 0xff;

unsigned groups(const Mask mask) {
  const auto bits = static_cast<std::uint64_t>(mask);
  constexpr auto r = static_cast<unsigned>(dsp56k::Register::R0);
  constexpr auto n = static_cast<unsigned>(dsp56k::Register::N0);
  constexpr auto m = static_cast<unsigned>(dsp56k::Register::M0);
  return static_cast<unsigned>(((bits >> r) | (bits >> n) | (bits >> m)) & register_set_mask);
}

unsigned accumulators(const Mask mask) {
  return (dsp56k::any(mask, Mask::A) ? 1U : 0U) | (dsp56k::any(mask, Mask::B) ? 2U : 0U);
}

Mask operand(const Instruction type, const dsp56k::Field field, const std::uint32_t opcode) {
  const auto reg = dsp56k::getRegister(type, field, opcode);
  if (reg == dsp56k::Register::Invalid) { throw Fault("Undefined DSP interlock register operand"); }
  constexpr auto allowed =
      static_cast<std::uint64_t>(Mask::AB) | static_cast<std::uint64_t>(Mask::X) |
      static_cast<std::uint64_t>(Mask::Y) | static_cast<std::uint64_t>(Mask::R) |
      static_cast<std::uint64_t>(Mask::N) | static_cast<std::uint64_t>(Mask::M) |
      static_cast<std::uint64_t>(Mask::SR) | static_cast<std::uint64_t>(Mask::OMR);
  const auto mask = dsp56k::convert(reg);
  if ((static_cast<std::uint64_t>(mask) & ~allowed) != 0) {
    throw Fault("Unqualified DSP interlock control-register operand");
  }
  return mask;
}

struct Move {
  Mask read = Mask::None;
  Mask write = Mask::None;
  unsigned agu = 0;
  unsigned units = 1;
  unsigned read_advance = 0;
  bool immediate = false;
};

void effective(Move &move, const Instruction type, const std::uint32_t opcode) {
  const auto mode = dsp56k::getFieldValue(type, dsp56k::Field_MMM, opcode);
  const auto reg = dsp56k::getFieldValue(type, dsp56k::Field_RRR, opcode);
  if (mode == 5 || mode == 7 || (mode == 6 && reg != 0 && reg != 4)) {
    throw Fault("Unqualified DSP interlock effective-address timing");
  }
  if (mode == 6) {
    move.units = 2;
    move.immediate = reg == 4;
  } else {
    move.agu = 1U << reg;
  }
}

enum class MoveFormat : std::uint8_t {
  empty,
  immediate,
  registers,
  control_registers,
  control_immediate,
  effective,
  absolute
};

MoveFormat format(const Instruction type) {
  if (type == dsp56k::Invalid || type == dsp56k::Nop || type == dsp56k::Move_Nop) {
    return MoveFormat::empty;
  }
  if (type == dsp56k::Move_xx) { return MoveFormat::immediate; }
  if (type == dsp56k::Mover) { return MoveFormat::registers; }
  if (type == dsp56k::Movec_S1D2) { return MoveFormat::control_registers; }
  if (type == dsp56k::Movec_xx) { return MoveFormat::control_immediate; }
  if (type == dsp56k::Movex_ea || type == dsp56k::Movey_ea || type == dsp56k::Movec_ea) {
    return MoveFormat::effective;
  }
  if (type == dsp56k::Movex_aa || type == dsp56k::Movey_aa || type == dsp56k::Movec_aa) {
    return MoveFormat::absolute;
  }
  throw Fault("Unqualified DSP interlock move format");
}

Move memory_move(const Instruction type, const std::uint32_t opcode, const bool has_ea) {
  Move result;
  if (has_ea) { effective(result, type, opcode); }
  const bool load = dsp56k::getFieldValue(type, dsp56k::Field_W, opcode) != 0;
  const bool control = type == dsp56k::Movec_ea || type == dsp56k::Movec_aa;
  const auto reg = operand(type, control ? dsp56k::Field_DDDDD : dsp56k::Field_ddd, opcode);
  if (load) {
    result.write = reg;
  } else {
    result.read = reg;
  }
  if (!load && result.immediate) { throw Fault("Invalid DSP interlock immediate store"); }
  if (!load && (type == dsp56k::Movex_aa || type == dsp56k::Movey_aa)) {
    throw Fault("Unqualified DSP interlock short-store nominal cost");
  }
  // A long data-address extension places accumulator sampling one clock later.
  // Independent MPY->long-store windows contain no additional dependency clock.
  if (!load) { result.read_advance = result.units - 1; }
  return result;
}

Move decode_move(const Instruction type, const std::uint32_t opcode) {
  Move result;
  switch (format(type)) {
    case MoveFormat::empty: return result;
    case MoveFormat::immediate:
      result.write = operand(type, dsp56k::Field_ddddd, opcode);
      result.immediate = true;
      return result;
    case MoveFormat::registers:
      result.read = operand(type, dsp56k::Field_eeeee, opcode);
      result.write = operand(type, dsp56k::Field_ddddd, opcode);
      return result;
    case MoveFormat::control_registers: {
      const auto control = operand(type, dsp56k::Field_DDDDD, opcode);
      const auto data = operand(type, dsp56k::Field_eeeeee, opcode);
      const bool to_control = dsp56k::getFieldValue(type, dsp56k::Field_W, opcode) != 0;
      result.read = to_control ? data : control;
      result.write = to_control ? control : data;
      return result;
    }
    case MoveFormat::control_immediate:
      result.write = operand(type, dsp56k::Field_DDDDD, opcode);
      result.immediate = true;
      return result;
    case MoveFormat::effective: return memory_move(type, opcode, true);
    case MoveFormat::absolute: return memory_move(type, opcode, false);
    default: throw Fault("Invalid DSP interlock move format");
  }
}
}

DspInterlockInstruction decode_interlock_instruction(const std::uint32_t opcode,
                                                     const unsigned native_units) {
  static const dsp56k::Opcodes opcodes;
  auto first = dsp56k::Invalid;
  auto second = dsp56k::Invalid;
  if (opcode > word_mask || opcodes.getInstructionTypes(opcode, first, second) == 0 ||
      first == dsp56k::Invalid) {
    throw Fault("Undefined DSP interlock opcode=" + hex(opcode, 6));
  }
  if (first == dsp56k::Btst_D && second == dsp56k::Invalid) {
    if (operand(first, dsp56k::Field_DDDDDD, opcode) != Mask::SR || native_units != 2) {
      throw Fault("Unqualified DSP interlock register bit-test timing");
    }
    return {.opcode = opcode, .native_units = native_units, .status_read = true};
  }
  unsigned arithmetic = 0;
  Instruction move_type = first;
  if (first == dsp56k::Mpy_S1S2D || first == dsp56k::Clr) {
    arithmetic = dsp56k::getFieldValue(first, dsp56k::Field_d, opcode) != 0 ? 2U : 1U;
    if (second != dsp56k::Invalid && second != dsp56k::Move_Nop) {
      throw Fault("Unqualified DSP interlock parallel arithmetic/move timing");
    }
    move_type = second;
  } else if (first == dsp56k::Nop && second != dsp56k::Invalid) {
    move_type = second;
  } else if (second != dsp56k::Invalid && second != dsp56k::Move_Nop) {
    throw Fault("Unqualified DSP interlock arithmetic opcode=" + hex(opcode, 6));
  }
  const auto move = decode_move(move_type, opcode);
  if (move.units != native_units) {
    throw Fault("DSP interlock nominal cost differs from its independent instruction form");
  }
  const bool whole = move.read == Mask::A || move.read == Mask::B;
  return {.opcode = opcode,
          .native_units = native_units,
          .agu_reads = move.agu,
          .agu_writes = groups(move.write),
          .move_reads = accumulators(move.read),
          .move_writes = accumulators(move.write),
          .arithmetic_writes = arithmetic,
          .agu_write_delay = move.immediate ? 4 - native_units : 3,
          .transfer_write_delay = move.immediate ? 2 - native_units : 1,
          .move_read_advance = move.read_advance,
          .status_write = arithmetic != 0 || whole,
          .partial_status_source = accumulators(move.read) != 0 && !whole,
          .status_read = dsp56k::any(move.read, Mask::SR)};
}
}
