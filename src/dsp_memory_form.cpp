// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_memory_form.hpp"

#include "blofeld/dsp_bus.hpp"
#include "dsp56kEmu/opcodeanalysis.h"
#include "dsp56kEmu/opcodes.h"
#include "dsp56kEmu/opcodetypes.h"

#include <cstdint>
#include <optional>

namespace blofeld {
namespace {
using Type = dsp56k::Instruction;
using Reg = dsp56k::Register;

bool data_register(const Reg reg) {
  return reg == Reg::X0 || reg == Reg::X1 || reg == Reg::Y0 || reg == Reg::Y1;
}

bool memory(const Type type) {
  return type == dsp56k::Movex_ea || type == dsp56k::Movey_ea || type == dsp56k::Movel_ea ||
         type == dsp56k::Movexy;
}

DspMemoryOperation
operation(const Type type, const dsp56k::Field field, const std::uint32_t opcode) {
  return dsp56k::getFieldValue(type, field, opcode) != 0 ? DspMemoryOperation::read
                                                         : DspMemoryOperation::write;
}

std::optional<DspMemoryForm> single(const Type type, const std::uint32_t opcode) {
  if (!data_register(dsp56k::getRegister(type, dsp56k::Field_ddd, opcode))) { return std::nullopt; }
  const auto mode = dsp56k::getFieldValue(type, dsp56k::Field_MMM, opcode);
  if (mode > 4) { return std::nullopt; }
  DspMemoryForm result;
  result.lanes[0] = {.space = type == dsp56k::Movex_ea ? DspSpace::x : DspSpace::y,
                     .operation = operation(type, dsp56k::Field_W, opcode),
                     .mode = mode,
                     .reg = dsp56k::getFieldValue(type, dsp56k::Field_RRR, opcode)};
  return result;
}

std::optional<DspMemoryForm> long_pair(const std::uint32_t opcode) {
  constexpr auto type = dsp56k::Movel_ea;
  const auto mode = dsp56k::getFieldValue(type, dsp56k::Field_MMM, opcode);
  const auto access = operation(type, dsp56k::Field_W, opcode);
  const auto field = dsp56k::getFieldValue(type, dsp56k::Field_L, dsp56k::Field_LL, opcode);
  // Full-accumulator store S/L phase placement remains unqualified.
  if (mode > 4 || (access == DspMemoryOperation::write && field >= 4)) { return std::nullopt; }
  DspMemoryForm result{.long_pair = true};
  result.lanes[0] = {.operation = access,
                     .mode = mode,
                     .reg = dsp56k::getFieldValue(type, dsp56k::Field_RRR, opcode)};
  return result;
}

std::optional<DspMemoryForm> dual(const std::uint32_t opcode) {
  constexpr auto type = dsp56k::Movexy;
  if (!data_register(dsp56k::getRegister(type, dsp56k::Field_ee, opcode)) ||
      !data_register(dsp56k::getRegister(type, dsp56k::Field_ff, opcode))) {
    return std::nullopt;
  }
  const auto second = dsp56k::getRegister(type, dsp56k::Field_rr, opcode);
  if (second < Reg::R0 || second > Reg::R7) { return std::nullopt; }
  DspMemoryForm result{.lane_count = 2};
  result.lanes[0] = {.operation = operation(type, dsp56k::Field_W, opcode),
                     .mode = dsp56k::getFieldValue(type, dsp56k::Field_MM, opcode),
                     .reg = dsp56k::getFieldValue(type, dsp56k::Field_RRR, opcode),
                     .dual_encoding = true};
  result.lanes[1] = {.space = DspSpace::y,
                     .operation = operation(type, dsp56k::Field_w, opcode),
                     .mode = dsp56k::getFieldValue(type, dsp56k::Field_mm, opcode),
                     .reg = static_cast<unsigned>(second) - static_cast<unsigned>(Reg::R0),
                     .dual_encoding = true};
  return result;
}
}

std::optional<DspMemoryForm> decode_memory_stage(const std::uint32_t opcode) {
  static const dsp56k::Opcodes opcodes;
  auto first = dsp56k::Invalid;
  auto second = dsp56k::Invalid;
  if (opcodes.getInstructionTypes(opcode, first, second) == 0 ||
      dsp56k::Opcodes::getOpcodeLength(opcode, first, second) != 1) {
    return std::nullopt;
  }
  const auto type = memory(first) ? first : second;
  if (type == dsp56k::Movex_ea || type == dsp56k::Movey_ea) { return single(type, opcode); }
  if (type == dsp56k::Movel_ea) { return long_pair(opcode); }
  if (type == dsp56k::Movexy) { return dual(opcode); }
  return std::nullopt;
}
}
