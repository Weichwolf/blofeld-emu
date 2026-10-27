// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_memory_form.hpp"

#include "dsp/dsp_bus.hpp"
#include "dsp56kEmu/opcodeanalysis.h"
#include "dsp56kEmu/opcodes.h"
#include "dsp56kEmu/opcodetypes.h"

#include <cstdint>
#include <optional>

namespace blofeld {
namespace {
using Type = dsp56k::Instruction;
using Reg = dsp56k::Register;
constexpr unsigned high_io_begin = 0xffffc0;
constexpr unsigned low_io_begin = 0xffff80;

bool data_register(const Reg reg) {
  return reg == Reg::X0 || reg == Reg::X1 || reg == Reg::Y0 || reg == Reg::Y1;
}

bool address_register(const Reg reg) {
  return (reg >= Reg::R0 && reg <= Reg::R7) || (reg >= Reg::N0 && reg <= Reg::N7) ||
         (reg >= Reg::M0 && reg <= Reg::M7);
}

bool memory(const Type type) {
  return type == dsp56k::Movex_ea || type == dsp56k::Movey_ea || type == dsp56k::Movel_ea ||
         type == dsp56k::Movexy || type == dsp56k::Movec_ea;
}

std::optional<DspMemoryForm> peripheral_immediate(const Type type, const unsigned opcode) {
  if (type != dsp56k::Movep_ppea && type != dsp56k::Movep_Xqqea && type != dsp56k::Movep_Yqqea) {
    return std::nullopt;
  }
  if (dsp56k::getFieldValue(type, dsp56k::Field_W, opcode) == 0 ||
      dsp56k::getFieldValue(type, dsp56k::Field_MMM, opcode) != 6 ||
      dsp56k::getFieldValue(type, dsp56k::Field_RRR, opcode) != 4) {
    return std::nullopt;
  }
  const bool high = type == dsp56k::Movep_ppea;
  const auto field = high ? dsp56k::Field_pppppp : dsp56k::Field_qqqqqq;
  const bool y = type == dsp56k::Movep_Yqqea ||
                 (high && dsp56k::getFieldValue(type, dsp56k::Field_s, opcode) != 0);
  return DspMemoryForm{.peripheral =
                           DspAddress{.space = y ? DspSpace::y : DspSpace::x,
                                      .address = (high ? high_io_begin : low_io_begin) +
                                                 dsp56k::getFieldValue(type, field, opcode)}};
}

std::optional<DspMemoryForm> program_transfer(const Type type, const unsigned opcode) {
  if (type != dsp56k::Movep_eapp && type != dsp56k::Movep_eaqq) { return std::nullopt; }
  const auto mode = dsp56k::getFieldValue(type, dsp56k::Field_MMM, opcode);
  if (dsp56k::getFieldValue(type, dsp56k::Field_W, opcode) != 0 || mode > 4) {
    return std::nullopt;
  }
  const bool high = type == dsp56k::Movep_eapp;
  const auto field = high ? dsp56k::Field_pppppp : dsp56k::Field_qqqqqq;
  const bool y = dsp56k::getFieldValue(type, high ? dsp56k::Field_s : dsp56k::Field_S, opcode) != 0;
  DspMemoryForm result{.peripheral =
                           DspAddress{.space = y ? DspSpace::y : DspSpace::x,
                                      .address = (high ? high_io_begin : low_io_begin) +
                                                 dsp56k::getFieldValue(type, field, opcode)}};
  result.lanes[0] = {.space = DspSpace::p,
                     .operation = DspMemoryOperation::write,
                     .mode = mode,
                     .reg = dsp56k::getFieldValue(type, dsp56k::Field_RRR, opcode)};
  return result;
}

std::optional<DspMemoryForm> peripheral_register(const Type type, const unsigned opcode) {
  if (type != dsp56k::Movep_Spp && type != dsp56k::Movep_SXqq && type != dsp56k::Movep_SYqq) {
    return std::nullopt;
  }
  const auto source = dsp56k::getRegister(type, dsp56k::Field_dddddd, opcode);
  if (dsp56k::getFieldValue(type, dsp56k::Field_W, opcode) == 0 ||
      (!data_register(source) && !address_register(source))) {
    return std::nullopt;
  }
  const bool high = type == dsp56k::Movep_Spp;
  const bool y = type == dsp56k::Movep_SYqq ||
                 (high && dsp56k::getFieldValue(type, dsp56k::Field_s, opcode) != 0);
  const auto offset =
      high ? dsp56k::getFieldValue(type, dsp56k::Field_pppppp, opcode)
           : dsp56k::getFieldValue(type, dsp56k::Field_q, dsp56k::Field_qqqqq, opcode);
  return DspMemoryForm{.peripheral =
                           DspAddress{.space = y ? DspSpace::y : DspSpace::x,
                                      .address = (high ? high_io_begin : low_io_begin) + offset},
                       .peripheral_register = true};
}

DspMemoryOperation
operation(const Type type, const dsp56k::Field field, const std::uint32_t opcode) {
  return dsp56k::getFieldValue(type, field, opcode) != 0 ? DspMemoryOperation::read
                                                         : DspMemoryOperation::write;
}

std::optional<DspMemoryForm> single(const Type type, const std::uint32_t opcode) {
  const bool control = type == dsp56k::Movec_ea;
  const auto reg =
      dsp56k::getRegister(type, control ? dsp56k::Field_DDDDD : dsp56k::Field_ddd, opcode);
  const auto access = operation(type, dsp56k::Field_W, opcode);
  if (!data_register(reg) && (!address_register(reg) || access != DspMemoryOperation::read) &&
      !full_accumulator_scalar_store(opcode)) {
    return std::nullopt;
  }
  const auto mode = dsp56k::getFieldValue(type, dsp56k::Field_MMM, opcode);
  if (mode > 4) { return std::nullopt; }
  DspMemoryForm result;
  const bool y = type == dsp56k::Movey_ea ||
                 (control && dsp56k::getFieldValue(type, dsp56k::Field_S, opcode) != 0);
  result.lanes[0] = {.space = y ? DspSpace::y : DspSpace::x,
                     .operation = access,
                     .mode = mode,
                     .reg = dsp56k::getFieldValue(type, dsp56k::Field_RRR, opcode)};
  return result;
}

std::optional<DspMemoryForm> long_pair(const std::uint32_t opcode) {
  constexpr auto type = dsp56k::Movel_ea;
  const auto mode = dsp56k::getFieldValue(type, dsp56k::Field_MMM, opcode);
  const auto access = operation(type, dsp56k::Field_W, opcode);
  const auto field = dsp56k::getFieldValue(type, dsp56k::Field_L, dsp56k::Field_LL, opcode);
  // FM13-126/127: A/B use a single signed 48-bit transfer; AB/BA have two
  // independent 24-bit limits and remain separately unqualified here.
  const bool store = access == DspMemoryOperation::write;
  const bool full_accumulator = store && (field == 4 || field == 5);
  constexpr unsigned arithmetic_bits = 0xff;
  if (mode > 4 || (store && field >= 6) || (full_accumulator && (opcode & arithmetic_bits) != 0)) {
    return std::nullopt;
  }
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

bool full_accumulator_long_store(const std::uint32_t opcode) {
  // FM13-127: 0100L0LL W1MMMRRR, with LLL=100/101 and W=0.
  constexpr unsigned source_mask = 0xfec000;
  constexpr unsigned full_accumulator_store = 0x484000;
  return (opcode & source_mask) == full_accumulator_store;
}

bool full_accumulator_scalar_store(const std::uint32_t opcode) {
  // FM13-118/122: 01ddSddd W1MMMRRR, ddddd=01110/01111, W=0.
  // S selects X/Y; only standalone transfers (no parallel ALU) are qualified.
  constexpr unsigned source_mask = 0xf6c0ff;
  constexpr unsigned full_accumulator_store = 0x564000;
  return (opcode & source_mask) == full_accumulator_store;
}

bool is_program_transfer(const DspMemoryForm &form) {
  return form.peripheral.has_value() && form.lanes[0].space == DspSpace::p;
}

std::optional<DspMemoryForm> decode_memory_stage(const std::uint32_t opcode) {
  static const dsp56k::Opcodes opcodes;
  auto first = dsp56k::Invalid;
  auto second = dsp56k::Invalid;
  if (opcodes.getInstructionTypes(opcode, first, second) == 0) { return std::nullopt; }
  if (const auto program = program_transfer(first, opcode)) { return program; }
  if (const auto peripheral = peripheral_immediate(first, opcode)) { return peripheral; }
  if (const auto peripheral = peripheral_register(first, opcode)) { return peripheral; }
  if (dsp56k::Opcodes::getOpcodeLength(opcode, first, second) != 1) { return std::nullopt; }
  const auto type = memory(first) ? first : second;
  if (type == dsp56k::Movex_ea || type == dsp56k::Movey_ea || type == dsp56k::Movec_ea) {
    return single(type, opcode);
  }
  if (type == dsp56k::Movel_ea) { return long_pair(opcode); }
  if (type == dsp56k::Movexy) { return dual(opcode); }
  return std::nullopt;
}
}
