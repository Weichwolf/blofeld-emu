// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_native_dependencies.hpp"

#include "dsp/dsp_interlocks.hpp"
#include "dsp56300/instruction.hpp"
#include "dsp56300/transfers.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>

namespace blofeld {
namespace {
using Register = dsp56300::RegisterId;
using Operand = dsp56300::TransferOperand;
using OperandKind = dsp56300::TransferOperandKind;

unsigned accumulator(const Register reg) {
  constexpr std::array first{Register::a0, Register::a1, Register::a2, Register::a, Register::a10};
  constexpr std::array second{Register::b0, Register::b1, Register::b2, Register::b, Register::b10};
  if (std::ranges::find(first, reg) != first.end()) { return 1; }
  if (std::ranges::find(second, reg) != second.end()) { return 2; }
  return reg == Register::ab || reg == Register::ba ? 3U : 0U;
}

unsigned agu(const Register reg) {
  const auto value = static_cast<unsigned>(reg);
  const auto first = static_cast<unsigned>(Register::r0);
  const auto last = static_cast<unsigned>(Register::m7);
  constexpr unsigned groups = 8;
  return value >= first && value <= last ? 1U << ((value - first) % groups) : 0;
}

bool describe_address(DspInterlockInstruction &result, const Operand &operand) {
  if (operand.kind != OperandKind::memory) { return true; }
  if (operand.space == dsp56300::TransferSpace::program ||
      operand.space == dsp56300::TransferSpace::long_pair ||
      operand.address.kind == dsp56300::AddressKind::pre_decrement ||
      operand.address.kind == dsp56300::AddressKind::indexed) {
    return false;
  }
  const auto inputs = dsp56300::address_inputs(operand.address);
  if (!inputs) { return false; }
  result.agu_reads |= inputs->r;
  return true;
}

bool describe_move(DspInterlockInstruction &result, const dsp56300::EncodedTransfer &move) {
  const auto &source = move.source;
  const auto &destination = move.destination;
  if (!describe_address(result, source) || !describe_address(result, destination)) { return false; }
  if (source.kind == OperandKind::register_value) {
    result.move_reads |= accumulator(source.reg);
    result.status_read |= source.reg == Register::sr;
    const bool whole = source.reg == Register::a || source.reg == Register::b;
    result.status_write |= whole;
    result.partial_status_source |= accumulator(source.reg) != 0 && !whole;
  }
  if (destination.kind == OperandKind::register_value) {
    result.move_writes |= accumulator(destination.reg);
    result.agu_writes |= agu(destination.reg);
    const bool immediate =
        source.kind == OperandKind::immediate || source.kind == OperandKind::extension;
    result.agu_write_delay = immediate ? 4U - std::min(result.native_units, 4U) : 3U;
    result.transfer_write_delay = immediate ? 2U - std::min(result.native_units, 2U) : 1U;
  }
  if (destination.kind == OperandKind::memory &&
      (destination.address.kind == dsp56300::AddressKind::absolute_extension ||
       destination.address.kind == dsp56300::AddressKind::displacement_extension)) {
    result.move_read_advance = 1;
  }
  return true;
}

bool scalar_alu(const dsp56300::InstructionKind kind) {
  using Kind = dsp56300::InstructionKind;
  return kind == Kind::immediate_alu || kind == Kind::arithmetic_shift_immediate ||
         kind == Kind::arithmetic_shift_register || kind == Kind::logical_shift_immediate ||
         kind == Kind::logical_shift_register || kind == Kind::bit_field_insert;
}

std::optional<DspInterlockInstruction> describe_scalar(DspInterlockInstruction result,
                                                       const dsp56300::InstructionKind kind) {
  using Kind = dsp56300::InstructionKind;
  const auto opcode = result.opcode;
  if (kind == Kind::immediate_alu) {
    const auto function = opcode & 7U;
    constexpr unsigned compare = 5;
    if (function == 1 || function == 7) { return std::nullopt; }
    if (function != compare) { result.arithmetic_writes = (opcode & 8U) != 0 ? 2U : 1U; }
  } else {
    const bool register_shift =
        kind == Kind::arithmetic_shift_register || kind == Kind::logical_shift_register;
    // Count selectors 0/1 are reserved; a zero immediate still uses the DALU.
    const auto selector = (opcode >> 1) & 7U;
    if (register_shift && selector < 2) { return std::nullopt; }
    constexpr unsigned accumulator_bits = 56;
    constexpr unsigned count_mask = 63;
    if (kind == Kind::arithmetic_shift_immediate &&
        ((opcode >> 1) & count_mask) >= accumulator_bits) {
      return std::nullopt;
    }
    result.arithmetic_writes = (opcode & 1U) != 0 ? 2U : 1U;
  }
  result.status_write = true;
  return result;
}

bool describe_parallel(DspInterlockInstruction &result, const unsigned alu) {
  constexpr unsigned product = 0x80;
  constexpr unsigned test = 3;
  constexpr unsigned compare = 5;
  constexpr unsigned compare_magnitude = 7;
  constexpr unsigned maximum = 0x1d;
  constexpr unsigned maximum_magnitude = 0x15;
  if (alu == 0) { return true; }
  if (alu == 4 || alu == 8 || alu == 12) { return false; }
  const auto source = (alu >> 4) & 7U;
  const auto operation = alu & 7U;
  const bool flags_only =
      alu < product &&
      ((alu & ~8U) == test ||
       ((source == 0 || source >= 4) && (operation == compare || operation == compare_magnitude)));
  if (!flags_only) {
    const bool writes_b = (alu & 8U) != 0 || alu == maximum || alu == maximum_magnitude;
    result.arithmetic_writes = writes_b ? 2U : 1U;
  }
  // Busy resources, not architectural changes: TFR still occupies the SR path;
  // MAX/MAXM and false IFcc/IFcc.U retain their result-path dependencies.
  result.status_write = true;
  return true;
}
}

std::optional<DspInterlockInstruction> inspect_native_dependencies(const std::uint32_t opcode,
                                                                   const unsigned native_units) {
  constexpr unsigned maximum_units = 1048576;
  if (native_units == 0 || native_units > maximum_units) { return std::nullopt; }
  const auto shape = dsp56300::inspect_instruction(opcode);
  if (!shape) { return std::nullopt; }
  DspInterlockInstruction result{.opcode = opcode, .native_units = native_units};
  if (scalar_alu(shape->kind)) { return describe_scalar(result, shape->kind); }
  const auto transfers = dsp56300::inspect_transfers(opcode);
  if (!transfers || transfers->address_update) { return std::nullopt; }
  if (transfers->instruction.kind == dsp56300::InstructionKind::parallel) {
    if (!describe_parallel(result, transfers->parallel_alu)) { return std::nullopt; }
  }
  for (unsigned index = 0; index < transfers->count; ++index) {
    if (!describe_move(result, transfers->moves.at(index))) { return std::nullopt; }
  }
  return result;
}
}
