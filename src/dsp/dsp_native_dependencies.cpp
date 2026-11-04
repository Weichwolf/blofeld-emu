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
}

std::optional<DspInterlockInstruction> inspect_native_dependencies(const std::uint32_t opcode,
                                                                   const unsigned native_units) {
  constexpr unsigned maximum_units = 1048576;
  if (native_units == 0 || native_units > maximum_units) { return std::nullopt; }
  const auto shape = dsp56300::inspect_instruction(opcode);
  if (!shape) { return std::nullopt; }
  DspInterlockInstruction result{.opcode = opcode, .native_units = native_units};
  if (shape->kind == dsp56300::InstructionKind::bit_field_insert) {
    result.arithmetic_writes = (opcode & 1U) != 0 ? 2U : 1U;
    result.status_write = true;
    return result;
  }
  const auto transfers = dsp56300::inspect_transfers(opcode);
  if (!transfers || transfers->address_update) { return std::nullopt; }
  if (transfers->instruction.kind == dsp56300::InstructionKind::parallel) {
    const auto alu = transfers->parallel_alu;
    constexpr unsigned clear_a = 0x13;
    constexpr unsigned destination_b = 8;
    constexpr unsigned product = 0x80;
    if (alu != 0) {
      if (alu != clear_a && alu != (clear_a | destination_b) && (alu & product) == 0) {
        return std::nullopt;
      }
      result.arithmetic_writes = (alu & destination_b) != 0 ? 2U : 1U;
      result.status_write = true;
    }
  }
  for (unsigned index = 0; index < transfers->count; ++index) {
    if (!describe_move(result, transfers->moves.at(index))) { return std::nullopt; }
  }
  return result;
}
}
