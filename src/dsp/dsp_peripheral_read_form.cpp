// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_peripheral_read_form.hpp"

#include "dsp/dsp_bus.hpp"
#include "dsp56300/instruction.hpp"
#include "dsp56300/transfers.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
namespace {
constexpr unsigned high_begin = 0xffffc0;
constexpr unsigned low_begin = 0xffff80;

std::optional<DspPeripheralReadForm> branch(const dsp56300::InstructionShape shape,
                                            const std::uint32_t opcode) {
  using enum dsp56300::InstructionKind;
  const bool relative = shape.kind == bit_branch;
  if (!relative && shape.kind != bit_jump) { return std::nullopt; }
  constexpr unsigned jump_high_mask = 0xfec080;
  constexpr unsigned jump_high = 0x0a8080;
  constexpr unsigned jump_low_mask = 0xff8080;
  constexpr unsigned jump_low = 0x018080;
  constexpr unsigned branch_high_mask = 0xfec080;
  constexpr unsigned branch_high = 0x0cc000;
  constexpr unsigned branch_low_mask = 0xffc000;
  constexpr unsigned branch_low = 0x048000;
  const bool high = relative ? (opcode & branch_high_mask) == branch_high
                             : (opcode & jump_high_mask) == jump_high;
  const bool low =
      relative ? (opcode & branch_low_mask) == branch_low : (opcode & jump_low_mask) == jump_low;
  constexpr unsigned bit_mask = 0x1f;
  constexpr unsigned word_bits = 24;
  const auto bit = opcode & bit_mask;
  if ((!high && !low) || bit >= word_bits) { return std::nullopt; }
  constexpr unsigned operand_shift = 8;
  constexpr unsigned operand_mask = 0x3f;
  constexpr unsigned space_bit = 0x40;
  constexpr unsigned set_bit = 0x20;
  constexpr unsigned high_call_bit = 0x10000;
  constexpr unsigned jump_low_call_bit = 0x4000;
  constexpr unsigned branch_low_call_bit = 0x80;
  auto call_bit = relative ? branch_low_call_bit : jump_low_call_bit;
  if (high) { call_bit = high_call_bit; }
  if ((opcode & call_bit) != 0) { return std::nullopt; }
  return DspPeripheralReadForm{
      .target = {.space = (opcode & space_bit) != 0 ? DspSpace::y : DspSpace::x,
                 .address =
                     (high ? high_begin : low_begin) + ((opcode >> operand_shift) & operand_mask)},
      .bit = bit,
      .set = (opcode & set_bit) != 0,
      .relative = relative,
      .words = shape.words};
}

std::optional<DspPeripheralReadForm> word(const std::uint32_t opcode) {
  const auto transfers = dsp56300::inspect_transfers(opcode);
  if (!transfers ||
      transfers->instruction.kind != dsp56300::InstructionKind::move_peripheral_register ||
      transfers->count != 1) {
    return std::nullopt;
  }
  const auto &source = transfers->moves[0].source;
  if (source.kind != dsp56300::TransferOperandKind::memory ||
      source.address.kind != dsp56300::AddressKind::absolute ||
      source.address.literal < low_begin ||
      (source.space != dsp56300::TransferSpace::x && source.space != dsp56300::TransferSpace::y)) {
    return std::nullopt;
  }
  return DspPeripheralReadForm{
      .target = {.space = source.space == dsp56300::TransferSpace::y ? DspSpace::y : DspSpace::x,
                 .address = source.address.literal}};
}
}

std::optional<DspPeripheralReadForm> decode_peripheral_read(const std::uint32_t opcode) {
  const auto shape = dsp56300::inspect_instruction(opcode);
  if (!shape) { return std::nullopt; }
  if (const auto form = branch(*shape, opcode)) { return form; }
  return word(opcode);
}
}
