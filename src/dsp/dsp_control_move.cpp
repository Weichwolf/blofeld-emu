// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_control_move.hpp"

#include "dsp56300/instruction.hpp"
#include "dsp56300/transfers.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
namespace {
bool supported_register(const dsp56300::RegisterId reg) {
  using enum dsp56300::RegisterId;
  return (reg >= x0 && reg <= y1) || (reg >= r0 && reg <= r7);
}
}

std::optional<unsigned> decode_control_move(const std::uint32_t opcode,
                                            const dsp56300::RegisterId destination) {
  const auto transfer = dsp56300::inspect_transfers(opcode);
  if (!transfer || transfer->count != 1) { return std::nullopt; }
  const auto &move = transfer->moves[0];
  using enum dsp56300::TransferOperandKind;
  using enum dsp56300::InstructionKind;
  if (move.destination.kind != register_value || move.destination.reg != destination) {
    return std::nullopt;
  }
  if (transfer->instruction.kind == move_control_immediate && move.source.kind == immediate) {
    return transfer->instruction.words;
  }
  if (transfer->instruction.kind == move_control_register && move.source.kind == register_value &&
      supported_register(move.source.reg)) {
    return transfer->instruction.words;
  }
  if (transfer->instruction.kind == move_control_effective && move.source.kind == extension) {
    return transfer->instruction.words;
  }
  return std::nullopt;
}
}
