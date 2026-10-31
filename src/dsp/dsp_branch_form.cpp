// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_branch_form.hpp"

#include "dsp56300/instruction.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
namespace {
constexpr std::uint32_t word_mask = 0xffffff;
constexpr std::uint32_t byte_prefix_mask = 0xff0000;
constexpr std::uint32_t short_relative_prefix = 0x050000;
constexpr std::uint32_t short_conditional_jump_prefix = 0x0e0000;
constexpr std::uint32_t long_relative_call = 0x0d1080;
constexpr std::uint32_t high_offset_mask = 0x1e0;
constexpr std::uint32_t low_offset_mask = 0x1f;
constexpr std::uint32_t offset_mask = 0x1ff;
constexpr std::uint32_t short_address_mask = 0xfff;
constexpr std::uint32_t relative_control_mask = 0xc00;
constexpr std::uint32_t bit_jump_prefix = 0x0a0000;
constexpr unsigned word_bits = 24;

std::optional<DspBranchForm> effective_branch(const std::uint32_t opcode, const bool call) {
  const auto mode = (opcode >> 11U) & 7U;
  const auto reg = (opcode >> 8U) & 7U;
  if (mode == 6 && reg == 0) {
    return DspBranchForm{.absolute_extension = true, .subroutine_call = call};
  }
  if (mode != 4) { return std::nullopt; }
  return DspBranchForm{.address_register = static_cast<std::uint8_t>(reg), .subroutine_call = call};
}

std::optional<DspBranchForm> decode_relative_branch(const std::uint32_t opcode) {
  if (opcode == long_relative_call) {
    return DspBranchForm{.relative_extension = true, .subroutine_call = true};
  }
  if ((opcode & byte_prefix_mask) != short_relative_prefix) { return std::nullopt; }
  constexpr unsigned unconditional = 0x800;
  constexpr unsigned no_call = 0x400;
  if ((opcode & unconditional) == 0 && (opcode & no_call) == 0) { return std::nullopt; }
  auto offset = ((opcode >> 1U) & high_offset_mask) | (opcode & low_offset_mask);
  if ((offset & 0x100U) != 0) { offset |= word_mask & ~offset_mask; }
  return DspBranchForm{.displacement = offset, .subroutine_call = (opcode & no_call) == 0};
}

std::optional<DspBranchForm> omr_bit_jump(const std::uint32_t opcode) {
  constexpr unsigned register_kind = 3;
  constexpr unsigned omr_selector = 0x3a;
  const auto source_kind = (opcode >> 14U) & 3U;
  const auto selector = (opcode >> 8U) & 0x3fU;
  const auto bit = opcode & 0x1fU;
  if ((opcode & byte_prefix_mask) != bit_jump_prefix || source_kind != register_kind ||
      selector != omr_selector || bit >= word_bits) {
    return std::nullopt;
  }
  return DspBranchForm{.bit = bit, .set = (opcode & 0x20U) != 0};
}
}

std::optional<DspBranchForm> decode_branch_form(const std::uint32_t opcode) {
  const auto instruction = dsp56300::inspect_instruction(opcode);
  if (!instruction) { return std::nullopt; }
  using enum dsp56300::InstructionKind;
  if (instruction->kind == jump_effective) { return effective_branch(opcode, false); }
  if (instruction->kind == call_effective) { return effective_branch(opcode, true); }
  if (instruction->kind == return_subroutine) { return DspBranchForm{.subroutine_return = true}; }
  if (instruction->kind == call_short) {
    return DspBranchForm{.absolute_short = opcode & short_address_mask, .subroutine_call = true};
  }
  if (instruction->kind == conditional_jump &&
      (opcode & byte_prefix_mask) == short_conditional_jump_prefix) {
    return DspBranchForm{.absolute_short = opcode & short_address_mask};
  }
  if (instruction->kind == relative_branch) { return decode_relative_branch(opcode); }
  if (instruction->kind == bit_jump) { return omr_bit_jump(opcode); }
  return std::nullopt;
}

std::optional<unsigned> branch_condition_code(const std::uint32_t opcode) {
  const auto instruction = dsp56300::inspect_instruction(opcode);
  if (!instruction) { return std::nullopt; }
  const auto prefix = opcode & byte_prefix_mask;
  if ((instruction->kind == dsp56300::InstructionKind::conditional_jump &&
       prefix == short_conditional_jump_prefix) ||
      (instruction->kind == dsp56300::InstructionKind::relative_branch &&
       prefix == short_relative_prefix && (opcode & relative_control_mask) == 0x400U)) {
    return (opcode >> 12U) & 0xfU;
  }
  return std::nullopt;
}
}
