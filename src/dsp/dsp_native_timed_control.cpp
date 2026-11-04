// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_interlocks.hpp"
#include "dsp/dsp_native_timed_execution.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/instruction.hpp"
#include "dsp56300/transfers.hpp"

#include <algorithm>
#include <array>
#include <optional>

namespace blofeld {
namespace {
bool literal_bit_memory(const dsp56300::Word opcode) {
  // FM BCLR/BSET/BCHG/BTST AA6, pp6 and qq6 forms: no register or EA
  // operand. Their C-only update is not a DALU status writer (NXP controls).
  struct Form {
    unsigned mask;
    unsigned value;
  };

  constexpr std::array forms{Form{.mask = 0xfec080, .value = 0x0a0000},
                             Form{.mask = 0xfec080, .value = 0x0a8000},
                             Form{.mask = 0xff8080, .value = 0x010000}};
  constexpr unsigned bit_mask = 0x1f;
  constexpr unsigned word_bits = 24;
  if ((opcode & bit_mask) >= word_bits) { return false; }
  return std::ranges::any_of(
      forms, [opcode](const Form form) { return (opcode & form.mask) == form.value; });
}

bool omr_bit_jump(const dsp56300::Word opcode) {
  // FM JCLR/JSET #n,OMR,absolute: PCU mode bits, not DALU/SR or AGU inputs.
  constexpr unsigned operand_mask = 0xffffc0;
  constexpr unsigned omr_operand = 0x0afa00;
  constexpr unsigned bit_mask = 0x1f;
  constexpr unsigned word_bits = 24;
  return (opcode & operand_mask) == omr_operand && (opcode & bit_mask) < word_bits;
}

std::optional<unsigned> bit_register_source(const dsp56300::Word opcode) {
  // Non-call JCLR/JSET and BRCLR/BRSET register forms (FM opcode tables).
  constexpr unsigned mask = 0xffc0c0;
  constexpr unsigned absolute = 0x0ac000;
  constexpr unsigned relative = 0x0cc080;
  constexpr unsigned bit_mask = 0x1f;
  constexpr unsigned word_bits = 24;
  if (((opcode & mask) != absolute && (opcode & mask) != relative) ||
      (opcode & bit_mask) >= word_bits) {
    return std::nullopt;
  }
  constexpr unsigned selector_mask = 63;
  constexpr unsigned selector_shift = 8;
  const auto source = (opcode >> selector_shift) & selector_mask;
  using Register = dsp56300::RegisterId;
  if (source < static_cast<unsigned>(Register::x0) ||
      source > static_cast<unsigned>(Register::b1)) {
    return std::nullopt;
  }
  return source;
}

bool literal_bit_jump(const dsp56300::Word opcode) {
  // Non-call JCLR/JSET on AA6/pp6/qq6: fixed memory, no DALU/AGU operand.
  constexpr unsigned mask = 0xffc080;
  constexpr std::array families{0x0a0080U, 0x0a8080U, 0x018080U};
  constexpr unsigned bit_mask = 0x1f;
  constexpr unsigned word_bits = 24;
  return (opcode & bit_mask) < word_bits &&
         std::ranges::find(families, opcode & mask) != families.end();
}

bool literal_conditional_jump(const dsp56300::Word opcode) {
  constexpr unsigned mask = 0xff0000;
  constexpr unsigned short_jump = 0x0e0000;
  return (opcode & mask) == short_jump;
}

bool literal_conditional_branch(const dsp56300::Word opcode) {
  constexpr unsigned mask = 0xff0c20;
  constexpr unsigned short_branch = 0x050400;
  return (opcode & mask) == short_branch;
}

bool literal_bit_branch(const dsp56300::Word opcode) {
  constexpr unsigned mask = 0xffc080;
  constexpr std::array families{0x0c8080U, 0x0cc000U, 0x048000U};
  constexpr unsigned bit_mask = 0x1f;
  constexpr unsigned word_bits = 24;
  return (opcode & bit_mask) < word_bits &&
         std::ranges::find(families, opcode & mask) != families.end();
}

bool simple_control(const dsp56300::InstructionKind kind, const dsp56300::Word opcode) {
  using Kind = dsp56300::InstructionKind;
  // ANDI/ORI use three base periods, not the DALU status-write path.
  // Independent normal-model SR reads contain no additional delay.
  constexpr std::array pcu{Kind::jump_short,
                           Kind::call_short,
                           Kind::return_subroutine,
                           Kind::repeat_immediate,
                           Kind::loop_immediate,
                           Kind::wait,
                           Kind::reset_peripherals,
                           Kind::and_control,
                           Kind::or_control};
  if (std::ranges::find(pcu, kind) != pcu.end()) { return true; }
  if (kind == Kind::bit_memory) { return literal_bit_memory(opcode); }
  if (kind == Kind::bit_jump) { return omr_bit_jump(opcode) || literal_bit_jump(opcode); }
  if (kind == Kind::conditional_jump) { return literal_conditional_jump(opcode); }
  if (kind == Kind::bit_branch) { return literal_bit_branch(opcode); }
  constexpr unsigned short_mask = 0xfffc20;
  constexpr unsigned short_bra = 0x050c00;
  constexpr unsigned long_bra = 0x0d10c0;
  constexpr unsigned short_bsr = 0x050800;
  constexpr unsigned long_bsr = 0x0d1080;
  return kind == Kind::relative_branch &&
         ((opcode & short_mask) == short_bra || opcode == long_bra ||
          (opcode & short_mask) == short_bsr || opcode == long_bsr ||
          literal_conditional_branch(opcode));
}

std::optional<DspInterlockInstruction> register_control(const dsp56300::InstructionKind kind,
                                                        const dsp56300::Word opcode,
                                                        const unsigned units) {
  using Kind = dsp56300::InstructionKind;
  if (kind == Kind::bit_jump || kind == Kind::bit_branch) {
    const auto source = bit_register_source(opcode);
    if (!source) { return std::nullopt; }
    constexpr auto first_accumulator = static_cast<unsigned>(dsp56300::RegisterId::a0);
    // Each accumulator part reads its matching transfer port. Independent NXP
    // CLR/MOVE windows require one period at gap0, none after a NOP.
    const auto reads = *source >= first_accumulator ? 1U << (*source & 1U) : 0U;
    return DspInterlockInstruction{.opcode = opcode, .native_units = units, .move_reads = reads};
  }
  if (kind == Kind::loop_register || kind == Kind::repeat_register) {
    constexpr unsigned selector_mask = 63;
    constexpr unsigned selector_shift = 8;
    const auto source = (opcode >> selector_shift) & selector_mask;
    using Register = dsp56300::RegisterId;
    if (source >= static_cast<unsigned>(Register::a0) &&
        source <= static_cast<unsigned>(Register::b1)) {
      // DO/DOR/REP use each accumulator part's matching transfer port.
      return DspInterlockInstruction{
          .opcode = opcode, .native_units = units, .move_reads = 1U << (source & 1U)};
    }
    if (source >= static_cast<unsigned>(Register::x0) &&
        source <= static_cast<unsigned>(Register::y1)) {
      return DspInterlockInstruction{.opcode = opcode, .native_units = units};
    }
    return std::nullopt;
  }
  if (kind != Kind::jump_effective && kind != Kind::call_effective) { return std::nullopt; }
  constexpr unsigned mode_shift = 11;
  constexpr unsigned register_shift = 8;
  constexpr unsigned field_mask = 7;
  const auto mode = (opcode >> mode_shift) & field_mask;
  const auto reg = (opcode >> register_shift) & field_mask;
  constexpr unsigned indirect = 4;
  constexpr unsigned extended = 6;
  if (mode == indirect) {
    return DspInterlockInstruction{.opcode = opcode, .native_units = units, .agu_reads = 1U << reg};
  }
  if (mode == extended && reg == 0) {
    return DspInterlockInstruction{.opcode = opcode, .native_units = units};
  }
  return std::nullopt;
}

std::optional<DspInterlockInstruction> long_output(const dsp56300::Word opcode,
                                                   const unsigned units) {
  const auto transfers = dsp56300::inspect_transfers(opcode);
  if (!transfers || transfers->count != 1 || transfers->parallel_alu != 0) { return std::nullopt; }
  const auto &source = transfers->moves.front().source;
  const auto &destination = transfers->moves.front().destination;
  using Operand = dsp56300::TransferOperandKind;
  if (source.kind != Operand::register_value || source.reg != dsp56300::RegisterId::a ||
      destination.kind != Operand::memory ||
      destination.space != dsp56300::TransferSpace::long_pair ||
      destination.address.kind != dsp56300::AddressKind::post_increment) {
    return std::nullopt;
  }
  const auto inputs = dsp56300::address_inputs(destination.address);
  if (!inputs) { return std::nullopt; }
  // L:A uses the 48-bit limiter and SR port, even without a flag change.
  // Independent NXP data/AGU/status windows retain the actual Core effects.
  return DspInterlockInstruction{.opcode = opcode,
                                 .native_units = units,
                                 .agu_reads = inputs->r,
                                 .move_reads = 1,
                                 .status_write = true};
}

std::optional<DspInterlockInstruction> program_memory(const dsp56300::Word opcode,
                                                      const unsigned units) {
  const auto transfers = dsp56300::inspect_transfers(opcode);
  if (!transfers || transfers->count != 1) { return std::nullopt; }
  const auto &source = transfers->moves.front().source;
  const auto &destination = transfers->moves.front().destination;
  using Operand = dsp56300::TransferOperandKind;
  using Space = dsp56300::TransferSpace;
  using Register = dsp56300::RegisterId;
  using Address = dsp56300::AddressKind;
  const bool load = source.kind == Operand::memory && source.space == Space::program;
  const auto &memory = load ? source : destination;
  const auto &reg = load ? destination : source;
  if (memory.kind != Operand::memory || memory.space != Space::program ||
      reg.kind != Operand::register_value) {
    return std::nullopt;
  }
  const bool ordinary = reg.reg >= Register::x0 && reg.reg <= Register::y1;
  const bool accumulator = reg.reg == Register::a1;
  if (!ordinary && (load || !accumulator)) { return std::nullopt; }
  constexpr std::array addresses{
      Address::indirect, Address::post_increment, Address::absolute_extension};
  if (std::ranges::find(addresses, memory.address.kind) == addresses.end()) { return std::nullopt; }
  const auto inputs = dsp56300::address_inputs(memory.address);
  if (!inputs) { return std::nullopt; }
  // NXP windows qualify the AGU/transfer ports and the extra absolute-address
  // read phase. Core still owns every P read/write and code invalidation.
  return DspInterlockInstruction{
      .opcode = opcode,
      .native_units = units,
      .agu_reads = inputs->r,
      .move_reads = !load && accumulator ? 1U : 0U,
      .move_read_advance = !load && memory.address.kind == Address::absolute_extension ? 1U : 0U,
      .partial_status_source = !load && accumulator};
}

std::optional<DspInterlockInstruction> program_input(const dsp56300::Word opcode,
                                                     const unsigned units) {
  const auto transfers = dsp56300::inspect_transfers(opcode);
  if (!transfers || transfers->count != 1) { return std::nullopt; }
  const auto &source = transfers->moves.front().source;
  const auto &destination = transfers->moves.front().destination;
  using Operand = dsp56300::TransferOperandKind;
  using Space = dsp56300::TransferSpace;
  using Address = dsp56300::AddressKind;
  constexpr unsigned peripheral_begin = 0xffff80;
  if (source.kind != Operand::memory || source.address.kind != Address::absolute ||
      source.address.literal < peripheral_begin ||
      (source.space != Space::x && source.space != Space::y) ||
      destination.kind != Operand::memory || destination.space != Space::program ||
      destination.address.kind != Address::post_increment) {
    return std::nullopt;
  }
  const auto inputs = dsp56300::address_inputs(destination.address);
  if (!inputs) { return std::nullopt; }
  // The real Core/bus still owns the peripheral read, P write, post-update
  // and code-view invalidation. This describes the AGU input only.
  return DspInterlockInstruction{.opcode = opcode, .native_units = units, .agu_reads = inputs->r};
}
}

bool DspNativeTimedExecution::conditional(const dsp56300::Word opcode) {
  const auto shape = dsp56300::inspect_instruction(opcode);
  if (!shape) { return false; }
  using Kind = dsp56300::InstructionKind;
  if (shape->kind == Kind::conditional_jump) { return literal_conditional_jump(opcode); }
  if (shape->kind == Kind::relative_branch) { return literal_conditional_branch(opcode); }
  if (shape->kind == Kind::bit_jump) {
    return omr_bit_jump(opcode) || literal_bit_jump(opcode) || bit_register_source(opcode);
  }
  return shape->kind == Kind::bit_branch &&
         (literal_bit_branch(opcode) || bit_register_source(opcode));
}

std::optional<DspInterlockInstruction> DspNativeTimedExecution::control(const dsp56300::Word opcode,
                                                                        const unsigned units) {
  const auto shape = dsp56300::inspect_instruction(opcode);
  if (!shape) { return std::nullopt; }
  if (simple_control(shape->kind, opcode)) {
    return DspInterlockInstruction{.opcode = opcode, .native_units = units};
  }
  if (const auto result = register_control(shape->kind, opcode, units)) { return result; }
  if (shape->kind == dsp56300::InstructionKind::parallel) {
    if (const auto result = long_output(opcode, units)) { return result; }
  }
  if (shape->kind == dsp56300::InstructionKind::move_program_effective) {
    return program_memory(opcode, units);
  }
  if (shape->kind == dsp56300::InstructionKind::move_peripheral_program) {
    return program_input(opcode, units);
  }
  return std::nullopt;
}
}
