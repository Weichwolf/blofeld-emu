// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_pipeline.hpp"
#include "dsp/dsp_accumulator_transfer.hpp"
#include "dsp/dsp_branch_form.hpp"
#include "dsp/dsp_memory_form.hpp"
#include "dsp/dsp_move_writeback.hpp"
#include "dsp/dsp_omr_move.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp56kEmu/opcodeanalysis.h"
#include "dsp56kEmu/opcodes.h"
#include "dsp56kEmu/opcodetypes.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace blofeld {
namespace {
using Type = dsp56k::Instruction;
using Mask = dsp56k::RegisterMask;
constexpr unsigned word_mask = 0xffffff;

bool immediate(const Type type, const unsigned opcode) {
  if (type == dsp56k::Move_xx || type == dsp56k::Movec_xx) { return true; }
  if (type != dsp56k::Movex_ea && type != dsp56k::Movey_ea && type != dsp56k::Movec_ea) {
    return false;
  }
  constexpr unsigned extended = 6;
  constexpr unsigned literal = 4;
  return dsp56k::getFieldValue(type, dsp56k::Field_W, opcode) != 0 &&
         dsp56k::getFieldValue(type, dsp56k::Field_MMM, opcode) == extended &&
         dsp56k::getFieldValue(type, dsp56k::Field_RRR, opcode) == literal;
}

bool transfer(const Type type, const unsigned opcode) {
  return type == dsp56k::Mover || type == dsp56k::Movec_S1D2 || immediate(type, opcode);
}

bool modifier_source(const Type first, const Type second, const std::uint32_t opcode) {
  if (first != dsp56k::Movec_S1D2 || second != dsp56k::Invalid) { return false; }
  // FM13-131/12-20: raw M0..M7 -> M0..M7, or M0..M7 -> full A/B.
  // Fixed bit6=0 excludes the SDK's undocumented alias; no other control
  // sources/destinations or partial accumulators are admitted here.
  constexpr unsigned modifier_mask = 0xfff8f8;
  constexpr unsigned modifier_to_modifier = 0x04e0a0;
  constexpr unsigned accumulator_mask = 0xfffef8;
  constexpr unsigned modifier_to_accumulator = 0x044ea0;
  return (opcode & modifier_mask) == modifier_to_modifier ||
         (opcode & accumulator_mask) == modifier_to_accumulator;
}

bool accumulator_source(const Type first, const Type second, const std::uint32_t opcode) {
  if (first != dsp56k::Mover || second != dsp56k::Invalid) { return false; }
  const auto source = dsp56k::getRegister(first, dsp56k::Field_eeeee, opcode);
  if (source != dsp56k::Register::A && source != dsp56k::Register::B) { return false; }
  auto written = Mask::None;
  auto read = Mask::None;
  dsp56k::getRegisters(written, read, first, opcode);
  constexpr auto destinations = static_cast<std::uint64_t>(Mask::X) |
                                static_cast<std::uint64_t>(Mask::Y) |
                                static_cast<std::uint64_t>(Mask::R);
  return written != Mask::None && (static_cast<std::uint64_t>(written) & ~destinations) == 0;
}

}

bool accumulator_transfer_source(const std::uint32_t opcode) {
  constexpr unsigned mask = 0xffffff;
  if (opcode > mask) { return false; }
  static const dsp56k::Opcodes opcodes;
  auto first = dsp56k::Invalid;
  auto second = dsp56k::Invalid;
  return opcodes.getInstructionTypes(opcode, first, second) != 0 &&
         accumulator_source(first, second, opcode);
}

unsigned pipeline_word_count(const DspProgramWord word) {
  static const dsp56k::Opcodes opcodes;
  Type first = dsp56k::Invalid;
  Type second = dsp56k::Invalid;
  if (opcodes.getInstructionTypes(word.value, first, second) == 0) {
    throw Fault("Undefined automatic instruction P:" + hex(word.address, 6));
  }
  const auto words = dsp56k::Opcodes::getOpcodeLength(word.value, first, second);
  if (words == 0 || words > 2) {
    throw Fault("Unqualified automatic instruction length P:" + hex(word.address, 6));
  }
  return words;
}

DspMoveFields pipeline_transfer_fields(const std::uint32_t opcode) {
  if (decode_omr_move(opcode)) { return {}; }
  static const dsp56k::Opcodes opcodes;
  Type first = dsp56k::Invalid;
  Type second = dsp56k::Invalid;
  if (opcodes.getInstructionTypes(opcode, first, second) == 0) { return {}; }
  const auto move = transfer(first, opcode) ? first : second;
  if (!transfer(move, opcode)) { return {}; }
  if (!immediate(move, opcode)) {
    Mask written = Mask::None;
    Mask read = Mask::None;
    dsp56k::getRegisters(written, read, move, opcode);
    const auto accumulator = accumulator_source(first, second, opcode) ? Mask::AB : Mask::None;
    const auto modifier = modifier_source(first, second, opcode) ? Mask::M : Mask::None;
    const auto allowed = static_cast<std::uint64_t>(accumulator) |
                         static_cast<std::uint64_t>(modifier) |
                         static_cast<std::uint64_t>(Mask::X) | static_cast<std::uint64_t>(Mask::Y) |
                         static_cast<std::uint64_t>(Mask::R);
    if ((static_cast<std::uint64_t>(read) & ~allowed) != 0) {
      throw Fault("Unqualified automatic transfer source or accumulator destination");
    }
  }
  return decode_move_writeback(opcode, DspMoveDecode::nonmemory);
}

std::uint8_t
pipeline_agu_dependencies(const std::array<std::optional<DspProgramWord>, 3> &preceding,
                          const DspProgramWord consumer) {
  const auto memory = decode_memory_stage(consumer.value);
  const auto branch = decode_branch_form(consumer.value);
  unsigned read = 0;
  if (branch && branch->address_register) {
    read = 1U << *branch->address_register;
  } else {
    if (!memory || (memory->peripheral && !is_program_transfer(*memory))) { return 0; }
    for (unsigned lane = 0; lane < memory->lane_count; ++lane) {
      read |= 1U << memory->lanes.at(lane).reg;
    }
  }
  unsigned result = 0;
  for (const auto word : preceding) {
    if (!word || decode_omr_move(word->value)) { continue; }
    // Only explicit move destinations rearm this interlock, never ordinary AG2 updates.
    const auto fields = decode_move_writeback(word->value);
    result |= read & static_cast<unsigned>(fields.r | fields.n | fields.m);
  }
  return static_cast<std::uint8_t>(result);
}

std::optional<DspProgramWord> pipeline_extension(const DspProgramFetchView &fetch,
                                                 const DspProgramWord instruction) {
  if (pipeline_word_count(instruction) == 1) { return std::nullopt; }
  const auto address = (instruction.address + 1) & word_mask;
  for (std::size_t index = 0; index < fetch.used; ++index) {
    const auto word = fetch.words.at(index);
    if (word.address == address) { return word; }
  }
  throw Fault("Automatic AG1 awaits actual extension read P:" + hex(address, 6) +
              " pc=" + hex(instruction.address, 6) + " omr=" + hex(fetch.omr, 6));
}
}
