// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_pipeline.hpp"
#include "dsp/dsp_memory_form.hpp"
#include "dsp/dsp_move_writeback.hpp"
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
    constexpr auto allowed =
        static_cast<std::uint64_t>(Mask::X) | static_cast<std::uint64_t>(Mask::Y);
    if ((static_cast<std::uint64_t>(read) & ~allowed) != 0) {
      throw Fault("Unqualified automatic transfer source; only X/Y data registers are admitted");
    }
  }
  return decode_move_writeback(opcode, DspMoveDecode::nonmemory);
}

std::uint8_t
pipeline_agu_dependencies(const std::array<std::optional<DspProgramWord>, 3> &preceding,
                          const DspProgramWord consumer) {
  const auto memory = decode_memory_stage(consumer.value);
  if (!memory || memory->peripheral) { return 0; }
  unsigned read = 0;
  for (unsigned lane = 0; lane < memory->lane_count; ++lane) {
    read |= 1U << memory->lanes.at(lane).reg;
  }
  unsigned result = 0;
  for (const auto word : preceding) {
    if (!word) { continue; }
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
