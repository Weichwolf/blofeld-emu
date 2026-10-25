// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_program_fetch.hpp"
#include "dsp56kEmu/opcodeanalysis.h"
#include "dsp56kEmu/opcodes.h"
#include "dsp56kEmu/opcodetypes.h"
#include "dsp_memory_form.hpp"
#include "dsp_move_writeback.hpp"
#include "dsp_pipeline.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>

namespace blofeld {
namespace {
using Type = dsp56k::Instruction;
// FM5.3/Table5-1 and A-1: normal single-word register ALU operations.
constexpr std::array allowed{
    dsp56k::Nop,       dsp56k::Abs,       dsp56k::Add_SD,     dsp56k::Add_xx,    dsp56k::Addl,
    dsp56k::Addr,      dsp56k::And_SD,    dsp56k::And_xx,     dsp56k::Asl_D,     dsp56k::Asl_ii,
    dsp56k::Asl_S1S2D, dsp56k::Asr_D,     dsp56k::Asr_ii,     dsp56k::Asr_S1S2D, dsp56k::Clr,
    dsp56k::Cmp_S1S2,  dsp56k::Cmp_xxS2,  dsp56k::Cmpm_S1S2,  dsp56k::Cmpu_S1S2, dsp56k::Eor_SD,
    dsp56k::Eor_xx,    dsp56k::Lsl_D,     dsp56k::Lsl_ii,     dsp56k::Lsl_SD,    dsp56k::Lsr_D,
    dsp56k::Lsr_ii,    dsp56k::Lsr_SD,    dsp56k::Mac_S1S2,   dsp56k::Macr_S1S2, dsp56k::Max,
    dsp56k::Maxm,      dsp56k::Mpy_S1S2D, dsp56k::Mpyr_S1S2D, dsp56k::Neg,       dsp56k::Not,
    dsp56k::Or_SD,     dsp56k::Or_xx,     dsp56k::Rnd,        dsp56k::Sub_SD,    dsp56k::Sub_xx,
    dsp56k::Subl,      dsp56k::Subr,      dsp56k::Tfr,        dsp56k::Tst,       dsp56k::Inc,
    dsp56k::Dec};
using Mask = dsp56k::RegisterMask;

struct Operands {
  Mask written = Mask::None;
  Mask read = Mask::None;
};

Operands operands(const DspProgramWord word, const bool arithmetic) {
  static const dsp56k::Opcodes opcodes;
  Type first = dsp56k::Invalid;
  Type second = dsp56k::Invalid;
  Operands result;
  if (opcodes.getInstructionTypes(word.value, first, second) == 0) { return result; }
  if (arithmetic) {
    if (std::ranges::contains(allowed, first)) {
      dsp56k::getRegisters(result.written, result.read, first, word.value);
    }
  } else {
    static_cast<void>(opcodes.getRegisters(result.written, result.read, word.value));
  }
  return result;
}

std::uint8_t dependency(const Mask written, const Mask read) {
  const bool a = dsp56k::any(written, Mask::A) && dsp56k::any(read, Mask::A);
  const bool b = dsp56k::any(written, Mask::B) && dsp56k::any(read, Mask::B);
  return static_cast<std::uint8_t>((a ? 1U : 0U) | (b ? 2U : 0U));
}

}

std::uint8_t deferred_alu_accumulators(const DspProgramWord producer,
                                       const DspProgramWord consumer) {
  return dependency(operands(producer, true).written, operands(consumer, true).read);
}

std::uint8_t memory_source_dependency(const DspInstructionDependency instructions) {
  const auto producer = instructions.producer;
  const auto consumer = instructions.consumer;
  static const dsp56k::Opcodes opcodes;
  Type first = dsp56k::Invalid;
  Type second = dsp56k::Invalid;
  if (opcodes.getInstructionTypes(consumer.value, first, second) == 0) { return 0; }
  const auto move = first == dsp56k::Movel_ea ? first : second;
  if (move != dsp56k::Movel_ea ||
      dsp56k::getFieldValue(move, dsp56k::Field_W, consumer.value) != 0) {
    return 0;
  }
  Mask written = Mask::None;
  Mask read = Mask::None;
  dsp56k::getRegisters(written, read, move, consumer.value);
  return dependency(operands(producer, false).written, read);
}

void validate_alu_pipeline_word(const DspProgramWord word,
                                const std::uint32_t omr,
                                const bool memory) {
  static const dsp56k::Opcodes opcodes;
  Type first = dsp56k::Invalid;
  Type second = dsp56k::Invalid;
  constexpr unsigned width = 6;
  const bool decoded = opcodes.getInstructionTypes(word.value, first, second) != 0;
  const bool arithmetic = std::ranges::contains(allowed, first);
  const bool standalone_memory = memory && second == dsp56k::Invalid;
  if (!decoded || (!arithmetic && !standalone_memory) ||
      (!memory && second != dsp56k::Invalid && second != dsp56k::Move_Nop) ||
      dsp56k::Opcodes::getOpcodeLength(word.value, first, second) != 1) {
    throw Fault("Unqualified automatic DSP decode P:" + hex(word.address, width) +
                " opcode=" + hex(word.value, width) + " pc=" + hex(word.address, width) +
                " omr=" + hex(omr, width));
  }
  if (memory) {
    try {
      if (!decode_memory_stage(word.value)) {
        throw Fault("Automatic memory decode lost its native form");
      }
      static_cast<void>(decode_move_writeback(word.value));
    } catch (const std::exception &error) {
      throw Fault("Unqualified automatic DSP decode P:" + hex(word.address, width) +
                  " opcode=" + hex(word.value, width) + " pc=" + hex(word.address, width) +
                  " omr=" + hex(omr, width) + ": " + error.what());
    }
  }
}
}
