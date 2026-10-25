// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_program_fetch.hpp"
#include "dsp56kEmu/opcodeanalysis.h"
#include "dsp56kEmu/opcodes.h"
#include "dsp56kEmu/opcodetypes.h"
#include "dsp_pipeline.hpp"

#include <algorithm>
#include <array>
#include <cstdint>

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
}

std::uint8_t deferred_alu_accumulators(const DspProgramWord producer,
                                       const DspProgramWord consumer) {
  static const dsp56k::Opcodes opcodes;
  using Mask = dsp56k::RegisterMask;
  auto written = Mask::None;
  auto read = Mask::None;
  auto ignored = Mask::None;
  // The native return value reports nonempty masks; NOP legitimately returns false.
  static_cast<void>(opcodes.getRegisters(written, ignored, producer.value));
  static_cast<void>(opcodes.getRegisters(ignored, read, consumer.value));
  const bool a = dsp56k::any(written, Mask::A) && dsp56k::any(read, Mask::A);
  const bool b = dsp56k::any(written, Mask::B) && dsp56k::any(read, Mask::B);
  return static_cast<std::uint8_t>((a ? 1U : 0U) | (b ? 2U : 0U));
}

void validate_alu_pipeline_word(const DspProgramWord word, const std::uint32_t omr) {
  static const dsp56k::Opcodes opcodes;
  Type first = dsp56k::Invalid;
  Type second = dsp56k::Invalid;
  constexpr unsigned width = 6;
  if (opcodes.getInstructionTypes(word.value, first, second) == 0 ||
      !std::ranges::contains(allowed, first) ||
      (second != dsp56k::Invalid && second != dsp56k::Move_Nop) ||
      dsp56k::Opcodes::getOpcodeLength(word.value, first, second) != 1) {
    throw Fault("Unqualified automatic DSP decode P:" + hex(word.address, width) +
                " opcode=" + hex(word.value, width) + " pc=" + hex(word.address, width) +
                " omr=" + hex(omr, width));
  }
}
}
