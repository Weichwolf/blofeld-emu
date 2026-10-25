// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_interrupt_timing.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp56kEmu/opcodeanalysis.h"
#include "dsp56kEmu/opcodecycles.h"
#include "dsp56kEmu/opcodes.h"
#include "dsp56kEmu/opcodetypes.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace blofeld {
unsigned nominal_vector_cycles(const DspInterruptVector &vector,
                               const dsp56k::Opcodes &opcodes,
                               const std::uint32_t external_memory) {
  if ((vector.address & 1U) != 0 || vector.request.level > 2) {
    throw Fault("Invalid DSP interrupt timing vector=" + hex(vector.address, 6));
  }
  unsigned units = 0;
  for (std::size_t index = 0; index < vector.words.size();) {
    const auto opcode = vector.words.at(index);
    const auto pc = vector.address + static_cast<unsigned>(index);
    dsp56k::Instruction first = dsp56k::Invalid;
    dsp56k::Instruction second = dsp56k::Invalid;
    if (opcodes.getInstructionTypes(opcode, first, second) == 0) {
      throw Fault("Undefined DSP interrupt timing opcode read P:" + hex(pc, 6));
    }
    const auto length = dsp56k::Opcodes::getOpcodeLength(opcode, first, second);
    const bool jsr = first == dsp56k::Jsr_ea || first == dsp56k::Jsr_xxx;
    const bool branch = (dsp56k::Opcodes::getFlags(first, second) & dsp56k::OpFlagBranch) != 0;
    if (length == 0 || length > vector.words.size() - index || (branch && !jsr) ||
        (first >= dsp56k::Do_ea && first <= dsp56k::Enddo) ||
        (first >= dsp56k::Rep_ea && first <= dsp56k::Reset) || first == dsp56k::Rti ||
        first == dsp56k::Rts || first == dsp56k::Stop || first == dsp56k::Wait ||
        first == dsp56k::Trap || first == dsp56k::Trapcc || first == dsp56k::Debug ||
        first == dsp56k::Debugcc || first == dsp56k::Illegal) {
      throw Fault("Unsupported DSP interrupt timing opcode read P:" + hex(pc, 6));
    }
    using Mask = dsp56k::RegisterMask;
    auto written = Mask::None;
    auto read = Mask::None;
    static_cast<void>(dsp56k::Opcodes::getRegisters(written, read, opcode, first, second));
    constexpr std::array restricted{Mask::LA, Mask::LC, Mask::SP, Mask::SC, Mask::SSH, Mask::SSL};
    bool restricted_write = false;
    for (const auto reg : restricted) { restricted_write |= dsp56k::any(written, reg); }
    const bool status_logic = (first == dsp56k::Andi || first == dsp56k::Ori) &&
                              (dsp56k::any(written, Mask::MR) || dsp56k::any(written, Mask::CCR));
    if (!jsr && (restricted_write || dsp56k::any(read, Mask::SSH) || status_logic)) {
      throw Fault("Restricted DSP interrupt timing register access P:" + hex(pc, 6));
    }
    units += dsp56k::calcCycles(first, second, pc, opcode, external_memory, 1);
    if (jsr) { break; }
    index += length;
  }
  return units;
}
}
