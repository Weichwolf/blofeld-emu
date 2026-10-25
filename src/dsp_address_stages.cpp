// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_address_stages.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_program_fetch.hpp"
#include "dsp56kEmu/dsp.h"
#include "dsp56kEmu/opcodeanalysis.h"
#include "dsp56kEmu/opcodes.h"
#include "dsp56kEmu/opcodetypes.h"
#include "dsp_memory_form.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>

namespace blofeld {
std::optional<DspMemoryForm> decode_memory_stage(const std::uint32_t opcode) {
  static const dsp56k::Opcodes opcodes;
  auto first = dsp56k::Invalid;
  auto second = dsp56k::Invalid;
  if (opcodes.getInstructionTypes(opcode, first, second) == 0) { return std::nullopt; }
  const auto memory = [](const auto type) {
    return type == dsp56k::Movex_ea || type == dsp56k::Movey_ea;
  };
  const auto type = memory(first) ? first : second;
  if (!memory(type) || (memory(first) ? second != dsp56k::Invalid : first != dsp56k::Nop) ||
      dsp56k::Opcodes::getOpcodeLength(opcode, first, second) != 1) {
    return std::nullopt;
  }
  const auto reg = dsp56k::getRegister(type, dsp56k::Field_ddd, opcode);
  using Reg = dsp56k::Register;
  if (reg != Reg::X0 && reg != Reg::X1 && reg != Reg::Y0 && reg != Reg::Y1) { return std::nullopt; }
  const auto mode = dsp56k::getFieldValue(type, dsp56k::Field_MMM, opcode);
  if (mode > 4) { return std::nullopt; }
  const bool load = dsp56k::getFieldValue(type, dsp56k::Field_W, opcode) != 0;
  return DspMemoryForm{.space = type == dsp56k::Movex_ea ? DspSpace::x : DspSpace::y,
                       .operation = load ? DspMemoryOperation::read : DspMemoryOperation::write,
                       .mode = mode,
                       .reg = dsp56k::getFieldValue(type, dsp56k::Field_RRR, opcode)};
}

std::size_t DspAddressStages::capture(dsp56k::DSP &cpu, const DspProgramWord word) {
  const auto form = decode_memory_stage(word.value);
  if (!form || serial == std::numeric_limits<std::uint64_t>::max()) {
    throw Fault("Unqualified/exhausted native AG1 at P:" + hex(word.address, 6));
  }
  for (std::size_t slot = 0; slot < records.size(); ++slot) {
    if (records.at(slot)) { continue; }
    const auto source = cpu.referenceAttemptState();
    const auto result = cpu.calculateReferenceAddress(form->mode, form->reg);
    const auto omr = static_cast<std::uint32_t>(source.registers.omr);
    records.at(slot) = DspAddressRecord{.view = {.word = word,
                                                 .update = {.index = form->reg,
                                                            .before = result.before,
                                                            .value = result.after,
                                                            .space = form->space,
                                                            .address = result.address,
                                                            .operation = form->operation,
                                                            .pc = word.address,
                                                            .omr = omr},
                                                 .pointer_update = form->mode != 4},
                                        .source = source,
                                        .serial = ++serial};
    return slot;
  }
  throw Fault("Native AG1 continuation capacity exhausted pc=" + hex(word.address, 6));
}

namespace {
template <typename Owner> auto &record(Owner &owner, const DspAddressPosition position) {
  if (position.slot >= owner.records.size()) {
    throw Fault("Unavailable native AG1/AG2/E1 record slot");
  }
  auto &value = owner.records.at(position.slot);
  if (!value || value->serial != position.serial) {
    throw Fault("Stale or unavailable native AG1/AG2/E1 record");
  }
  return *value;
}
}

const DspAddressRecord &DspAddressStages::get(const DspAddressPosition position) const {
  return record(*this, position);
}

DspAddressRecord &DspAddressStages::get(const DspAddressPosition position) {
  return record(*this, position);
}
}
