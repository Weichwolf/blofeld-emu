// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_address_stages.hpp"

#include "common/bus.hpp"
#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_memory_form.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp56kEmu/dsp.h"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>

namespace blofeld {
namespace {
DspAddressStageLane calculate(dsp56k::DSP &cpu,
                              const DspMemoryLane form,
                              const DspProgramWord word,
                              const unsigned omr) {
  const auto result = cpu.calculateReferenceAddress(form.mode, form.reg, form.dual_encoding);
  return {.update = {.index = form.reg,
                     .before = result.before,
                     .value = result.after,
                     .space = form.space,
                     .address = result.address,
                     .operation = form.operation,
                     .pc = word.address,
                     .omr = omr},
          .pointer_update = form.dual_encoding ? form.mode != 0 : form.mode != 4};
}

DspMemoryAccess access(const DspAddressUpdate update) {
  return {.target = {.space = update.space, .address = update.address},
          .operation = update.operation};
}

DspAddressStageView prepare(dsp56k::DSP &cpu,
                            const DspMemoryForm &form,
                            const DspProgramWord word,
                            const unsigned omr) {
  const auto first = calculate(cpu, form.lanes[0], word, omr);
  DspAddressStageView result{
      .word = word, .update = first.update, .pointer_update = first.pointer_update};
  result.data_accesses[0] = access(first.update);
  result.data_access_count = 1;
  if (form.lane_count == 2) {
    result.secondary = calculate(cpu, form.lanes[1], word, omr);
    result.data_accesses[1] = access(result.secondary->update);
    result.data_access_count = 2;
    // The native dual move performs stores before loads, X before Y within each kind.
    if (result.data_accesses[0].operation == DspMemoryOperation::read &&
        result.data_accesses[1].operation == DspMemoryOperation::write) {
      std::swap(result.data_accesses[0], result.data_accesses[1]);
    }
  } else if (form.long_pair) {
    result.data_accesses[1] = result.data_accesses[0];
    result.data_accesses[1].target.space = DspSpace::y;
    result.data_access_count = 2;
  }
  if (is_program_transfer(form)) {
    const auto peripheral = form.peripheral;
    if (!peripheral) { throw Fault("Native P-transfer lost its peripheral operand"); }
    result.data_accesses[1] = result.data_accesses[0];
    result.data_accesses[0] = {.target = *peripheral};
    result.data_access_count = 2;
  }
  return result;
}
}

std::size_t DspAddressStages::capture(dsp56k::DSP &cpu, const DspFetchedInstruction &fetched) {
  const auto word = fetched.opcode.word;
  const auto extension = fetched.extension ? std::optional(fetched.extension->word) : std::nullopt;
  const auto form = decode_memory_stage(word.value);
  if (!form || serial == std::numeric_limits<std::uint64_t>::max()) {
    throw Fault("Unqualified/exhausted native AG1 at P:" + hex(word.address, 6));
  }
  for (std::size_t slot = 0; slot < records.size(); ++slot) {
    if (records.at(slot)) { continue; }
    const auto source = cpu.referenceAttemptState();
    const auto omr = static_cast<std::uint32_t>(source.registers.omr);
    DspAddressStageView view;
    if (form->peripheral && !is_program_transfer(*form)) {
      if (extension.has_value() == form->peripheral_register) {
        throw Fault("Native peripheral AG1 has a mismatched F2 extension form");
      }
      const auto target = *form->peripheral;
      view = {.word = word,
              .update = {.space = target.space,
                         .address = target.address,
                         .operation = DspMemoryOperation::write,
                         .pc = word.address,
                         .omr = omr},
              .data_accesses = {DspMemoryAccess{.target = target,
                                                .operation = DspMemoryOperation::write}},
              .data_access_count = 1,
              .kind = form->peripheral_register ? DspAddressStageKind::peripheral_register
                                                : DspAddressStageKind::peripheral_immediate,
              .extension = extension};
    } else {
      if (extension) { throw Fault("Unexpected extension for native register AG1"); }
      view = prepare(cpu, *form, word, omr);
    }
    records.at(slot) =
        DspAddressRecord{.view = view, .fetched = fetched, .source = source, .serial = ++serial};
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
