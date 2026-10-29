// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_address_stages.hpp"

#include "common/bus.hpp"
#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/dsp_address_only.hpp"
#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_memory_form.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp56kEmu/dsp.h"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>

namespace blofeld {
namespace {
bool supported_modifier(const unsigned raw) {
  // FM4.3.4/Table4-2: arbitrary raw Mn storage is valid; this checks its use
  // for arithmetic only. Upper XX is ignored. Multiple-wrap modulo requires
  // bit14=0 and a power-of-two size between2 and16384.
  constexpr unsigned low_mask = 0xffff;
  constexpr unsigned normal_maximum = 0x7fff;
  constexpr unsigned multiple_maximum = 0xbfff;
  constexpr unsigned multiple_mask = 0x3fff;
  const auto low = raw & low_mask;
  if (low <= normal_maximum || low == low_mask) { return true; }
  const auto modulus_minus_one = low & multiple_mask;
  return low <= multiple_maximum && modulus_minus_one != 0 &&
         std::has_single_bit(modulus_minus_one + 1);
}

void validate_modifier(const dsp56k::DSP &cpu,
                       const DspMemoryLane form,
                       const DspProgramWord word,
                       const unsigned omr,
                       const bool address_only = false) {
  const bool arithmetic = form.dual_encoding ? form.mode != 0 : form.mode != 4;
  if (!arithmetic) { return; }
  const auto raw = cpu.regs().m.at(form.reg).toWord();
  if (supported_modifier(raw)) { return; }
  constexpr std::array spaces{"P:", "X:", "Y:"};
  const std::string operation = form.operation == DspMemoryOperation::read ? "read " : "write ";
  const auto operand =
      address_only ? "update (R" : operation + spaces.at(static_cast<unsigned>(form.space)) + "(R";
  throw Fault("Unqualified reserved modifier AG1 " + operand + std::to_string(form.reg) +
              ") pc=" + hex(word.address, 6) + " omr=" + hex(omr, 6) +
              " opcode=" + hex(word.value, 6) + " r=" + hex(cpu.regs().r.at(form.reg).toWord(), 6) +
              " n=" + hex(cpu.regs().n.at(form.reg).toWord(), 6) + " m=" + hex(raw, 6));
}

DspAddressStageLane calculate(dsp56k::DSP &cpu,
                              const DspMemoryLane form,
                              const DspProgramWord word,
                              const unsigned omr,
                              const bool address_only = false) {
  validate_modifier(cpu, form, word, omr, address_only);
  const auto result = cpu.calculateReferenceAddress(form.mode, form.reg, form.dual_encoding);
  return {.update = {.index = form.reg,
                     .before = result.before,
                     .value = result.after,
                     .space = form.space,
                     .address = result.address,
                     .operation = form.operation,
                     .pc = word.address,
                     .omr = omr},
          .pointer_update = form.dual_encoding ? form.mode != 0 : form.mode != 4 && form.mode != 5};
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

DspAddressStageView peripheral(const DspMemoryForm &form,
                               const DspProgramWord word,
                               const std::optional<DspProgramWord> extension,
                               const unsigned omr) {
  if (!form.peripheral) { throw Fault("Native fixed-port AG1 lost its peripheral operand"); }
  const auto bit = form.peripheral_bit;
  const bool immediate = !form.peripheral_register && !bit;
  if (extension.has_value() != immediate) {
    throw Fault("Native peripheral AG1 has a mismatched F2 extension form");
  }
  const auto target = *form.peripheral;
  DspAddressStageView view{
      .word = word,
      .update = {.space = target.space,
                 .address = target.address,
                 .operation = DspMemoryOperation::write,
                 .pc = word.address,
                 .omr = omr},
      .data_accesses = {DspMemoryAccess{.target = target, .operation = DspMemoryOperation::write}},
      .data_access_count = 1,
      .kind = form.peripheral_register ? DspAddressStageKind::peripheral_register
                                       : DspAddressStageKind::peripheral_immediate,
      .extension = extension};
  if (bit) {
    const auto operation = bit->operation == DspBitOperation::test ? DspMemoryOperation::read
                                                                   : DspMemoryOperation::write;
    view.kind = DspAddressStageKind::peripheral_bit;
    view.update.operation = operation;
    view.data_accesses[0] = {
        .target = target, .operation = operation, .phase = DspMemoryPhase::peripheral_bit};
  }
  return view;
}

DspAddressStageView absolute(const DspMemoryForm &form,
                             const DspProgramWord word,
                             const std::optional<DspProgramWord> extension,
                             const unsigned omr) {
  const bool long_address = is_program_memory(form) && form.lanes[0].mode == 6;
  DspAddress target;
  if (long_address) {
    if (!extension) { throw Fault("Native absolute-long AG1 lost its actual F2 extension"); }
    target = {.space = DspSpace::p, .address = extension->value};
  } else {
    if (extension || !form.absolute_short) {
      throw Fault("Native absolute-short AG1 has a mismatched literal/F2 form");
    }
    target = *form.absolute_short;
  }
  const auto operation = form.lanes[0].operation;
  return {.word = word,
          .update = {.space = target.space,
                     .address = target.address,
                     .operation = operation,
                     .pc = word.address,
                     .omr = omr},
          .data_accesses = {DspMemoryAccess{.target = target, .operation = operation}},
          .data_access_count = 1,
          .kind = long_address ? DspAddressStageKind::absolute_long
                               : DspAddressStageKind::absolute_short,
          .extension = extension};
}
}

std::size_t DspAddressStages::capture(dsp56k::DSP &cpu, const DspFetchedInstruction &fetched) {
  const auto word = fetched.opcode.word;
  const auto extension = fetched.extension ? std::optional(fetched.extension->word) : std::nullopt;
  const auto form = decode_memory_stage(word.value);
  const auto update = decode_address_only(word.value);
  if ((!form && !update) || serial == std::numeric_limits<std::uint64_t>::max()) {
    throw Fault("Unqualified/exhausted native AG1 at P:" + hex(word.address, 6));
  }
  for (std::size_t slot = 0; slot < records.size(); ++slot) {
    if (records.at(slot)) { continue; }
    const auto source = cpu.referenceAttemptState();
    const auto omr = static_cast<std::uint32_t>(source.registers.omr);
    DspAddressStageView view;
    const bool absolute_long = form && is_program_memory(*form) && form->lanes[0].mode == 6;
    if (update) {
      if (extension) { throw Fault("Address-only AG1 cannot capture an extension"); }
      const auto lane = calculate(cpu, {.mode = update->mode, .reg = update->reg}, word, omr, true);
      view = {.word = word,
              .update = lane.update,
              .pointer_update = true,
              .kind = DspAddressStageKind::address_only};
    } else if (form->absolute_short || absolute_long) {
      view = absolute(*form, word, extension, omr);
    } else if (form->peripheral && !is_program_transfer(*form)) {
      view = peripheral(*form, word, extension, omr);
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
