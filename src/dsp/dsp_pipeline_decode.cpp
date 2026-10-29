// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_pipeline.hpp"
#include "dsp/dsp_accumulator_transfer.hpp"
#include "dsp/dsp_address_only.hpp"
#include "dsp/dsp_branch_form.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_loop_source.hpp"
#include "dsp/dsp_memory_form.hpp"
#include "dsp/dsp_move_writeback.hpp"
#include "dsp/dsp_omr_move.hpp"
#include "dsp/dsp_peripheral_read_form.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_repeat_fetch.hpp"
#include "dsp56kEmu/opcodeanalysis.h"
#include "dsp56kEmu/opcodes.h"
#include "dsp56kEmu/opcodetypes.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <optional>

namespace blofeld {
namespace {
using Type = dsp56k::Instruction;
// FM5.3/Table5-1 and A-1: register ALU operations and immediate CMP forms.
constexpr std::array allowed{
    dsp56k::Nop,       dsp56k::Abs,        dsp56k::Add_SD,    dsp56k::Add_xx,     dsp56k::Addl,
    dsp56k::Addr,      dsp56k::And_SD,     dsp56k::And_xx,    dsp56k::Asl_D,      dsp56k::Asl_ii,
    dsp56k::Asl_S1S2D, dsp56k::Asr_D,      dsp56k::Asr_ii,    dsp56k::Asr_S1S2D,  dsp56k::Clr,
    dsp56k::Cmp_S1S2,  dsp56k::Cmp_xxxxS2, dsp56k::Cmp_xxS2,  dsp56k::Cmpm_S1S2,  dsp56k::Cmpu_S1S2,
    dsp56k::Eor_SD,    dsp56k::Eor_xx,     dsp56k::Lsl_D,     dsp56k::Lsl_ii,     dsp56k::Lsl_SD,
    dsp56k::Lsr_D,     dsp56k::Lsr_ii,     dsp56k::Lsr_SD,    dsp56k::Mac_S1S2,   dsp56k::Macr_S1S2,
    dsp56k::Max,       dsp56k::Maxm,       dsp56k::Mpy_S1S2D, dsp56k::Mpyr_S1S2D, dsp56k::Neg,
    dsp56k::Not,       dsp56k::Or_SD,      dsp56k::Or_xx,     dsp56k::Rnd,        dsp56k::Sub_SD,
    dsp56k::Sub_xx,    dsp56k::Subl,       dsp56k::Subr,      dsp56k::Tfr,        dsp56k::Tst,
    dsp56k::Inc,       dsp56k::Dec};
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

std::uint8_t accumulator_transfer_dependency(const DspInstructionDependency instructions) {
  const auto consumer = instructions.consumer.value;
  if (!accumulator_transfer_source(consumer) && !middle_accumulator_transfer_source(consumer)) {
    return 0;
  }
  static const dsp56k::Opcodes opcodes;
  auto first = dsp56k::Invalid;
  auto second = dsp56k::Invalid;
  static_cast<void>(opcodes.getInstructionTypes(consumer, first, second));
  const auto move = first == dsp56k::Mover ? first : second;
  auto written = Mask::None;
  auto read = Mask::None;
  dsp56k::getRegisters(written, read, move, consumer);
  // FM3.5.1/3.5.2.1: only the bus source interlocks; ordinary ALU operands
  // retain their existing forwarding,including the other accumulator.
  return dependency(operands(instructions.producer, false).written, read);
}

std::uint8_t memory_source_dependency(const DspInstructionDependency instructions) {
  const auto producer = instructions.producer;
  const auto consumer = instructions.consumer;
  const auto memory = decode_memory_stage(consumer.value);
  const bool program_store = memory && is_program_memory(*memory) &&
                             memory->lanes[0].operation == DspMemoryOperation::write;
  if (program_store || full_accumulator_scalar_store(consumer.value) ||
      partial_accumulator_scalar_store(consumer.value) ||
      middle_accumulator_peripheral_store(consumer.value)) {
    // FM3.5.1: the native E1 memory source must observe the preceding E2 writer.
    return dependency(operands(producer, false).written, operands(consumer, false).read);
  }
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

bool pipeline_counted_loop(const DspProgramWord word) {
  return automatic_loop_source(word.value).has_value();
}

bool pipeline_loop_source_dependency(const DspInstructionDependency instructions) {
  const auto source = automatic_loop_source(instructions.consumer.value);
  const bool repeat = instructions.consumer.value == dsp_repeat_from_a1;
  if (!repeat && source != DspLoopCountSource::a && source != DspLoopCountSource::a0) {
    return false;
  }
  // Retained NXP windows: DO A/A0 and REP A1 interlock on any preceding A part.
  // Two-word writers already separate Decode and are no longer E2 producers.
  return dsp56k::any(operands(instructions.producer, false).written, Mask::A);
}

DspPipelineFormat decode_pipeline_word(const DspProgramWord word,
                                       const std::uint32_t omr,
                                       const DspFetchPipeline profile) {
  static const dsp56k::Opcodes opcodes;
  Type first = dsp56k::Invalid;
  Type second = dsp56k::Invalid;
  constexpr unsigned width = 6;
  const bool decoded = opcodes.getInstructionTypes(word.value, first, second) != 0;
  const bool arithmetic =
      std::ranges::contains(allowed, first) || pipeline_ccr_immediate(word.value);
  try {
    if (!decoded) { throw Fault("Undefined native instruction"); }
    const auto port_read = profile == DspFetchPipeline::automatic_memory
                               ? decode_peripheral_read(word.value)
                               : std::nullopt;
    const bool branch =
        profile == DspFetchPipeline::automatic_memory &&
        (decode_branch_form(word.value).has_value() || (port_read && port_read->bit));
    const auto form = decode_memory_stage(word.value);
    const bool address_only = profile == DspFetchPipeline::automatic_memory &&
                              decode_address_only(word.value).has_value();
    const bool memory = profile == DspFetchPipeline::automatic_memory && form.has_value();
    const bool transfer = profile == DspFetchPipeline::automatic_memory && !memory &&
                          pipeline_transfer_fields(word.value) != DspMoveFields{};
    const bool loop = profile == DspFetchPipeline::automatic_memory && pipeline_counted_loop(word);
    const bool repeat =
        profile == DspFetchPipeline::automatic_memory && fetch_repeat_prefix(word.value);
    const bool control =
        profile == DspFetchPipeline::automatic_memory && decode_omr_move(word.value).has_value();
    const bool standalone_move =
        (memory || transfer || port_read || control || address_only) && second == dsp56k::Invalid;
    const auto words = pipeline_word_count(word);
    if ((!arithmetic && !standalone_move && !branch && !loop && !repeat) ||
        (!memory && !transfer && !address_only && second != dsp56k::Invalid &&
         second != dsp56k::Move_Nop) ||
        words > ((transfer || branch || loop || control || first == dsp56k::Cmp_xxxxS2 ||
                  (memory && (form->peripheral || is_program_memory(*form))))
                     ? 2U
                     : 1U)) {
      throw Fault("Unqualified native instruction form");
    }
    if (memory || transfer || port_read) { static_cast<void>(decode_move_writeback(word.value)); }
    return {.words = words, .memory = memory, .transfer = transfer, .branch = branch};
  } catch (const std::exception &error) {
    throw Fault("Unqualified automatic DSP decode P:" + hex(word.address, width) +
                " opcode=" + hex(word.value, width) + " pc=" + hex(word.address, width) +
                " omr=" + hex(omr, width) + ": " + error.what());
  }
}
}
