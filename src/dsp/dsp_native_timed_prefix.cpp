// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_interlocks.hpp"
#include "dsp/dsp_native_timed_execution.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/instruction.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>

namespace blofeld {
namespace {
bool sequential(const dsp56300::InstructionKind kind) {
  using Kind = dsp56300::InstructionKind;
  // Unlisted families end the plan, even when their dependency metadata becomes
  // available later. Native Core may still execute that final control opcode.
  constexpr std::array allowed{Kind::parallel,
                               Kind::nop,
                               Kind::arithmetic_shift_immediate,
                               Kind::arithmetic_shift_register,
                               Kind::logical_shift_immediate,
                               Kind::logical_shift_register,
                               Kind::immediate_alu,
                               Kind::load_relative_address,
                               Kind::load_updated_address,
                               Kind::divide_iteration,
                               Kind::normalize_iteration,
                               Kind::normalize_fast,
                               Kind::viterbi_shift,
                               Kind::product_immediate,
                               Kind::product_power,
                               Kind::product_mixed,
                               Kind::product_double,
                               Kind::increment,
                               Kind::decrement,
                               Kind::compare_unsigned,
                               Kind::bit_field_extract,
                               Kind::bit_field_insert,
                               Kind::bit_field_merge,
                               Kind::count_leading_bits,
                               Kind::conditional_transfer,
                               Kind::move_displacement,
                               Kind::move_program_effective,
                               Kind::move_program_short,
                               Kind::move_peripheral_register,
                               Kind::move_peripheral_memory,
                               Kind::move_peripheral_program,
                               Kind::bit_memory};
  return std::ranges::find(allowed, kind) != allowed.end();
}
}

std::optional<DspNativeTimedExecution::Entry> DspNativeTimedExecution::inspect(
    const Cursor &cursor, const DspInterlocks &history, const std::uint64_t available) const {
  std::optional<dsp56300::Word> opcode;
  if (cursor.first) {
    opcode = cursor.initial.opcode;
  } else if (cursor.repeated) {
    opcode = cursor.repeated;
  } else {
    opcode = cursor.code.inspect_program(cursor.address);
  }
  if (!opcode) { return std::nullopt; }
  const auto units = cost(*opcode);
  const auto shape = dsp56300::inspect_instruction(*opcode);
  if (!units || !shape || *units > available) { return std::nullopt; }
  const auto decoded = describe(*opcode, *units);
  if (!decoded || history.preview(*decoded) != 0) { return std::nullopt; }
  dsp56300::InstructionWords words{.address = cursor.address, .opcode = *opcode};
  if (shape->words == 2) {
    words.extension = cursor.first
                          ? cursor.initial.extension
                          : cursor.code.inspect_program((cursor.address + 1) & dsp56300::word_mask);
    if (!words.extension) { return std::nullopt; }
  }
  return Entry{.words = words, .dependencies = *decoded, .sequential = sequential(shape->kind)};
}

unsigned DspNativeTimedExecution::plan(const dsp56300::Core &cpu,
                                       const dsp56300::Io &code,
                                       const dsp56300::BoundedRunResult &preview,
                                       const dsp56300::RunBudget budget) {
  prefix_.clear();
  if (control_delay_ != 0) { return 0; }
  const auto fetch = cpu.instruction_fetch();
  if (!preview.next || fetch.exception_vector) { return 0; }
  const auto registers = cpu.registers();
  const bool repeated = fetch.latched_opcode.has_value();
  auto count = std::min(budget.instructions, maximum_prefix);
  if (repeated) {
    count = std::min(count, registers.lc != 0 ? registers.lc : 1U);
  } else if (!cpu.interruptible()) {
    count = std::min(count, 1U);
  }
  auto history = history_;
  auto address = preview.next->address;
  unsigned cycles = 0;
  for (unsigned index = 0; index < count; ++index) {
    const auto entry = inspect({.code = code,
                                .initial = *preview.next,
                                .repeated = fetch.latched_opcode,
                                .address = address,
                                .first = index == 0},
                               history,
                               budget.cycles - cycles);
    if (!entry) { break; }
    prefix_.push_back(*entry);
    history.prepare(entry->dependencies);
    history.observe_opcode(entry->words.opcode);
    static_cast<void>(history.complete(entry->dependencies.native_units));
    cycles += entry->dependencies.native_units;
    if (!entry->sequential) { break; }
    constexpr unsigned loop_flag = 1U << 15;
    const auto last = (address + (entry->words.extension ? 1U : 0U)) & dsp56300::word_mask;
    if ((registers.sr & loop_flag) != 0 && last == registers.la) { break; }
    if (!repeated) { address = (last + 1) & dsp56300::word_mask; }
  }
  return cycles;
}

void DspNativeTimedExecution::accept(const dsp56300::Core &cpu,
                                     const dsp56300::BoundedRunResult &actual) {
  const auto retired = actual.execution.dispatches;
  if (retired > prefix_.size() || actual.execution.progress != cpu.progress()) {
    throw Fault("DSP native prefix lost its actual retirement receipt");
  }
  auto history = history_;
  std::uint64_t cycles = 0;
  for (unsigned index = 0; index < retired; ++index) {
    const auto &entry = prefix_.at(index);
    if (history.preview(entry.dependencies) != 0) {
      throw Fault("DSP native prefix crossed an unclocked dependency");
    }
    history.prepare(entry.dependencies);
    history.observe_opcode(entry.words.opcode);
    static_cast<void>(history.complete(entry.dependencies.native_units));
    cycles += entry.dependencies.native_units;
  }
  const auto after = cpu.progress();
  if (actual.cycles != cycles || after.instructions < expected_.instructions ||
      after.estimated_cycles < expected_.estimated_cycles ||
      after.instructions - expected_.instructions != retired ||
      after.estimated_cycles - expected_.estimated_cycles != cycles) {
    throw Fault("DSP native prefix changed its qualified code/base costs");
  }
  history_ = history;
  expected_ = after;
  if (retired != 0) {
    const auto &last = prefix_.at(retired - 1).words;
    control_delay_ = 0;
    if (conditional(last.opcode)) {
      const auto outcome = cpu.instruction_outcome();
      if (!outcome || outcome->instructions != after.instructions ||
          outcome->address != last.address || outcome->opcode != last.opcode) {
        throw Fault("DSP conditional prefix lacks its actual control-flow receipt");
      }
      control_delay_ = outcome->control_transfer ? 0U : 1U;
    }
  }
}

dsp56300::BoundedRunResult DspNativeTimedExecution::run(dsp56300::Core &cpu,
                                                        const dsp56300::Io &code,
                                                        const dsp56300::RunBudget budget) {
  check_progress(cpu);
  auto preview = cpu.run(dsp56300::RunBudget{});
  learn(preview);
  const auto revision = code.program_revision();
  const auto cycles = plan(cpu, code, preview, budget);
  if (prefix_.empty()) { return preview; }
  const auto actual = cpu.run(
      dsp56300::RunBudget{.instructions = static_cast<unsigned>(prefix_.size()), .cycles = cycles});
  if (code.program_revision() != revision) {
    throw Fault("DSP native prefix code/mapping changed while executing");
  }
  accept(cpu, actual);
  learn(actual);
  return actual;
}
}
