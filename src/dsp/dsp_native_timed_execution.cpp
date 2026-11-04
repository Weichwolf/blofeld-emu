// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_native_timed_execution.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_interlocks.hpp"
#include "dsp/dsp_native_dependencies.hpp"
#include "dsp56300/core.hpp"

#include <algorithm>
#include <optional>

namespace blofeld {

DspNativeTimedExecution::DspNativeTimedExecution(const dsp56300::Progress initial)
    : expected_(initial) {
  prefix_.reserve(maximum_prefix);
}

DspNativeTimedExecution::DspNativeTimedExecution(const DspNativeTimedExecution &other)
    : expected_(other.expected_),
      control_delay_(other.control_delay_),
      history_(other.history_),
      offered_(other.offered_),
      costs_(other.costs_) {
  prefix_.reserve(maximum_prefix);
}

DspNativeTimedExecution &DspNativeTimedExecution::operator=(const DspNativeTimedExecution &other) {
  if (this != &other) {
    expected_ = other.expected_;
    control_delay_ = other.control_delay_;
    history_ = other.history_;
    offered_ = other.offered_;
    costs_ = other.costs_;
    prefix_.clear();
  }
  return *this;
}

unsigned DspNativeTimedExecution::slot(const dsp56300::Word opcode) {
  constexpr unsigned byte_bits = 8;
  return (opcode ^ (opcode >> byte_bits) ^ (opcode >> (2 * byte_bits))) % cost_slots;
}

void DspNativeTimedExecution::check_progress(const dsp56300::Core &cpu) const {
  if (cpu.progress() != expected_) {
    throw Fault("DSP dependency history lost actual CPU progress pc=" + hex(cpu.registers().pc, 6));
  }
}

std::optional<unsigned> DspNativeTimedExecution::cost(const dsp56300::Word opcode) const {
  const auto &entry = costs_.at(slot(opcode));
  return entry.opcode == opcode ? std::optional{entry.units} : std::nullopt;
}

void DspNativeTimedExecution::learn(const dsp56300::BoundedRunResult &preview) {
  offered_.reset();
  if (!preview.next || !preview.next_cycles) { return; }
  if (*preview.next_cycles == 0) { throw Fault("DSP Core supplied an empty base cost"); }
  const auto opcode = preview.next->opcode;
  auto &entry = costs_.at(slot(opcode));
  if (entry.opcode == opcode && entry.units != *preview.next_cycles) {
    throw Fault("DSP Core changed an observed opcode base cost pc=" +
                hex(preview.next->address, 6));
  }
  entry = {.opcode = opcode, .units = *preview.next_cycles};
  offered_ = Offered{.words = *preview.next, .base_units = *preview.next_cycles};
}

void DspNativeTimedExecution::check_offer(const dsp56300::InstructionWords words,
                                          const unsigned units) const {
  if (!offered_ || offered_->base_units != units || offered_->words.address != words.address ||
      offered_->words.opcode != words.opcode || offered_->words.extension != words.extension) {
    throw Fault("DSP owner instruction differs from the actual Core offer pc=" +
                hex(words.address, 6));
  }
}

std::optional<DspInterlockInstruction>
DspNativeTimedExecution::describe(const dsp56300::Word opcode, const unsigned units) {
  if (const auto result = control(opcode, units)) { return result; }
  return inspect_native_dependencies(opcode, units);
}

DspInterlockInstruction DspNativeTimedExecution::required(const dsp56300::InstructionWords words,
                                                          const unsigned units) {
  const auto decoded = describe(words.opcode, units);
  if (!decoded) {
    throw Fault("Unqualified DSP dependency instruction pc=" + hex(words.address, 6) +
                " opcode=" + hex(words.opcode, 6));
  }
  return *decoded;
}

DspTimedInstruction DspNativeTimedExecution::prepare_owner(const dsp56300::Core &cpu,
                                                           const dsp56300::InstructionWords words,
                                                           const unsigned base_units) const {
  check_progress(cpu);
  check_offer(words, base_units);
  return {.words = words,
          .base_units = base_units,
          .dependency_units =
              std::max(history_.preview(required(words, base_units)), control_delay_)};
}

void DspNativeTimedExecution::complete_owner(const dsp56300::Core &cpu,
                                             const DspTimedInstruction accounted) {
  const auto words = accounted.words;
  const auto base_units = accounted.base_units;
  check_offer(words, base_units);
  const auto after = cpu.progress();
  const auto outcome = cpu.instruction_outcome();
  if (cpu.fault().kind != dsp56300::FaultKind::none ||
      after.instructions < expected_.instructions ||
      after.estimated_cycles < expected_.estimated_cycles ||
      after.instructions - expected_.instructions != 1 ||
      after.estimated_cycles - expected_.estimated_cycles != base_units || !outcome ||
      outcome->instructions != after.instructions || outcome->address != words.address ||
      outcome->opcode != words.opcode) {
    throw Fault("DSP dependency completion lacks its actual staged retirement pc=" +
                hex(words.address, 6));
  }
  const auto decoded = required(words, base_units);
  if (std::max(history_.preview(decoded), control_delay_) != accounted.dependency_units) {
    throw Fault("DSP owner clocked a different dependency delay pc=" + hex(words.address, 6));
  }
  if (control_delay_ != 0) { history_.elapse_idle(control_delay_); }
  history_.prepare(decoded);
  history_.observe_opcode(words.opcode);
  static_cast<void>(history_.complete(base_units));
  expected_ = after;
  offered_.reset();
  control_delay_ = conditional(words.opcode) && !outcome->control_transfer ? 1U : 0U;
}

void DspNativeTimedExecution::reset(const dsp56300::Progress actual) {
  history_.reset();
  expected_ = actual;
  offered_.reset();
  control_delay_ = 0;
  prefix_.clear();
}

void DspNativeTimedExecution::idle(const dsp56300::Core &cpu, const unsigned units) {
  check_progress(cpu);
  if (cpu.processing_state() != dsp56300::ProcessingState::waiting || control_delay_ != 0) {
    throw Fault("DSP dependency idle clocks require the real waiting CPU");
  }
  history_.elapse_idle(units);
}
}
