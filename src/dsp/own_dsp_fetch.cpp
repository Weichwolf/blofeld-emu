// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/own_dsp_fetch.hpp"

#include "common/bus.hpp"
#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/instruction.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <utility>

namespace blofeld {
namespace {
constexpr unsigned cache_enable = 1U << 19U;

class CycleGuard {
public:
  explicit CycleGuard(bool &active) : active_(active) { active_ = true; }

  ~CycleGuard() { active_ = false; }

  CycleGuard(const CycleGuard &) = delete;
  CycleGuard &operator=(const CycleGuard &) = delete;

private:
  bool &active_;
};
}

OwnDspFetch::OwnDspFetch(OwnDspMachine &machine)
    : machine_(machine), identity_(std::make_shared<const bool>(false)) {
  machine_.require_running();
  if (machine_.advancing_effect_ || !machine_.captured_fetch_owner_.expired() ||
      machine_.interrupts_.pending()) {
    throw Fault("Own DSP fetch cannot acquire CPU pc=" + hex(machine_.cpu_.registers().pc, 6));
  }
  const auto context = machine_.cpu_.instruction_fetch();
  native_latch_ = context.latched_opcode.has_value();
  fetch_.start(context.address, machine_.cpu_.registers().omr);
  machine_.captured_fetch_owner_ = identity_;
}

void OwnDspFetch::check_owner() const {
  if (machine_.captured_fetch_owner_.lock() != identity_) {
    throw Fault("Stale own DSP captured fetch pc=" + hex(machine_.cpu_.registers().pc, 6));
  }
  machine_.require_running();
  if (machine_.advancing_effect_) { throw Fault("Own DSP F2 requires an inactive effect"); }
}

void OwnDspFetch::check_sample_context() const {
  const auto regs = machine_.cpu_.registers();
  const auto state = fetch_.view();
  if (in_flight_ && !machine_.cpu_.pending_instruction()) {
    throw Fault("Own DSP F2 requires retirement synchronization read P:" +
                hex(state.pending_address.value_or(state.next_address), 6) +
                " pc=" + hex(regs.pc, 6) + " omr=" + hex(regs.omr, 6));
  }
  if ((regs.sr & cache_enable) != 0 || native_fetch_ || native_latch_ || state.omr != regs.omr) {
    throw Fault("Unqualified own DSP F2 memory context read P:" +
                hex(state.pending_address.value_or(state.next_address), 6) +
                " pc=" + hex(regs.pc, 6) + " omr=" + hex(regs.omr, 6));
  }
  if (repeated_) {
    throw Fault("Own DSP REP F2 prefetch is unqualified read P:" +
                hex(state.pending_address.value_or(state.next_address), 6) +
                " pc=" + hex(regs.pc, 6) + " omr=" + hex(regs.omr, 6));
  }
  if (interrupt_vector_) {
    throw Fault("Own DSP vector owns program fetch read P:" +
                hex(state.pending_address.value_or(state.next_address), 6) +
                " pc=" + hex(regs.pc, 6) + " omr=" + hex(regs.omr, 6));
  }
}

DspProgramFetchView OwnDspFetch::view() const {
  check_owner();
  return fetch_.view();
}

void OwnDspFetch::issue() {
  check_owner();
  check_sample_context();
  fetch_.issue();
}

void OwnDspFetch::capture() {
  check_owner();
  check_sample_context();
  fetch_.capture(machine_.bus_);
}

OwnDspFetchCycleResult OwnDspFetch::advance_dma_capture(const DspDmaInputs inputs) {
  check_owner();
  check_sample_context();
  const auto state = fetch_.view();
  const auto regs = machine_.cpu_.registers();
  const auto context = " pc=" + hex(regs.pc, 6) + " omr=" + hex(regs.omr, 6);
  if (machine_.clock_inputs_.switch_timing != DspClockSwitch::reference_cycle_boundary ||
      machine_.cpu_.fault().kind != dsp56300::FaultKind::none ||
      machine_.cpu_.processing_state() != dsp56300::ProcessingState::running || !state.active ||
      !state.pending_address || state.used == state.words.size()) {
    throw Fault("Own DSP DMA/F2 cycle requires a real pending F2 read" + context);
  }
  const std::array accesses{DspMemoryAccess{.target = {.address = *state.pending_address}}};
  const CycleGuard guard(machine_.advancing_effect_);
  static_cast<void>(machine_.clock_.next_step_ticks(machine_.bus_, 1));
  const bool switch_was_pending = machine_.bus_.switch_delay_ != 0;
  machine_.clock_.begin_step(machine_.bus_, {.pc = regs.pc, .omr = regs.omr});
  try {
    OwnDspFetchCycleResult result;
    result.dma =
        machine_.bus_.advance_dma(inputs,
                                  {.accesses = std::span(accesses),
                                   .knowledge = DspCoreBusKnowledge::reference_cycle_occupancy});
    if (!result.dma.core_stall) {
      fetch_.capture(machine_.bus_);
      result.captured = true;
    }
    machine_.clock_.finish_dma_cycle(machine_.bus_, inputs.cycle, switch_was_pending);
    return result;
  } catch (const std::exception &error) {
    machine_.clock_.fail();
    throw Fault("Own DSP DMA/F2 cycle read P:" + hex(*state.pending_address, 6) + context + ": " +
                error.what());
  }
}

bool OwnDspFetch::admit_interrupt() {
  check_owner();
  if (in_flight_ || interrupt_vector_ || repeated_ || native_latch_ ||
      (machine_.cpu_.registers().sr & cache_enable) != 0) {
    throw Fault("Own DSP interrupt admission requires a foreground boundary pc=" +
                hex(machine_.cpu_.registers().pc, 6));
  }
  const auto context = machine_.cpu_.instruction_fetch();
  const auto state = fetch_.view();
  if (context.latched_opcode || state.consume_address != context.address ||
      state.omr != machine_.cpu_.registers().omr) {
    throw Fault("Own DSP interrupt foreground differs from F2 read P:" + hex(context.address, 6) +
                " pc=" + hex(context.address, 6) + " omr=" + hex(state.omr, 6));
  }
  const auto opcode = fetch_.select(context.address);
  const auto shape = dsp56300::inspect_instruction(opcode.word.value);
  if (!shape) {
    throw Fault("Unsupported own DSP interrupt foreground opcode " + hex(opcode.word.value, 6) +
                " read P:" + hex(context.address, 6) + " pc=" + hex(context.address, 6) +
                " omr=" + hex(state.omr, 6));
  }
  const auto captured = fetch_.instruction(opcode, shape->words);
  captured.check_front(fetch_);
  const dsp56300::InstructionWords foreground{
      .address = captured.opcode.word.address,
      .opcode = captured.opcode.word.value,
      .extension =
          captured.extension ? std::optional(captured.extension->word.value) : std::nullopt};
  if (!machine_.interrupts_.admit(foreground)) { return false; }
  const auto vector = machine_.cpu_.instruction_fetch();
  if (!vector.exception_vector) {
    throw Fault("Own DSP admitted interrupt without native vector ownership pc=" +
                hex(machine_.cpu_.registers().pc, 6));
  }
  interrupt_vector_ = vector.address;
  return true;
}

dsp56300::InstructionProgress OwnDspFetch::begin() {
  check_owner();
  if (in_flight_) { throw Fault("Own DSP F2 instruction awaits retirement"); }
  if (machine_.cpu_.progress().instructions == std::numeric_limits<std::uint64_t>::max()) {
    throw Fault("Own DSP F2 instruction counter exhausted pc=" +
                hex(machine_.cpu_.registers().pc, 6));
  }
  if (interrupt_vector_) { return begin_native(); }
  if ((machine_.cpu_.registers().sr & cache_enable) != 0 || native_latch_) {
    return begin_native();
  }
  const auto context = machine_.cpu_.instruction_fetch();
  const auto state = fetch_.view();
  if (state.omr != machine_.cpu_.registers().omr) {
    throw Fault("Own DSP F2 context differs from CPU read P:" + hex(context.address, 6) +
                " pc=" + hex(context.address, 6) + " omr=" + hex(state.omr, 6));
  }
  if (context.latched_opcode) {
    if (!repeated_ || repeated_->extension || repeated_->opcode.word.address != context.address ||
        repeated_->opcode.word.value != *context.latched_opcode) {
      throw Fault("Own DSP REP latch lost its actual F2 sample read P:" + hex(context.address, 6) +
                  " pc=" + hex(context.address, 6) + " omr=" + hex(state.omr, 6));
    }
    return admit(*repeated_);
  }
  if (repeated_ || state.consume_address != context.address) {
    throw Fault("Own DSP F2 context differs from CPU read P:" + hex(context.address, 6) +
                " pc=" + hex(context.address, 6) + " omr=" + hex(state.omr, 6));
  }
  const auto opcode = fetch_.select(context.address);
  const auto shape = dsp56300::inspect_instruction(opcode.word.value);
  if (!shape) {
    throw Fault("Unsupported own DSP opcode " + hex(opcode.word.value, 6) +
                " read P:" + hex(context.address, 6) + " pc=" + hex(context.address, 6) +
                " omr=" + hex(state.omr, 6));
  }
  const auto captured = fetch_.instruction(opcode, shape->words);
  captured.check_front(fetch_);
  auto next = fetch_;
  static_cast<void>(next.consume(captured.opcode.word.address));
  if (captured.extension) { static_cast<void>(next.consume(captured.extension->word.address)); }
  captured.check_consumed(next.consumed_pair(), state.omr);
  auto result = admit(captured);
  if (result.fault.kind == dsp56300::FaultKind::none) { fetch_ = std::move(next); }
  return result;
}

dsp56300::InstructionProgress OwnDspFetch::begin_native() {
  auto result = machine_.cpu_.begin_fetch(dsp56300::InstructionPublication::pcu_updates);
  if (result.fault.kind == dsp56300::FaultKind::none) {
    instructions_ = machine_.cpu_.progress().instructions;
    in_flight_ = true;
    native_fetch_ = true;
  }
  return result;
}

dsp56300::InstructionProgress OwnDspFetch::admit(const DspFetchedInstruction &captured) {
  const dsp56300::InstructionWords words{
      .address = captured.opcode.word.address,
      .opcode = captured.opcode.word.value,
      .extension =
          captured.extension ? std::optional(captured.extension->word.value) : std::nullopt};
  auto result =
      machine_.cpu_.begin_instruction(words, dsp56300::InstructionPublication::pcu_updates);
  if (result.fault.kind == dsp56300::FaultKind::none) {
    admitted_ = captured;
    instructions_ = machine_.cpu_.progress().instructions;
    in_flight_ = true;
  }
  return result;
}

void OwnDspFetch::synchronize() {
  check_owner();
  if (!in_flight_ || machine_.cpu_.pending_instruction() ||
      machine_.cpu_.fault().kind != dsp56300::FaultKind::none ||
      machine_.cpu_.progress().instructions != instructions_ + 1) {
    throw Fault("Own DSP F2 synchronization requires its actual retirement pc=" +
                hex(machine_.cpu_.registers().pc, 6));
  }
  if (interrupt_vector_) {
    finish_interrupt_vector();
    native_fetch_ = false;
    in_flight_ = false;
    return;
  }
  const auto context = machine_.cpu_.instruction_fetch();
  const auto regs = machine_.cpu_.registers();
  const auto state = fetch_.view();
  if (native_fetch_ || (regs.sr & cache_enable) != 0) {
    fetch_.redirect(context.address, regs.omr);
    native_latch_ = context.latched_opcode.has_value();
    native_fetch_ = false;
    repeated_.reset();
    admitted_.reset();
    in_flight_ = false;
    return;
  }
  if (context.latched_opcode) {
    if (!admitted_ || admitted_->extension || admitted_->opcode.word.address != context.address ||
        admitted_->opcode.word.value != *context.latched_opcode || state.omr != regs.omr ||
        (repeated_ && repeated_ != admitted_)) {
      throw Fault("Own DSP REP result lost its actual F2 sample read P:" + hex(context.address, 6) +
                  " pc=" + hex(context.address, 6) + " omr=" + hex(regs.omr, 6));
    }
    repeated_ = admitted_;
  } else if (state.consume_address != context.address || state.omr != regs.omr) {
    fetch_.redirect(context.address, regs.omr);
  }
  if (!context.latched_opcode) { repeated_.reset(); }
  admitted_.reset();
  in_flight_ = false;
}

void OwnDspFetch::finish_interrupt_vector() {
  const auto vector = interrupt_vector_;
  if (!vector) { throw Fault("Own DSP interrupt vector ownership is missing"); }
  const auto context = machine_.cpu_.instruction_fetch();
  const auto regs = machine_.cpu_.registers();
  if (machine_.interrupts_.pending()) {
    constexpr unsigned mask = 0xffffff;
    if (!context.exception_vector || context.address != ((*vector + 1U) & mask)) {
      throw Fault("Unqualified DSP interrupt vector continuation read P:" +
                  hex(context.address, 6) + " pc=" + hex(regs.pc, 6) + " omr=" + hex(regs.omr, 6));
    }
    return;
  }
  if (context.exception_vector) {
    throw Fault("Own DSP IRQ acknowledged before native vector completion read P:" +
                hex(context.address, 6) + " pc=" + hex(regs.pc, 6) + " omr=" + hex(regs.omr, 6));
  }
  fetch_.redirect(context.address, regs.omr);
  interrupt_vector_.reset();
}
}
