// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/own_dsp_fetch.hpp"

#include "common/bus.hpp"
#include "dsp/detail/dsp_dma_resource.hpp"
#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_loop_publication.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/instruction.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>

namespace blofeld {
namespace {
constexpr unsigned cache_enable = 1U << 19U;

DspFetchedWord front_opcode(const DspProgramFetch &fetch, const std::uint32_t address) {
  const auto front = fetch.queued(0);
  if (!front || front->word.address != address) {
    throw Fault("Native opcode differs from the actual F2 front read P:" + hex(address, 6) +
                " pc=" + hex(fetch.view().consume_address, 6) + " omr=" + hex(fetch.view().omr, 6));
  }
  // Equal PCs in different loop iterations retain distinct F2 identities.
  return fetch.select(address, front->occurrence);
}

bool dma_controller_access(const dsp56300::PendingInstruction &pending) {
  // FM ch10: controller registers are separate from P-memory fetch resources.
  // Native handlers retain read-only, unknown-state and busy-write checks.
  // Atomic bit effects also have read footprints and remain separately guarded.
  return (pending.effect.kind == dsp56300::EffectKind::read ||
          pending.effect.kind == dsp56300::EffectKind::write) &&
         pending.effect.space == dsp56300::Space::x && DspDma::handles(pending.effect.address);
}

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
  // Private tokens own unique allocations; matching ownership proves liveness.
  if (machine_.captured_fetch_owner_.owner_before(identity_) ||
      identity_.owner_before(machine_.captured_fetch_owner_)) {
    throw Fault("Stale own DSP captured fetch pc=" + hex(machine_.cpu_.registers().pc, 6));
  }
  machine_.require_running();
  if (machine_.advancing_effect_) { throw Fault("Own DSP F2 requires an inactive effect"); }
}

void OwnDspFetch::check_sample_context(const bool hold_fetch) const {
  const auto regs = machine_.cpu_.registers();
  const bool has_pending = !in_flight_ || machine_.cpu_.pending_instruction().has_value();
  check_sample_context(regs, has_pending, hold_fetch);
}

void OwnDspFetch::check_sample_context(const dsp56300::Registers &regs,
                                       const bool has_pending,
                                       const bool hold_fetch) const {
  const auto state = fetch_.view();
  if (in_flight_ && !has_pending) {
    throw Fault("Own DSP F2 requires retirement synchronization read P:" +
                hex(state.pending_address.value_or(state.next_address), 6) +
                " pc=" + hex(regs.pc, 6) + " omr=" + hex(regs.omr, 6));
  }
  const bool held_repeat = hold_fetch && in_flight_ && (repeated_ || native_latch_);
  const bool held_vector = hold_fetch && in_flight_ && native_fetch_ && interrupt_vector_;
  if ((regs.sr & cache_enable) != 0 || (native_fetch_ && !held_vector && !held_repeat) ||
      (native_latch_ && !held_repeat) || (state.omr != regs.omr && !(hold_fetch && in_flight_))) {
    throw Fault("Unqualified own DSP F2 memory context read P:" +
                hex(state.pending_address.value_or(state.next_address), 6) +
                " pc=" + hex(regs.pc, 6) + " omr=" + hex(regs.omr, 6));
  }
  if (repeated_ && !held_repeat) {
    throw Fault("Own DSP REP F2 prefetch is unqualified read P:" +
                hex(state.pending_address.value_or(state.next_address), 6) +
                " pc=" + hex(regs.pc, 6) + " omr=" + hex(regs.omr, 6));
  }
  if (interrupt_vector_ && !(hold_fetch && in_flight_ && native_fetch_)) {
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
  check_reference_timing();
  check_sample_context();
  fetch_.issue();
}

void OwnDspFetch::capture() {
  check_owner();
  check_reference_timing();
  check_sample_context();
  fetch_.capture(machine_.bus_);
}

OwnDspFetchCycleResult OwnDspFetch::advance_dma_capture(const DspDmaInputs inputs) {
  return capture_cycle(inputs, false, true, false, false);
}

OwnDspFetchCycleResult OwnDspFetch::advance_dma_capture_native(const DspDmaInputs inputs,
                                                               const OwnDspNativeCycleModel model) {
  if (model != OwnDspNativeCycleModel::isolated_effect) {
    throw Fault("Own DSP DMA/F2/native cycle requires an isolated-effect hypothesis pc=" +
                hex(machine_.cpu_.registers().pc, 6));
  }
  return capture_cycle(inputs, true, true, false, false);
}

OwnDspFetchCycleResult OwnDspFetch::advance_dma_pipeline(const DspDmaInputs inputs,
                                                         const bool admit_native,
                                                         const OwnDspNativeCycleModel model) {
  if (admit_native && model != OwnDspNativeCycleModel::isolated_effect) {
    throw Fault("Own DSP pipeline requires an isolated-effect hypothesis pc=" +
                hex(machine_.cpu_.registers().pc, 6));
  }
  return capture_cycle(inputs, admit_native, false, true, false);
}

OwnDspFetchCycleResult OwnDspFetch::advance_dma_automatic(const DspDmaInputs inputs,
                                                          const OwnDspAutomaticTiming timing) {
  check_owner();
  const bool native_budget = timing == OwnDspAutomaticTiming::native_cycle_budget;
  if (timing != OwnDspAutomaticTiming::one_effect_per_cycle &&
      timing != OwnDspAutomaticTiming::one_effect_per_cycle_program_write_priority &&
      timing != OwnDspAutomaticTiming::one_effect_per_cycle_write_priority_with_interrupts &&
      timing != OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts &&
      !native_budget) {
    throw Fault("Own DSP automatic execution requires an explicit timing hypothesis pc=" +
                hex(machine_.cpu_.registers().pc, 6));
  }
  if (in_flight_ && native_budget != budget_active_) {
    throw Fault("DSP automatic timing cannot change during an active instruction pc=" +
                hex(machine_.cpu_.registers().pc, 6));
  }
  const bool started = begin_automatic(
      timing == OwnDspAutomaticTiming::one_effect_per_cycle_write_priority_with_interrupts ||
      timing == OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts ||
      native_budget);
  if (started && native_budget) {
    budget_active_ = true;
    native_cycles_ = 0;
  }
  const NativeSample sample{.regs = machine_.cpu_.registers(),
                            .pending = machine_.cpu_.pending_instruction()};
  const bool stale_context = in_flight_ && fetch_.view().omr != sample.regs.omr;
  const bool hold_fetch = hold_program_fetch(timing, sample.pending) || stale_context ||
                          repeated_ || native_latch_ || interrupt_vector_;
  auto result = capture_cycle(inputs, in_flight_, false, true, hold_fetch, sample, native_budget);
  result.instruction_started = started;
  if (result.native && result.native->retired) {
    synchronize();
    budget_active_ = false;
    native_cycles_ = 0;
    if (native_budget) { issue_after_retirement(result); }
  }
  return result;
}

bool OwnDspFetch::hold_program_fetch(
    const OwnDspAutomaticTiming timing,
    const std::optional<dsp56300::PendingInstruction> &pending) const {
  if (pending && pending->effect.kind == dsp56300::EffectKind::read &&
      pending->effect.space == dsp56300::Space::program &&
      (timing == OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts ||
       timing == OwnDspAutomaticTiming::native_cycle_budget)) {
    return fetch_.view().pending_address.has_value();
  }
  if (!pending || pending->effect.kind != dsp56300::EffectKind::write ||
      pending->effect.space != dsp56300::Space::program) {
    return false;
  }
  const auto state = fetch_.view();
  const auto address = pending->effect.address;
  bool already_fetched = state.pending_address == address;
  for (std::size_t index = 0; index < state.used; ++index) {
    already_fetched = already_fetched || state.words.at(index).address == address;
  }
  if (already_fetched) {
    throw Fault("Unqualified own DSP program write to issued/fetched P:" + hex(address, 6) +
                " pc=" + hex(pending->effect.pc, 6) + " omr=" + hex(pending->effect.omr, 6));
  }
  return (timing == OwnDspAutomaticTiming::one_effect_per_cycle_program_write_priority ||
          timing == OwnDspAutomaticTiming::one_effect_per_cycle_write_priority_with_interrupts ||
          timing == OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts ||
          timing == OwnDspAutomaticTiming::native_cycle_budget) &&
         state.pending_address.has_value();
}

bool OwnDspFetch::begin_automatic(const bool admit_interrupts) {
  if (in_flight_) { return false; }
  const auto regs = machine_.cpu_.registers();
  const auto context = machine_.cpu_.instruction_fetch();
  if (interrupt_vector_ && admit_interrupts) {
    const auto progress = begin();
    if (progress.fault.kind != dsp56300::FaultKind::none) {
      throw Fault("Own DSP automatic vector admission fault pc=" + hex(regs.pc, 6));
    }
    return true;
  }
  if ((regs.sr & cache_enable) != 0 || native_fetch_ || interrupt_vector_ ||
      context.exception_vector || ((repeated_ || native_latch_) && !context.latched_opcode)) {
    throw Fault("Unqualified own DSP automatic cache/REP/vector fetch read P:" +
                hex(context.address, 6) + " pc=" + hex(regs.pc, 6) + " omr=" + hex(regs.omr, 6));
  }
  constexpr unsigned mask_shift = 8;
  constexpr unsigned mask_bits = 3;
  const auto request = machine_.bus_.interrupt_request((regs.sr >> mask_shift) & mask_bits);
  if (request && !admit_interrupts) {
    throw Fault("Unqualified own DSP automatic interrupt vector=" + hex(request->vector, 6) +
                " pc=" + hex(regs.pc, 6) + " omr=" + hex(regs.omr, 6));
  }
  if (context.latched_opcode) {
    const auto progress = begin();
    if (progress.fault.kind != dsp56300::FaultKind::none) {
      throw Fault("Own DSP automatic REP admission fault pc=" + hex(regs.pc, 6));
    }
    return true;
  }
  const auto state = fetch_.view();
  if (state.used == 0) { return false; }
  const auto opcode = state.words[0].require_known({.pc = context.address, .omr = regs.omr});
  const auto shape = dsp56300::inspect_instruction(opcode.value);
  if (!shape) {
    throw Fault("Unsupported own DSP automatic opcode " + hex(opcode.value, 6) + " read P:" +
                hex(opcode.address, 6) + " pc=" + hex(regs.pc, 6) + " omr=" + hex(regs.omr, 6));
  }
  if (state.used < shape->words) { return false; }
  if (request && admit_interrupts && admit_interrupt()) {
    const auto progress = begin();
    if (progress.fault.kind != dsp56300::FaultKind::none) {
      throw Fault("Own DSP automatic vector admission fault pc=" + hex(regs.pc, 6));
    }
    return true;
  }
  const auto progress = begin();
  if (progress.fault.kind != dsp56300::FaultKind::none) {
    throw Fault("Own DSP automatic instruction admission fault pc=" + hex(regs.pc, 6) +
                " opcode=" + hex(opcode.value, 6));
  }
  return true;
}

std::optional<DspMemoryAccess>
OwnDspFetch::native_overlap(const DspProgramFetchView &state,
                            const dsp56300::PendingInstruction &pending) const {
  const auto access = OwnDspMachine::dma_native_footprint(pending);
  if (dma_controller_access(pending)) { return access; }
  if (access && pending.effect.kind == dsp56300::EffectKind::read &&
      machine_.bus_.unselected_read(access->target)) {
    return access;
  }
  if (access && detail::independent_dma_peripheral(*access)) { return access; }
  if (access && access->operation == DspMemoryOperation::write &&
      machine_.bus_.unselected_write(access->target)) {
    return access;
  }
  if (!access || !state.pending_address) { return access; }
  constexpr unsigned io_base = 0xffff80;
  const DspMemoryAccess program{.target = {.address = *state.pending_address}};
  const bool data = access->target.space != DspSpace::p;
  const bool peripheral_write = data && access->target.address >= io_base &&
                                access->operation == DspMemoryOperation::write &&
                                !machine_.bus_.has_active_dma();
  constexpr unsigned shi_status = 0xffff91;
  constexpr unsigned shi_receive = 0xffff94;
  const bool shi_read =
      access->target.space == DspSpace::x &&
      ((access->target.address == shi_status && access->phase == DspMemoryPhase::peripheral_bit) ||
       (access->target.address == shi_receive &&
        access->phase == DspMemoryPhase::peripheral_sample)) &&
      !machine_.bus_.has_active_dma();
  const bool gpio_read = access->phase == DspMemoryPhase::peripheral_sample &&
                         DspBus::handles_peripheral_bit(access->target) &&
                         !machine_.bus_.has_active_dma();
  constexpr unsigned primary_status = 0xffffb3;
  constexpr unsigned secondary_status = 0xffff93;
  const bool esai_read =
      access->phase == DspMemoryPhase::peripheral_sample &&
      ((access->target.space == DspSpace::x && access->target.address == primary_status) ||
       (access->target.space == DspSpace::y && access->target.address == secondary_status)) &&
      !machine_.bus_.has_active_dma();
  const bool peripheral_bit = access->phase == DspMemoryPhase::peripheral_bit &&
                              DspBus::handles_peripheral_bit(access->target) &&
                              !machine_.bus_.has_active_dma();
  // FM table10-10: read-only DMA status uses its controller register route,
  // distinct from P RAM. The explicit DMA-first cycle samples the real bit later.
  constexpr unsigned dma_status = 0xfffff4;
  const bool dma_status_test = access->phase == DspMemoryPhase::peripheral_bit &&
                               pending.effect.kind == dsp56300::EffectKind::bit &&
                               pending.effect.action == dsp56300::BitAction::test &&
                               pending.effect.space == dsp56300::Space::x &&
                               pending.effect.address == dma_status;
  if (peripheral_write || shi_read || gpio_read || esai_read || peripheral_bit || dma_status_test ||
      (data && access->target.address < io_base &&
       machine_.bus_.memory_conflict({.core = program, .dma = *access}) ==
           DspMemoryConflict::independent)) {
    return access;
  }
  constexpr std::array spaces{"P:", "X:", "Y:"};
  const auto *operation = access->operation == DspMemoryOperation::read ? "read " : "write ";
  const auto regs = machine_.cpu_.registers();
  throw Fault(
      "Unqualified own DSP F2/native overlap read P:" + hex(*state.pending_address, 6) +
      " native=" + std::string(operation) + spaces.at(static_cast<unsigned>(access->target.space)) +
      hex(access->target.address, 6) + " pc=" + hex(regs.pc, 6) + " omr=" + hex(regs.omr, 6));
}

void OwnDspFetch::observe_native_and_issue(OwnDspFetchCycleResult &result, const bool requested) {
  const auto updated = fetch_.view();
  const bool observe_omr = result.native && result.native->fault.kind == dsp56300::FaultKind::none;
  const bool retired = result.native && result.native->retired;
  const bool issue = requested && !result.dma.core_stall && !retired &&
                     machine_.cpu_.fault().kind == dsp56300::FaultKind::none &&
                     !updated.pending_address && updated.used < updated.words.size() &&
                     !native_fetch_ && !native_latch_ && !repeated_ && !interrupt_vector_;
  if (!observe_omr && !issue) { return; }
  const auto after = machine_.cpu_.registers();
  if (observe_omr) { machine_.bus_.observe_omr(after.omr); }
  if (issue && updated.omr == after.omr && (after.sr & cache_enable) == 0) {
    fetch_.issue();
    result.issued_address = fetch_.view().pending_address;
  }
}

OwnDspFetchCycleResult OwnDspFetch::capture_cycle(const DspDmaInputs inputs,
                                                  const bool admit_native,
                                                  const bool require_capture,
                                                  const bool issue_next,
                                                  const bool hold_fetch) {
  check_owner();
  const NativeSample sample{.regs = machine_.cpu_.registers(),
                            .pending = machine_.cpu_.pending_instruction()};
  return capture_cycle(inputs, admit_native, require_capture, issue_next, hold_fetch, sample);
}

OwnDspFetchCycleResult OwnDspFetch::capture_cycle(const DspDmaInputs inputs,
                                                  const bool admit_native,
                                                  const bool require_capture,
                                                  const bool issue_next,
                                                  const bool hold_fetch,
                                                  const NativeSample &sample,
                                                  const bool native_budget) {
  check_owner();
  if (!native_budget) { check_reference_timing(); }
  const auto &regs = sample.regs;
  const auto &pending = sample.pending;
  check_sample_context(regs, pending.has_value(), hold_fetch);
  const auto state = fetch_.view();
  const auto context = [&regs] { return " pc=" + hex(regs.pc, 6) + " omr=" + hex(regs.omr, 6); };
  if (machine_.clock_inputs_.switch_timing != DspClockSwitch::reference_cycle_boundary ||
      machine_.cpu_.fault().kind != dsp56300::FaultKind::none ||
      machine_.cpu_.processing_state() != dsp56300::ProcessingState::running || !state.active ||
      (require_capture && !state.pending_address) ||
      (state.pending_address && state.used == state.words.size())) {
    throw Fault("Own DSP DMA/F2 cycle requires a qualified fetch boundary" + context());
  }
  std::array<DspMemoryAccess, 2> accesses{};
  std::size_t count = 0;
  if (state.pending_address && !hold_fetch) {
    accesses[count++] = {.target = {.address = *state.pending_address}};
  }
  auto native_state = state;
  if (hold_fetch) { native_state.pending_address.reset(); }
  if (admit_native) {
    if (!in_flight_ || !pending) {
      throw Fault("Own DSP DMA/F2/native cycle requires its pending owned effect" + context());
    }
    if (const auto access = native_overlap(native_state, *pending)) { accesses[count++] = *access; }
  }
  const CycleGuard guard(machine_.advancing_effect_);
  static_cast<void>(machine_.clock_.next_step_ticks(machine_.bus_, 1));
  const bool switch_was_pending = machine_.bus_.switch_delay_ != 0;
  machine_.clock_.begin_step(machine_.bus_, {.pc = regs.pc, .omr = regs.omr});
  try {
    OwnDspFetchCycleResult result;
    result.dma = machine_.bus_.advance_dma(
        inputs,
        {.accesses = std::span(accesses).first(count),
         .knowledge = DspCoreBusKnowledge::reference_cycle_occupancy,
         .indivisible_instruction = machine_.bus_.has_active_dma() &&
                                    OwnDspMachine::dma_indivisible_instruction(pending)});
    if (!result.dma.core_stall) {
      if (state.pending_address && !hold_fetch) {
        fetch_.capture(machine_.bus_);
        result.captured = true;
      }
      if (admit_native) { advance_native(result, *pending, native_budget); }
    }
    observe_native_and_issue(result, issue_next);
    machine_.clock_.finish_dma_cycle(machine_.bus_, inputs.cycle, switch_was_pending);
    return result;
  } catch (const std::exception &error) {
    machine_.clock_.fail();
    throw Fault(
        "Own DSP DMA/F2 cycle P:" + hex(state.pending_address.value_or(state.next_address), 6) +
        context() + ": " + error.what());
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
  const auto opcode = front_opcode(fetch_, context.address);
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
  static_cast<void>(fetch_.suspend_loop_forecast(captured));
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
  const auto opcode = front_opcode(fetch_, context.address);
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
  const auto outcome = machine_.cpu_.instruction_outcome();
  if (!in_flight_ || machine_.cpu_.pending_instruction() ||
      machine_.cpu_.fault().kind != dsp56300::FaultKind::none ||
      machine_.cpu_.progress().instructions != instructions_ + 1 || !outcome ||
      outcome->instructions != instructions_ + 1) {
    throw Fault("Own DSP F2 synchronization requires its actual retirement pc=" +
                hex(machine_.cpu_.registers().pc, 6));
  }
  if (!native_fetch_ && (!admitted_ || admitted_->opcode.word.address != outcome->address ||
                         admitted_->opcode.word.value != outcome->opcode)) {
    throw Fault("DSP F2 retirement differs from its admitted instruction read P:" +
                hex(outcome->address, 6) + " pc=" + hex(machine_.cpu_.registers().pc, 6) +
                " omr=" + hex(machine_.cpu_.registers().omr, 6));
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
  if (context.exception_vector) {
    interrupt_vector_ = context.address;
    native_fetch_ = false;
    native_latch_ = false;
    repeated_.reset();
    admitted_.reset();
    in_flight_ = false;
    return;
  }
  if (native_fetch_ || (regs.sr & cache_enable) != 0) {
    fetch_.redirect(context.address, regs.omr);
    native_latch_ = context.latched_opcode.has_value();
    native_fetch_ = false;
    repeated_.reset();
    admitted_.reset();
    in_flight_ = false;
    return;
  }
  if (confirm_loop_publication(fetch_, *admitted_, regs, context, *outcome)) {
    admitted_.reset();
    in_flight_ = false;
    return;
  }
  const DspFetchCommit completed{.pc = outcome->address,
                                 .next_pc = context.address,
                                 .omr = regs.omr,
                                 .redirected = outcome->control_transfer};
  if (context.latched_opcode) {
    if (!admitted_ || admitted_->extension || admitted_->opcode.word.address != context.address ||
        admitted_->opcode.word.value != *context.latched_opcode || state.omr != regs.omr ||
        (repeated_ && repeated_ != admitted_)) {
      throw Fault("Own DSP REP result lost its actual F2 sample read P:" + hex(context.address, 6) +
                  " pc=" + hex(context.address, 6) + " omr=" + hex(regs.omr, 6));
    }
    repeated_ = admitted_;
  } else if (outcome->control_transfer || state.consume_address != context.address ||
             state.omr != regs.omr) {
    redirect_loop_control(fetch_, *admitted_, completed);
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
    throw Fault("Own DSP vector retired before its actual pair acknowledgement read P:" +
                hex(context.address, 6) + " pc=" + hex(regs.pc, 6) + " omr=" + hex(regs.omr, 6));
  }
  if (context.exception_vector) {
    constexpr unsigned mask = 0xffffff;
    if (context.address != ((*vector + 1U) & mask) || !context.latched_opcode) {
      throw Fault("Unqualified DSP interrupt vector continuation read P:" +
                  hex(context.address, 6) + " pc=" + hex(regs.pc, 6) + " omr=" + hex(regs.omr, 6));
    }
    return;
  }
  fetch_.redirect(context.address, regs.omr);
  interrupt_vector_.reset();
}
}
