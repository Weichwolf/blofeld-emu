// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_machine.hpp"

#include "common/bus.hpp"
#include "dsp/detail/dsp_reference_cycle.hpp"
#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_agu_interlocks.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dax.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_interlocks.hpp"
#include "dsp/dsp_interrupts.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_cycle.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_run_clock.hpp"

#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>

namespace blofeld {
namespace {
struct RunState {
  std::optional<DspRunContract> contract;
  std::optional<DspRunClock> clock;
  std::optional<DspAguInterlocks> agu;
  std::optional<DspAguEffect> pending_agu;
  std::optional<DspInterlocks> interlocks;
};

RunState initial_run(const std::optional<DspRunContract> contract) {
  if (!contract) { return {}; }
  const bool agu = contract->timing == DspRunTiming::native_internal_agu_interlocks;
  const bool interlocks = contract->timing == DspRunTiming::native_interlock_estimates;
  if (contract->timing != DspRunTiming::native_internal_estimates && !interlocks && !agu) {
    throw Fault("DSP run requires an explicit native-cycle-estimate timing contract");
  }
  if (contract->interrupts != DspInterruptTiming::unqualified &&
      contract->interrupts != DspInterruptTiming::native_vector_estimates) {
    throw Fault("Unsupported DSP interrupt timing contract");
  }
  // Nominal AGU timing admits ordinary prepared steps; preview/begin still
  // refuse actual IRQ entry. The broader interlock profile has no vector contract.
  if (interlocks && contract->interrupts != DspInterruptTiming::unqualified) {
    throw Fault("DSP interlock estimates do not qualify interrupt-vector timing");
  }
  if (contract->resets != DspResetTiming::unqualified &&
      contract->resets != DspResetTiming::reference_instruction_boundary &&
      contract->resets != DspResetTiming::reference_extal_hold) {
    throw Fault("Unsupported DSP reset timing contract");
  }
  return {.contract = contract,
          .clock = DspRunClock(contract->clock),
          .agu = contract->timing == DspRunTiming::native_internal_agu_interlocks
                     ? std::optional(DspAguInterlocks{})
                     : std::nullopt,
          .pending_agu = std::nullopt,
          .interlocks = interlocks ? std::optional{DspInterlocks{}} : std::nullopt};
}
}

struct DspCheckpoint::Storage {
  Storage(DspBus bus_state, DspCoreCheckpoint cpu_state, RunState run_state)
      : bus(std::move(bus_state)), cpu(std::move(cpu_state)), run(run_state) {}

  const DspBus bus;
  const DspCoreCheckpoint cpu;
  const RunState run;
};

struct DspMachine::State {
  explicit State(const RunState run_state) : run(run_state), cpu(bus) {
    if (run.clock) {
      if (!run.contract) { throw Fault("Clocked DSP owner lost its timing contract"); }
      const bool vector_estimates =
          run.contract.value().interrupts == DspInterruptTiming::native_vector_estimates;
      cpu.set_run_hooks(
          {.ready =
               [this] {
                 if (run.clock->hardware_reset_active()) {
                   const auto context = cpu.state();
                   throw Fault("DSP instruction during hardware reset pc=" + hex(context.pc, 6) +
                               " omr=" + hex(context.omr, 6));
                 }
               },
           .begin =
               [this](const auto state,
                      const bool interrupt,
                      const unsigned units,
                      const std::optional<std::uint32_t> opcode) {
                 begin_step(state, interrupt, units, opcode);
               },
           .complete = [this](const auto cycles) { complete_step(cycles); },
           .failed = [this] { run.clock->fail(); },
           .reset = [this] { reset_clock(); },
           .fetched = run.interlocks ? std::function<void(std::uint32_t)>(
                                           [this](const auto opcode) { observe_opcode(opcode); })
                                     : std::function<void(std::uint32_t)>{},
           .data = run.interlocks ? std::function<void(DspMemoryAccess)>([this](const auto access) {
             static_cast<void>(bus.memory_resource(access));
           })
                                  : std::function<void(DspMemoryAccess)>{},
           .vector_estimates = vector_estimates,
           .timing_opcode = run.agu.has_value()});
    }
  }

  [[nodiscard]] DspAguEffect preview_agu(const DspCore::StepEstimate &estimate) const {
    if (!run.agu) { return {}; }
    if (estimate.interrupt || !estimate.opcode) {
      throw Fault("Unqualified interrupt entry in nominal DSP AGU timing");
    }
    return run.agu->preview(*estimate.opcode);
  }

  void begin_step(const DspState context,
                  const bool interrupt,
                  const unsigned units,
                  const std::optional<std::uint32_t> opcode) {
    if (!run.clock || !run.contract) { throw Fault("DSP run lost its clock contract"); }
    const bool vector_estimates =
        run.contract->interrupts == DspInterruptTiming::native_vector_estimates;
    constexpr unsigned compatibility = 0x2000;
    constexpr unsigned arithmetic_or_loop = 0x28000;
    if ((interrupt && !vector_estimates) || (context.sr & compatibility) != 0) {
      throw Fault("Unqualified DSP interrupt or compatibility-mode timing");
    }
    if (run.interlocks && (context.sr & arithmetic_or_loop) != 0) {
      throw Fault("Unqualified DSP interlock arithmetic/loop-mode timing");
    }
    if (cpu.native_cycles() != run.clock->epoch_native_cycle_units()) {
      throw Fault("DSP clock/native counters require coordinated recovery");
    }
    if (run.agu) {
      if (interrupt || !opcode) {
        throw Fault("Unqualified interrupt entry in nominal DSP AGU timing");
      }
      run.pending_agu = run.agu->preview(*opcode);
    }
    prepare_interlocks(context, units);
    run.clock->begin_step(bus, {.pc = context.pc, .omr = context.omr});
  }

  [[nodiscard]] unsigned preview_stalls(const DspState context, const unsigned units) const {
    if (!run.interlocks) { return 0; }
    try {
      constexpr unsigned unsupported_status = 0x2a000; // SA, LF, SC.
      if ((context.sr & unsupported_status) != 0) {
        throw Fault("Unqualified DSP interlock arithmetic/loop/compatibility mode");
      }
      return run.interlocks->preview(bus.inspect_program(context.pc), units);
    } catch (const std::exception &error) {
      throw Fault("DSP interlock opcode read P:" + hex(context.pc, 6) +
                  " omr=" + hex(context.omr, 6) + ": " + error.what());
    }
  }

  void prepare_interlocks(const DspState context, const unsigned units) {
    static_cast<void>(preview_stalls(context, units));
    if (run.interlocks) { run.interlocks->prepare(bus.inspect_program(context.pc), units); }
  }

  void observe_opcode(const std::uint32_t opcode) {
    if (!run.interlocks) { throw Fault("DSP opcode timing hook lost its dependency state"); }
    run.interlocks->observe_opcode(opcode);
  }

  void complete_step(const unsigned units) {
    if (!run.clock) { throw Fault("DSP instruction completion lost its clock"); }
    const auto effect = run.pending_agu.value_or(DspAguEffect{});
    const auto stalls = effect.stalls + (run.interlocks ? run.interlocks->complete(units) : 0);
    run.clock->finish_step(bus, DspClockUnits{.native = units, .interlocks = stalls});
    if (run.agu) { run.agu->complete(effect, units); }
    run.pending_agu.reset();
  }

  void reset_clock() {
    if (!run.clock || !run.contract) { throw Fault("Clocked DSP reset lost its timing contract"); }
    auto &clock = run.clock.value();
    const auto context = cpu.state();
    if (clock.faulted() || clock.active() || cpu.faulted() ||
        cpu.native_cycles() != clock.epoch_native_cycle_units()) {
      throw Fault("Clocked DSP reset requires coordinated recovery pc=" + hex(context.pc, 6) +
                  " omr=" + hex(context.omr, 6));
    }
    if (clock.native_cycle_units() != 0 &&
        run.contract.value().resets == DspResetTiming::unqualified) {
      throw Fault("Clocked DSP reset requires explicit reference reset timing pc=" +
                  hex(context.pc, 6) + " omr=" + hex(context.omr, 6));
    }
    clock.restart_native_epoch();
    if (run.agu) { run.agu = DspAguInterlocks{}; }
    run.pending_agu.reset();
    if (run.interlocks) { run.interlocks->reset(); }
  }

  void check_boundary() const {
    const auto &clock = run.clock;
    if (clock && (clock->faulted() || clock->active() ||
                  cpu.native_cycles() != clock->epoch_native_cycle_units())) {
      throw Fault("DSP checkpoint requires coordinated healthy reference clock counters");
    }
  }

  DspBus bus;
  RunState run;
  DspCore cpu;
  const std::shared_ptr<const bool> execution_owner = std::make_shared<const bool>(true);
};

DspCheckpoint::DspCheckpoint(std::shared_ptr<const Storage> storage)
    : storage_(std::move(storage)) {}

DspCheckpoint::~DspCheckpoint() = default;

DspMachine::DspMachine() : DspMachine(std::nullopt) {}

DspMachine::DspMachine(const std::optional<DspRunContract> run)
    : state_(std::make_unique<State>(initial_run(run))) {}

DspMachine::~DspMachine() = default;

DspBus &DspMachine::bus() {
  return state_->bus;
}

DspCore &DspMachine::cpu() {
  return state_->cpu;
}

const DspBus &DspMachine::bus() const {
  return state_->bus;
}

const DspCore &DspMachine::cpu() const {
  return state_->cpu;
}

bool DspMachine::clocked() const {
  return state_->run.clock.has_value();
}

bool DspMachine::hardware_reset_active() const {
  const auto &clock = state_->run.clock;
  return clock && clock->hardware_reset_active();
}

void DspMachine::require_running() const {
  if (hardware_reset_active()) {
    const auto context = state_->cpu.state();
    throw Fault("DSP operation during hardware reset pc=" + hex(context.pc, 6) +
                " omr=" + hex(context.omr, 6));
  }
}

void DspMachine::begin_hardware_reset(const DspHardwareResetInputs inputs) {
  constexpr unsigned all_modes = 15;
  constexpr unsigned spi_mode = 5;
  const auto contract = state_->run.contract;
  auto &optional_clock = state_->run.clock;
  const auto context = state_->cpu.state();
  if (!contract || !optional_clock || contract->resets != DspResetTiming::reference_extal_hold ||
      inputs.mode_known != all_modes || inputs.mode_levels != spi_mode || !inputs.pinit ||
      inputs.pinit.value() || inputs.recovery_cycles < 2 || hardware_reset_active() ||
      state_->cpu.executing()) {
    throw Fault("DSP hardware reset requires an inactive explicit EXTAL-hold contract, stable "
                "observed mode5/PINIT-low and qualified recovery of at least2 periods pc=" +
                hex(context.pc, 6) + " omr=" + hex(context.omr, 6));
  }
  auto &clock = optional_clock.value();
  state_->check_boundary();
  try {
    state_->bus.observe_irqa((inputs.mode_levels & 1U) != 0);
    state_->bus.pll().observe_pinit(inputs.pinit);
    state_->cpu.reset_spi_boot();
    clock.begin_hardware_reset(inputs.recovery_cycles);
  } catch (...) {
    clock.fail();
    throw;
  }
}

std::uint64_t DspMachine::advance_hardware_reset(const std::uint64_t maximum_extal_cycles) {
  auto &optional_clock = state_->run.clock;
  if (!optional_clock || !optional_clock->hardware_reset_active()) {
    throw Fault("DSP hardware reset is not active");
  }
  auto &clock = optional_clock.value();
  state_->check_boundary();
  try {
    return clock.advance_hardware_reset(maximum_extal_cycles);
  } catch (const std::exception &error) {
    clock.fail();
    const auto context = state_->cpu.state();
    throw Fault("DSP reset EXTAL advance pc=" + hex(context.pc, 6) + " omr=" + hex(context.omr, 6) +
                ": " + error.what());
  }
}

void DspMachine::release_hardware_reset() {
  auto &optional_clock = state_->run.clock;
  if (!optional_clock || !optional_clock->hardware_reset_active()) {
    throw Fault("DSP hardware reset is not active");
  }
  auto &clock = optional_clock.value();
  state_->check_boundary();
  try {
    clock.release_hardware_reset();
  } catch (const std::exception &error) {
    clock.fail();
    const auto context = state_->cpu.state();
    throw Fault("DSP reset release pc=" + hex(context.pc, 6) + " omr=" + hex(context.omr, 6) +
                ": " + error.what());
  }
}

std::uint64_t DspMachine::advance_hardware_reset_ticks(const std::uint64_t maximum_ticks) {
  auto &optional_clock = state_->run.clock;
  if (!optional_clock || !optional_clock->hardware_reset_active()) {
    throw Fault("DSP hardware reset is not active");
  }
  auto &clock = optional_clock.value();
  state_->check_boundary();
  try {
    return clock.advance_hardware_reset_ticks(maximum_ticks);
  } catch (const std::exception &error) {
    clock.fail();
    const auto context = state_->cpu.state();
    throw Fault("DSP reset time advance pc=" + hex(context.pc, 6) + " omr=" + hex(context.omr, 6) +
                ": " + error.what());
  }
}

const DspRunClock &DspMachine::run_clock() const {
  const auto &clock = state_->run.clock;
  if (!clock) { throw Fault("DSP machine has no reference run clock"); }
  return *clock;
}

bool DspMachine::timing_faulted() const {
  const auto &clock = state_->run.clock;
  return clock && (clock->faulted() || state_->cpu.faulted());
}

std::optional<DspDmaWord> DspMachine::advance_dma(const DspDmaInputs inputs) {
  if (state_->cpu.executing() || state_->cpu.faulted() || timing_faulted()) {
    throw Fault("DSP DMA observation requires a healthy native boundary");
  }
  try {
    return state_->bus.advance_dma(inputs);
  } catch (const std::exception &error) {
    const auto context = state_->cpu.state();
    throw Fault("DSP DMA observation pc=" + hex(context.pc, 6) + " omr=" + hex(context.omr, 6) +
                ": " + error.what());
  }
}

void DspMachine::observe_dax(const DspDaxInputs inputs) {
  if (state_->cpu.executing() || state_->cpu.faulted() || timing_faulted()) {
    throw Fault("DAX observation requires a healthy native boundary");
  }
  try {
    state_->bus.observe_dax(inputs);
  } catch (const std::exception &error) {
    const auto context = state_->cpu.state();
    throw Fault("DSP DAX observation pc=" + hex(context.pc, 6) + " omr=" + hex(context.omr, 6) +
                ": " + error.what());
  }
}

DspDmaBusResult DspMachine::advance_dma(const DspDmaInputs inputs, const DspCoreBusUse core) {
  if (state_->cpu.executing() || state_->cpu.faulted() || timing_faulted()) {
    throw Fault("DSP DMA arbitration requires a healthy native boundary");
  }
  try {
    return state_->bus.advance_dma(inputs, core);
  } catch (const std::exception &error) {
    const auto context = state_->cpu.state();
    throw Fault("DSP DMA arbitration pc=" + hex(context.pc, 6) + " omr=" + hex(context.omr, 6) +
                ": " + error.what());
  }
}

DspStepPlan DspMachine::plan_step() const {
  require_running();
  const auto &clock = run_clock();
  state_->check_boundary();
  const auto before = state_->cpu.debug_state();
  try {
    const auto run_contract = state_->run.contract;
    if (!run_contract) { throw Fault("Clocked DSP owner lost its timing contract"); }
    const bool vector_estimates =
        run_contract->interrupts == DspInterruptTiming::native_vector_estimates;
    const auto estimate =
        state_->cpu.next_step_cycles(vector_estimates, nullptr, state_->run.agu.has_value());
    const auto agu = state_->preview_agu(estimate);
    const auto stalls = agu.stalls + state_->preview_stalls(before.cpu, estimate.units);
    return {.before = before,
            .native_cycle_units = estimate.units,
            .interlock_cycle_units = stalls,
            .start_ticks = clock.elapsed_ticks(),
            .duration_ticks = clock.next_step_ticks(
                state_->bus, DspClockUnits{.native = estimate.units, .interlocks = stalls}),
            .interrupt = estimate.interrupt};
  } catch (const std::exception &error) {
    throw Fault("DSP run plan pc=" + hex(before.cpu.pc, 6) + " omr=" + hex(before.cpu.omr, 6) +
                ": " + error.what());
  }
}

void DspMachine::check_admission_contract() const {
  static_cast<void>(run_clock());
  const auto contract = state_->run.contract;
  if (!contract || contract->interrupts != DspInterruptTiming::native_vector_estimates) {
    throw Fault("Prepared DSP interrupt admission requires explicit native vector estimates");
  }
}

void DspMachine::verify_prepared_step(const DspStepPlan &plan) const {
  require_running();
  check_admission_contract();
  state_->check_boundary();
  const auto before = state_->cpu.debug_state();
  const auto &clock = run_clock();
  try {
    if (before != plan.before || clock.elapsed_ticks() != plan.start_ticks) {
      throw Fault("DSP admitted instruction lost its native boundary");
    }
    DspCore::Admission admission;
    if (plan.interrupt) { admission.interrupt = plan.interrupt->request; }
    const auto estimate =
        state_->cpu.next_step_cycles(true, &admission, state_->run.agu.has_value());
    const auto agu = state_->preview_agu(estimate);
    const auto stalls = agu.stalls + state_->preview_stalls(before.cpu, estimate.units);
    if (estimate.units != plan.native_cycle_units || estimate.interrupt != plan.interrupt ||
        stalls != plan.interlock_cycle_units ||
        clock.next_step_ticks(state_->bus,
                              DspClockUnits{.native = estimate.units, .interlocks = stalls}) !=
            plan.duration_ticks) {
      throw Fault("DSP admitted instruction changed before its declared completion");
    }
  } catch (const std::exception &error) {
    throw Fault("DSP admitted plan pc=" + hex(before.cpu.pc, 6) + " omr=" + hex(before.cpu.omr, 6) +
                ": " + error.what());
  }
}

void DspMachine::execute_prepared_step(const DspStepPlan &plan) {
  verify_prepared_step(plan);
  std::optional<DspInterruptRequest> request;
  if (plan.interrupt) { request = plan.interrupt->request; }
  state_->cpu.step_admitted(request);
}

DspReferenceProgress DspMachine::begin_reference_instruction(const DspReferenceExecution contract) {
  if (clocked()) { throw Fault("Native access suspension requires an unclocked reference owner"); }
  return state_->cpu.begin_reference_instruction(contract);
}

void DspMachine::validate_reference_overlap(const DspReferenceTicket &ticket) const {
  if (clocked()) { throw Fault("Native overlap requires an unclocked reference owner"); }
  state_->cpu.validate_reference_overlap(ticket);
}

DspReferenceProgress DspMachine::overlap_reference_instruction(const DspReferenceTicket &ticket) {
  if (clocked()) { throw Fault("Native overlap requires an unclocked reference owner"); }
  return state_->cpu.overlap_reference_instruction(ticket);
}

std::optional<DspReferencePending> DspMachine::reference_retirement() const {
  return state_->cpu.reference_retirement();
}

DspInstructionRetire DspMachine::retire_reference_overlap(const DspReferenceTicket &ticket) {
  if (clocked()) { throw Fault("Native overlap requires an unclocked reference owner"); }
  return state_->cpu.retire_reference_overlap(ticket);
}

DspReferenceProgress DspMachine::begin_reference_interrupt(const DspReferenceExecution contract) {
  if (clocked()) { throw Fault("Native access suspension requires an unclocked reference owner"); }
  return state_->cpu.begin_reference_interrupt(contract);
}

DspReferencePending DspMachine::latch_reference_program(const DspReferenceTicket &ticket) {
  if (clocked()) { throw Fault("Program latching requires an unclocked reference owner"); }
  return state_->cpu.latch_reference_program(ticket);
}

DspProgramFetchView DspMachine::start_reference_fetch(const DspReferenceTicket &ticket) {
  if (clocked()) { throw Fault("Program fetching requires an unclocked reference owner"); }
  return state_->cpu.start_reference_fetch(ticket);
}

DspProgramFetchView DspMachine::advance_reference_fetch(const DspProgramFetchStage stage) {
  if (clocked()) { throw Fault("Program fetching requires an unclocked reference owner"); }
  return state_->cpu.advance_reference_fetch(stage);
}

DspProgramFetchView DspMachine::reference_fetch() const {
  return state_->cpu.reference_fetch();
}

DspReferenceProgress DspMachine::advance_reference_access(const DspReferenceTicket &ticket) {
  if (clocked()) { throw Fault("Native access suspension requires an unclocked reference owner"); }
  return state_->cpu.advance_reference_access(ticket);
}

DspAddressStageTicket DspMachine::capture_reference_address(const std::uint32_t address) {
  if (clocked()) { throw Fault("Native AG1 requires an unclocked reference owner"); }
  return state_->cpu.capture_reference_address(address);
}

DspAddressStageView DspMachine::address_stage(const DspAddressStageTicket &ticket) const {
  return state_->cpu.address_stage(ticket);
}

DspAddressStageTicket DspMachine::rebind_address_stage(const DspAddressStageTicket &ticket) const {
  return state_->cpu.rebind_address_stage(ticket);
}

std::optional<DspReferencePending> DspMachine::reference_pending() const {
  return state_->cpu.reference_pending();
}

DspReferenceCycleResult
DspMachine::advance_reference_bus_cycle(const DspReferenceCycleInputs &inputs) {
  if (clocked() || state_->cpu.executing()) {
    throw Fault("Composed native bus cycles require an unclocked healthy reference owner");
  }
  const auto pending = state_->cpu.reference_pending();
  const auto context = state_->cpu.state();
  DspReferenceCycleResult result;
  try {
    if (inputs.order != DspReferenceBusOrder::dma_then_fetch_then_native) {
      throw Fault("Native bus cycle requires explicit DMA/fetch/native order");
    }
    if (inputs.retirement) { state_->cpu.validate_reference_retirement(*inputs.retirement); }
    if (inputs.access) {
      state_->cpu.validate_reference_admission(*inputs.access, inputs.retirement.has_value());
    }
    if (inputs.fetch) { state_->cpu.validate_reference_fetch(*inputs.fetch); }
    std::optional<DspAddressStageView> memory;
    if (inputs.address_update) {
      state_->cpu.validate_reference_address(*inputs.address_update, false);
    }
    if (inputs.memory) {
      state_->cpu.validate_reference_address(*inputs.memory, true);
      memory = state_->cpu.address_stage(*inputs.memory);
    }
    result = reference_cycle_footprint(
        state_->bus, inputs, pending, state_->cpu.reference_fetch(), memory);
  } catch (const std::exception &error) {
    throw Fault("DSP bus-cycle preflight pc=" + hex(context.pc, 6) + " omr=" + hex(context.omr, 6) +
                ": " + error.what());
  }
  try {
    result.dma =
        advance_dma(inputs.dma,
                    {.accesses = std::span(result.core_accesses).first(result.core_access_count),
                     .knowledge = DspCoreBusKnowledge::reference_cycle_occupancy});
    if (result.dma.core_stall) { return result; }
    if (inputs.fetch) { result.fetch = state_->cpu.advance_reference_fetch(*inputs.fetch); }
    if (inputs.retirement) {
      result.retirement = state_->cpu.retire_reference_overlap(*inputs.retirement);
    }
    if (inputs.access) { result.progress = state_->cpu.advance_reference_access(*inputs.access); }
    if (inputs.address_update) {
      result.address_update = state_->cpu.publish_reference_address(*inputs.address_update);
    }
    if (inputs.memory) { result.memory = state_->cpu.execute_reference_memory(*inputs.memory); }
    if (inputs.fetch) { result.fetch = state_->cpu.reference_fetch(); }
    return result;
  } catch (const std::exception &error) {
    state_->cpu.fail_reference_cycle();
    throw Fault("DSP bus cycle=" + std::to_string(inputs.dma.cycle) + " DMA/fetch/native pc=" +
                hex(context.pc, 6) + " omr=" + hex(context.omr, 6) + ": " + error.what());
  } catch (...) {
    state_->cpu.fail_reference_cycle();
    throw Fault("Non-standard DSP bus-cycle failure pc=" + hex(context.pc, 6) +
                " omr=" + hex(context.omr, 6));
  }
}

DspCheckpoint DspMachine::checkpoint() const {
  const auto cpu_state = state_->cpu.checkpoint();
  state_->check_boundary();
  return DspCheckpoint(
      std::make_shared<const DspCheckpoint::Storage>(state_->bus, cpu_state, state_->run));
}

void DspMachine::fail_reference_clock() noexcept {
  state_->cpu.fail_reference_cycle();
}

std::shared_ptr<const bool> DspMachine::reference_clock_owner() const {
  return state_->execution_owner;
}

void DspMachine::restore(const DspCheckpoint &checkpoint) {
  if (state_->cpu.executing()) { throw Fault("Cannot restore during active DSP execution"); }
  if (!checkpoint.storage_) { throw Fault("Cannot restore an empty DSP checkpoint"); }
  if (checkpoint.storage_->run.contract != state_->run.contract) {
    throw Fault("DSP checkpoint has an incompatible reference run contract");
  }
  auto replacement = std::make_unique<State>(checkpoint.storage_->run);
  replacement->bus = checkpoint.storage_->bus;
  replacement->cpu.restore_checkpoint(checkpoint.storage_->cpu);
  state_.swap(replacement);
}

}
