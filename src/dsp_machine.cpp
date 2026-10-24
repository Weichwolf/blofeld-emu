// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_machine.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_dax.hpp"
#include "blofeld/dsp_dma.hpp"
#include "blofeld/dsp_interrupts.hpp"
#include "blofeld/dsp_reference_execution.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "dsp_interlocks.hpp"

#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <utility>

namespace blofeld {
namespace {
struct RunState {
  std::optional<DspRunContract> contract;
  std::optional<DspRunClock> clock;
  std::optional<DspInterlocks> interlocks;
};

RunState initial_run(const std::optional<DspRunContract> contract) {
  if (!contract) { return {}; }
  const bool interlocks = contract->timing == DspRunTiming::native_interlock_estimates;
  if (contract->timing != DspRunTiming::native_internal_estimates && !interlocks) {
    throw Fault("DSP run requires an explicit native-cycle-estimate timing contract");
  }
  if (contract->interrupts != DspInterruptTiming::unqualified &&
      contract->interrupts != DspInterruptTiming::native_vector_estimates) {
    throw Fault("Unsupported DSP interrupt timing contract");
  }
  if (interlocks && contract->interrupts != DspInterruptTiming::unqualified) {
    throw Fault("DSP interlock estimates do not qualify interrupt-vector timing");
  }
  if (contract->resets != DspResetTiming::unqualified &&
      contract->resets != DspResetTiming::reference_instruction_boundary) {
    throw Fault("Unsupported DSP reset timing contract");
  }
  return {.contract = contract,
          .clock = DspRunClock(contract->clock),
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
          {.begin = [this](const auto state,
                           const bool interrupt,
                           const unsigned units) { begin_step(state, interrupt, units); },
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
           .vector_estimates = vector_estimates});
    }
  }

  void begin_step(const DspState context, const bool interrupt, const unsigned units) {
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
    const auto stalls = run.interlocks ? run.interlocks->complete(units) : 0;
    run.clock->finish_step(bus, DspClockUnits{.native = units, .interlocks = stalls});
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
        run.contract.value().resets != DspResetTiming::reference_instruction_boundary) {
      throw Fault("Clocked DSP reset requires explicit reference reset timing pc=" +
                  hex(context.pc, 6) + " omr=" + hex(context.omr, 6));
    }
    clock.restart_native_epoch();
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
  const auto &clock = run_clock();
  state_->check_boundary();
  const auto before = state_->cpu.debug_state();
  try {
    const auto run_contract = state_->run.contract;
    if (!run_contract) { throw Fault("Clocked DSP owner lost its timing contract"); }
    const bool vector_estimates =
        run_contract->interrupts == DspInterruptTiming::native_vector_estimates;
    const auto estimate = state_->cpu.next_step_cycles(vector_estimates);
    const auto stalls = state_->preview_stalls(before.cpu, estimate.units);
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
    const auto estimate = state_->cpu.next_step_cycles(true, &admission);
    const auto stalls = state_->preview_stalls(before.cpu, estimate.units);
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

DspReferenceProgress DspMachine::begin_reference_interrupt(const DspReferenceExecution contract) {
  if (clocked()) { throw Fault("Native access suspension requires an unclocked reference owner"); }
  return state_->cpu.begin_reference_interrupt(contract);
}

DspReferencePending DspMachine::latch_reference_program(const DspReferenceTicket &ticket) {
  if (clocked()) { throw Fault("Program latching requires an unclocked reference owner"); }
  return state_->cpu.latch_reference_program(ticket);
}

DspReferenceProgress DspMachine::advance_reference_access(const DspReferenceTicket &ticket) {
  if (clocked()) { throw Fault("Native access suspension requires an unclocked reference owner"); }
  return state_->cpu.advance_reference_access(ticket);
}

std::optional<DspReferencePending> DspMachine::reference_pending() const {
  return state_->cpu.reference_pending();
}

DspCheckpoint DspMachine::checkpoint() const {
  const auto cpu_state = state_->cpu.checkpoint();
  state_->check_boundary();
  return DspCheckpoint(
      std::make_shared<const DspCheckpoint::Storage>(state_->bus, cpu_state, state_->run));
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
