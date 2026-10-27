// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_fetch_run.hpp"

#include "common/bus.hpp"
#include "dsp/detail/dsp_fetch_run.hpp"
#include "dsp/detail/dsp_pipeline.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"

#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <variant>

namespace blofeld {
namespace {
class FetchGuard {
public:
  explicit FetchGuard(bool &active) : active_(active) { active_ = true; }

  ~FetchGuard() { active_ = false; }

  FetchGuard(const FetchGuard &) = delete;
  FetchGuard &operator=(const FetchGuard &) = delete;

private:
  bool &active_;
};

void validate_inputs(const DspFetchRunInputs inputs) {
  constexpr unsigned minimum_extal = 500000;
  constexpr unsigned maximum_extal = 100000000;
  constexpr std::uint64_t maximum_timebase = 1000000000000;
  if ((inputs.pipeline != DspFetchPipeline::explicit_effects &&
       inputs.pipeline != DspFetchPipeline::automatic_alu &&
       inputs.pipeline != DspFetchPipeline::automatic_memory) ||
      (inputs.pipeline != DspFetchPipeline::explicit_effects &&
       inputs.execution != DspReferenceExecution::staged_register_boundaries)) {
    throw Fault("Automatic DSP stages require the explicit staged native result contract");
  }
  if (inputs.resets != DspFetchResetTiming::unqualified &&
      inputs.resets != DspFetchResetTiming::reference_cycle_boundary) {
    throw Fault("Unsupported DSP fetch reset timing contract");
  }
  if (inputs.timing != DspFetchTiming::reference_cycle_order || inputs.extal_hz < minimum_extal ||
      inputs.extal_hz > maximum_extal || inputs.timebase_hz < inputs.extal_hz ||
      inputs.timebase_hz > maximum_timebase || inputs.timebase_hz % inputs.extal_hz != 0 ||
      (inputs.execution != DspReferenceExecution::access_boundaries &&
       inputs.execution != DspReferenceExecution::access_and_retirement_boundaries &&
       inputs.execution != DspReferenceExecution::address_and_retirement_boundaries &&
       inputs.execution != DspReferenceExecution::staged_register_boundaries)) {
    throw Fault("DSP fetch run requires an explicit integral EXTAL/reference-cycle clock contract");
  }
}

std::string context(const DspMachine &machine) {
  const auto cpu = machine.cpu().state();
  return " pc=" + hex(cpu.pc, 6) + " omr=" + hex(cpu.omr, 6);
}
}

DspFetchRun::DspFetchRun(const DspFetchRunInputs inputs, DspMachine &machine)
    : impl_(std::make_unique<Impl>()) {
  validate_inputs(inputs);
  if (machine.clocked()) { throw Fault("DSP fetch run cannot replace a completion-clock owner"); }
  const auto pending = machine.reference_pending();
  if (!pending) { throw Fault("DSP fetch run requires a pending opcode" + context(machine)); }
  if (pending->execution != inputs.execution) {
    throw Fault("DSP fetch run requires its matching native execution contract" + context(machine));
  }
  const auto *access = std::get_if<DspNativeAccess>(&pending->effect);
  if (access == nullptr || access->kind != DspNativeAccessKind::opcode) {
    throw Fault("DSP fetch run requires a genuine pending ordinary opcode" + context(machine));
  }
  auto &state = impl_->state;
  state.inputs = inputs;
  if (inputs.pipeline != DspFetchPipeline::explicit_effects) {
    state.pipeline = DspInstructionPipeline{.profile = inputs.pipeline,
                                            .view = {.decode_address = access->target.address}};
  }
  const auto last = machine.bus().last_dma_cycle();
  if (last && *last == std::numeric_limits<std::uint64_t>::max()) {
    throw Fault("DSP fetch run cannot follow an exhausted DMA cycle index");
  }
  state.next_dma_cycle = last ? *last + 1 : 0;
  try {
    static_cast<void>(state.period(machine));
    static_cast<void>(machine.start_reference_fetch(pending->ticket));
    impl_->owner = &machine;
    state.bind(machine);
  } catch (const std::exception &error) {
    throw Fault("DSP fetch initialization" + context(machine) + ": " + error.what());
  }
}

DspFetchRun::~DspFetchRun() = default;

void DspFetchRun::check_owner(const DspMachine &machine) const {
  if (impl_->owner != &machine || impl_->active || impl_->faulted) {
    throw Fault("DSP fetch run requires its healthy inactive bound owner");
  }
}

void DspFetchRun::check_running(const DspMachine &machine) const {
  check_owner(machine);
  impl_->state.verify(machine);
  if (hardware_reset_active()) {
    throw Fault("DSP fetch native cycles are gated during reset assertion/recovery");
  }
}

void DspFetchRun::fail(DspMachine &machine) noexcept {
  impl_->faulted = true;
  machine.fail_reference_clock();
}

DspReferenceProgress DspFetchRun::begin_instruction(DspMachine &machine) {
  check_running(machine);
  if (impl_->state.pipeline) { throw Fault("Automatic DSP stages own native instruction issue"); }
  if (machine.reference_pending()) {
    throw Fault("DSP fetch run already has a pending instruction");
  }
  const FetchGuard guard(impl_->active);
  try {
    const auto progress = machine.begin_reference_instruction(impl_->state.inputs.execution);
    impl_->state.bind(machine);
    return progress;
  } catch (const std::exception &error) {
    fail(machine);
    throw Fault("DSP fetch backend begin" + context(machine) + ": " + error.what());
  }
}

DspReferenceProgress DspFetchRun::overlap_instruction(DspMachine &machine) {
  check_running(machine);
  if (impl_->state.pipeline) { throw Fault("Automatic DSP stages own native instruction issue"); }
  const auto pending = machine.reference_pending();
  if (!pending) { throw Fault("DSP fetch overlap requires an actual held result"); }
  machine.validate_reference_overlap(pending->ticket);
  const FetchGuard guard(impl_->active);
  try {
    const auto progress = machine.overlap_reference_instruction(pending->ticket);
    impl_->state.bind(machine);
    return progress;
  } catch (const std::exception &error) {
    fail(machine);
    throw Fault("DSP fetch overlap" + context(machine) + ": " + error.what());
  }
}

void DspFetchRun::complete_cycle(DspMachine &machine,
                                 const std::uint64_t period,
                                 const bool switch_was_pending) {
  auto &state = impl_->state;
  machine.bus().advance_reference_cycle_delays(state.next_dma_cycle, switch_was_pending);
  if (((state.cycles - state.epoch_cycles) & 1U) != 0) {
    machine.bus().timers().advance_clk2(1);
    ++state.clk2_edges;
  }
  ++state.cycles;
  state.ticks += period;
  ++state.next_dma_cycle;
}

DspFetchCycleResult DspFetchRun::advance_cycle(DspMachine &machine,
                                               const DspFetchCycleInputs inputs) {
  check_running(machine);
  if (impl_->state.pipeline && (inputs.admit_native || inputs.retire_previous)) {
    throw Fault("Automatic DSP stages reject caller-selected native phases");
  }
  if (inputs.admit_native && !machine.reference_pending()) {
    throw Fault("DSP fetch run has no native effect selected for admission");
  }
  const FetchGuard guard(impl_->active);
  auto &state = impl_->state;
  try {
    DspFetchCycleResult result;
    result.start_ticks = state.ticks;
    result.period_ticks = state.period(machine);
    if (state.pipeline) {
      if (machine.bus().dma_.active()) {
        throw Fault("Unqualified automatic ALU/DMA arbitration at X:0xfffff4" + context(machine));
      }
      state.pipeline->prepare(machine);
    }
    const auto plan = state.plan(machine, inputs, result.native_starved);
    const bool switch_was_pending = machine.bus().switch_delay_ != 0;
    result.bus =
        state.pipeline
            ? machine.advance_reference_bus_cycle_impl(
                  plan, plan.peripheral_capture ? state.pipeline->peripheral_occurrence() : 0)
            : machine.advance_reference_bus_cycle(plan);
    if (state.pipeline) { state.pipeline->complete(machine, result.bus); }
    const auto fetch = machine.reference_fetch();
    const bool source_stall = state.pipeline && (state.pipeline->view.cycle.memory_source_stall ||
                                                 state.pipeline->view.cycle.agu_dependencies != 0 ||
                                                 state.pipeline->held_program_memory);
    const bool branch_stall = state.pipeline && state.pipeline->hold_fetch();
    if (!result.bus.dma.core_stall && !source_stall && !branch_stall && !fetch.pending_address &&
        fetch.used < dsp_program_fetch_capacity) {
      const auto issued = machine.advance_reference_fetch(DspProgramFetchStage::address);
      result.issued_address = issued.pending_address;
    }
    if (state.pipeline) { state.pipeline->finish_cycle(); }
    complete_cycle(machine, result.period_ticks, switch_was_pending);
    state.bind(machine);
    result.completed_cycles = state.cycles;
    result.completion_ticks = state.ticks;
    result.fetch = machine.reference_fetch();
    if (state.pipeline) { result.pipeline = state.pipeline->view.cycle; }
    return result;
  } catch (const std::exception &error) {
    fail(machine);
    throw Fault("DSP fetch completed-cycles=" + std::to_string(state.cycles) +
                " DMA-index=" + std::to_string(state.next_dma_cycle) +
                " ticks=" + std::to_string(state.ticks) + context(machine) + ": " + error.what());
  } catch (...) {
    fail(machine);
    throw Fault("Non-standard DSP fetch-cycle failure" + context(machine));
  }
}

std::uint64_t DspFetchRun::next_cycle_ticks(const DspMachine &machine) const {
  check_running(machine);
  return impl_->state.period(machine);
}

std::uint64_t DspFetchRun::elapsed_ticks() const {
  return impl_->state.ticks;
}

std::uint64_t DspFetchRun::completed_cycles() const {
  return impl_->state.cycles;
}

std::uint64_t DspFetchRun::timer_clk2_edges() const {
  return impl_->state.clk2_edges;
}

bool DspFetchRun::faulted() const {
  return impl_->faulted;
}

std::optional<DspPipelineView> DspFetchRun::pipeline() const {
  const auto &pipeline = impl_->state.pipeline;
  if (!pipeline) { return std::nullopt; }
  return pipeline->view;
}
}
