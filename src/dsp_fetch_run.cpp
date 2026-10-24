// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_fetch_run.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/dsp_program_fetch.hpp"
#include "blofeld/dsp_reference_execution.hpp"
#include "dsp_fetch_run.hpp"

#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
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
  if (inputs.timing != DspFetchTiming::reference_cycle_order || inputs.extal_hz < minimum_extal ||
      inputs.extal_hz > maximum_extal || inputs.timebase_hz < inputs.extal_hz ||
      inputs.timebase_hz > maximum_timebase || inputs.timebase_hz % inputs.extal_hz != 0) {
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
  const auto *access = std::get_if<DspNativeAccess>(&pending->effect);
  if (access == nullptr || access->kind != DspNativeAccessKind::opcode) {
    throw Fault("DSP fetch run requires a genuine pending ordinary opcode" + context(machine));
  }
  auto &state = impl_->state;
  state.inputs = inputs;
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

void DspFetchRun::fail(DspMachine &machine) noexcept {
  impl_->faulted = true;
  machine.fail_reference_clock();
}

DspReferenceProgress DspFetchRun::begin_instruction(DspMachine &machine) {
  check_owner(machine);
  impl_->state.verify(machine);
  if (machine.reference_pending()) {
    throw Fault("DSP fetch run already has a pending instruction");
  }
  const FetchGuard guard(impl_->active);
  try {
    const auto progress =
        machine.begin_reference_instruction(DspReferenceExecution::access_boundaries);
    impl_->state.bind(machine);
    return progress;
  } catch (const std::exception &error) {
    fail(machine);
    throw Fault("DSP fetch backend begin" + context(machine) + ": " + error.what());
  }
}

void DspFetchRun::complete_cycle(DspMachine &machine,
                                 const std::uint64_t period,
                                 const bool switch_was_pending) {
  auto &state = impl_->state;
  machine.bus().advance_reference_cycle_delays(state.next_dma_cycle, switch_was_pending);
  if ((state.cycles & 1U) != 0) {
    machine.bus().timers().advance_clk2(1);
    ++state.clk2_edges;
  }
  ++state.cycles;
  state.ticks += period;
  ++state.next_dma_cycle;
}

DspFetchCycleResult DspFetchRun::advance_cycle(DspMachine &machine,
                                               const DspFetchCycleInputs inputs) {
  check_owner(machine);
  impl_->state.verify(machine);
  if (inputs.admit_native && !machine.reference_pending()) {
    throw Fault("DSP fetch run has no native effect selected for admission");
  }
  const FetchGuard guard(impl_->active);
  auto &state = impl_->state;
  try {
    DspFetchCycleResult result;
    result.start_ticks = state.ticks;
    result.period_ticks = state.period(machine);
    const auto plan = state.plan(machine, inputs, result.native_starved);
    const bool switch_was_pending = machine.bus().switch_delay_ != 0;
    result.bus = machine.advance_reference_bus_cycle(plan);
    const auto fetch = machine.reference_fetch();
    if (!result.bus.dma.core_stall && !fetch.pending_address &&
        fetch.used < dsp_program_fetch_capacity) {
      const auto issued = machine.advance_reference_fetch(DspProgramFetchStage::address);
      result.issued_address = issued.pending_address;
    }
    complete_cycle(machine, result.period_ticks, switch_was_pending);
    state.bind(machine);
    result.completed_cycles = state.cycles;
    result.completion_ticks = state.ticks;
    result.fetch = machine.reference_fetch();
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
}
