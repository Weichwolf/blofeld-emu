// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_io.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <limits>
#include <optional>
#include <string>

namespace blofeld {
namespace {
constexpr unsigned maximum_region_cycles = 1048576;
constexpr unsigned maximum_effects = 512;

class NativeObservation {
public:
  NativeObservation(bool &active, DspPeripheralObserver &slot, const DspPeripheralObserver observer)
      : active_(active), slot_(slot), previous_(slot) {
    active_ = true;
    slot_ = observer;
  }

  ~NativeObservation() {
    slot_ = previous_;
    active_ = false;
  }

  NativeObservation(const NativeObservation &) = delete;
  NativeObservation &operator=(const NativeObservation &) = delete;

private:
  bool &active_;
  DspPeripheralObserver &slot_;
  DspPeripheralObserver previous_;
};

bool same_words(const dsp56300::InstructionWords &a, const dsp56300::InstructionWords &b) {
  return a.address == b.address && a.opcode == b.opcode && a.extension == b.extension;
}
}

unsigned OwnDspTimeline::native_capacity(const std::uint64_t horizon,
                                         const OwnDspBoundary boundary,
                                         const unsigned maximum_cycles) const {
  const auto now = epoch_ticks_ + expected_ticks_;
  if (horizon <= now) { return 0; }
  auto available = horizon - now;
  if (boundary == OwnDspBoundary::before) { --available; }
  if (const auto device = machine_.clock_.next_native_boundary_ticks(machine_.bus_)) {
    available = std::min(available, *device);
  }
  const auto period = machine_.clock_.next_step_ticks(machine_.bus_, 1);
  return static_cast<unsigned>(std::min({available / period,
                                         static_cast<std::uint64_t>(maximum_cycles),
                                         static_cast<std::uint64_t>(maximum_region_cycles)}));
}

void OwnDspTimeline::publish_native_boundary() {
  expected_ticks_ = machine_.clock_.elapsed_ticks();
  expected_dma_ = machine_.bus_.last_dma_cycle();
  if (expected_ticks_ > std::numeric_limits<std::uint64_t>::max() - epoch_ticks_) {
    throw Fault("DSP native region exhausted its common epoch");
  }
  horizon_ = epoch_ticks_ + expected_ticks_;
  boundary_ = OwnDspBoundary::through;
}

void OwnDspTimeline::native_time(const unsigned cycles, DspDmaInputs inputs) {
  native_time(DspClockUnits{.native = cycles}, inputs);
}

void OwnDspTimeline::native_time(const DspClockUnits units, DspDmaInputs inputs) {
  const auto cycles = units.native + units.interlocks;
  const auto previous = machine_.bus_.last_dma_cycle();
  if (previous && cycles > std::numeric_limits<std::uint64_t>::max() - *previous) {
    throw Fault("DSP native region exhausted its DMA cursor");
  }
  inputs.cycle = previous ? *previous + cycles : 0;
  const auto regs = machine_.cpu_.registers();
  machine_.clock_.begin_step(machine_.bus_, {.pc = regs.pc, .omr = regs.omr});
  if (machine_.bus_.native_dma_quiescent(inputs)) {
    machine_.clock_.finish_native_region(machine_.bus_, units, inputs);
  } else {
    if (cycles != 1) { throw Fault("Active DMA requires its actual next native cycle"); }
    static_cast<void>(machine_.clock_.finish_native_cycle(machine_.bus_, inputs, units));
  }
  publish_native_boundary();
}

void OwnDspTimeline::reserve_native(const dsp56300::BoundedRunResult &preview) {
  if (preview.stop == dsp56300::RunStop::unknown_code || !preview.next) {
    auto fetched = machine_.cpu_.begin_fetch(dsp56300::InstructionPublication::pcu_updates);
    for (unsigned index = 0; index < maximum_effects && fetched.pending; ++index) {
      const auto effect = fetched.pending->effect;
      if (effect.kind != dsp56300::EffectKind::fetch_opcode &&
          effect.kind != dsp56300::EffectKind::fetch_extension) {
        break;
      }
      fetched = machine_.interrupts_.advance(fetched.pending->ticket);
      if (fetched.fault.kind != dsp56300::FaultKind::none) { return; }
    }
    throw Fault("DSP genuine fetch requires a qualified continuation cost pc=" +
                hex(preview.pc, 6));
  }
  if (!preview.next || !preview.next_cycles || *preview.next_cycles == 0 ||
      *preview.next_cycles > maximum_region_cycles) {
    throw Fault("DSP native boundary has no qualified captured code/cycle cost pc=" +
                hex(preview.pc, 6));
  }
  native_words_ = preview.next;
  native_cost_ = *preview.next_cycles;
  native_remaining_ = native_cost_;
  const auto instruction =
      native_history_.prepare_owner(machine_.cpu_, *native_words_, native_cost_);
  native_dependencies_ = instruction.dependency_units;
  native_dependency_remaining_ = native_dependencies_;
  // The fixed machine profile has no cacheable external P interval. Core
  // control/vector ownership still requires its genuine captured fetch path.
  native_staged_ = preview.stop == dsp56300::RunStop::control_event ||
                   machine_.cpu_.instruction_fetch().exception_vector;
  if (!native_staged_) { return; }
  auto result = machine_.cpu_.begin_fetch(dsp56300::InstructionPublication::pcu_updates);
  // Vector/cache fetches remain genuine owned reads, acknowledged by the
  // interrupt owner. Their words are latched before peer time can change them.
  for (unsigned index = 0; index < maximum_effects && result.pending; ++index) {
    const auto effect = result.pending->effect;
    if (effect.kind != dsp56300::EffectKind::fetch_opcode &&
        effect.kind != dsp56300::EffectKind::fetch_extension) {
      return;
    }
    result = machine_.interrupts_.advance(result.pending->ticket);
    if (result.fault.kind != dsp56300::FaultKind::none) { return; }
  }
  if (result.fault.kind == dsp56300::FaultKind::none) {
    throw Fault("DSP native fetch exceeded its owned effect budget");
  }
}

void OwnDspTimeline::advance_native_effects() {
  if (!native_words_) { throw Fault("DSP native effects require their captured instruction"); }
  const auto pc = native_words_->address;
  for (unsigned index = 0; index < maximum_effects; ++index) {
    const auto pending = machine_.cpu_.pending_instruction();
    if (!pending) { throw Fault("DSP native reservation lost its real instruction effects"); }
    if (pending->effect.kind == dsp56300::EffectKind::retire) {
      const auto cost = machine_.cpu_.retirement_delta(pending->ticket);
      if (cost.instructions != 1 || cost.estimated_cycles != native_cost_) {
        throw Fault("DSP native reservation differs from its actual retirement cost pc=" +
                    hex(pc, 6));
      }
    }
    const auto result = machine_.interrupts_.advance(pending->ticket);
    if (result.fault.kind != dsp56300::FaultKind::none || result.retired) { return; }
  }
  throw Fault("DSP native reservation exceeded its owned effect budget");
}

void OwnDspTimeline::finish_native_reservation() {
  if (!native_words_ || native_remaining_ != 0 || native_dependency_remaining_ != 0) {
    throw Fault("DSP native completion requires a fully clocked reservation");
  }
  if (!native_staged_) {
    bool completed = false;
    if (native_dependencies_ == 0) {
      const auto preview = machine_.cpu_.run(dsp56300::RunBudget{});
      if (preview.next && same_words(*preview.next, *native_words_)) {
        const auto native = native_history_.run(
            machine_.cpu_, machine_.io_, {.instructions = 1, .cycles = native_cost_});
        const bool retired = native.execution.dispatches == 1;
        const bool invalid =
            native.execution.dispatches > 1 || (retired && native.cycles != native_cost_);
        if (native.execution.fault.kind == dsp56300::FaultKind::none && invalid) {
          throw Fault("DSP native completion changed its single-opcode budget or base cost");
        }
        completed = retired;
      }
    } else {
      completed = native_history_.run_owner(machine_.cpu_,
                                            machine_.io_,
                                            {.words = *native_words_,
                                             .base_units = native_cost_,
                                             .dependency_units = native_dependencies_});
    }
    if (machine_.cpu_.fault().kind != dsp56300::FaultKind::none) { return; }
    if (completed) {
      machine_.bus_.observe_omr(machine_.cpu_.registers().omr);
      native_words_.reset();
      return;
    }
    // A peripheral boundary or code mutation uses the captured original
    // instruction, never newer bytes with the previously reserved cost.
    const auto begun = machine_.cpu_.begin_instruction(
        *native_words_, dsp56300::InstructionPublication::pcu_updates);
    if (begun.fault.kind != dsp56300::FaultKind::none) { return; }
  }
  advance_native_effects();
  if (machine_.cpu_.fault().kind == dsp56300::FaultKind::none) {
    native_history_.complete_owner(machine_.cpu_,
                                   {.words = *native_words_,
                                    .base_units = native_cost_,
                                    .dependency_units = native_dependencies_});
    machine_.bus_.observe_omr(machine_.cpu_.registers().omr);
    native_words_.reset();
    native_staged_ = false;
  }
}

dsp56300::RunResult OwnDspTimeline::native_snapshot(const std::uint64_t before) const {
  const auto progress = machine_.cpu_.progress();
  const auto retired = progress.instructions - before;
  if (retired > std::numeric_limits<std::uint32_t>::max()) {
    throw Fault("DSP native receipt exceeded its instruction counter");
  }
  return {.dispatches = static_cast<std::uint32_t>(retired),
          .state = machine_.cpu_.processing_state(),
          .progress = progress,
          .fault = machine_.cpu_.fault()};
}

dsp56300::RunResult OwnDspTimeline::execute_native_region(const NativeSlice &slice) {
  const auto before = machine_.cpu_.progress().instructions;
  if (!native_words_ && !machine_.interrupts_.pending()) {
    static_cast<void>(machine_.interrupts_.admit());
  }
  if (!native_words_) {
    const bool idle_dma = machine_.bus_.native_dma_quiescent(slice.inputs);
    const auto result = native_history_.run(
        machine_.cpu_,
        machine_.io_,
        {.instructions = idle_dma ? slice.capacity : 0, .cycles = idle_dma ? slice.capacity : 0});
    if (result.cycles != 0) {
      native_time(static_cast<unsigned>(result.cycles), slice.inputs);
      machine_.bus_.observe_omr(machine_.cpu_.registers().omr);
      return result.execution;
    }
    if (result.execution.fault.kind != dsp56300::FaultKind::none) { return result.execution; }
    if (result.execution.state == dsp56300::ProcessingState::waiting) {
      native_time(slice.capacity, slice.inputs);
      native_history_.idle(machine_.cpu_, slice.capacity);
      return native_snapshot(before);
    }
    if (result.execution.state != dsp56300::ProcessingState::running) {
      throw Fault("DSP native oscillator state needs an explicit wake/debug contract pc=" +
                  hex(result.pc, 6));
    }
    reserve_native(result);
  }
  consume_native_reservation(slice);
  return native_snapshot(before);
}

void OwnDspTimeline::consume_native_reservation(const NativeSlice &slice) {
  if (machine_.cpu_.fault().kind != dsp56300::FaultKind::none) { return; }
  auto available = native_capacity(slice.horizon, slice.boundary, slice.maximum_cycles);
  if (!machine_.bus_.native_dma_quiescent(slice.inputs)) { available = std::min(available, 1U); }
  if (native_dependency_remaining_ != 0) {
    const auto consumed = std::min(available, native_dependency_remaining_);
    if (consumed == 0) { throw Fault("DSP dependency reservation lost its clock capacity"); }
    native_time(DspClockUnits{.interlocks = consumed}, slice.inputs);
    native_dependency_remaining_ -= consumed;
    return;
  }
  const auto consumed = std::min(available, native_remaining_);
  if (consumed == 0) { throw Fault("DSP native reservation lost its positive clock capacity"); }
  native_time(consumed, slice.inputs);
  native_remaining_ -= consumed;
  if (native_remaining_ == 0) { finish_native_reservation(); }
}

std::optional<OwnDspRegionResult>
OwnDspTimeline::advance_native_region(const std::uint64_t horizon,
                                      const OwnDspBoundary boundary,
                                      const DspDmaInputs inputs,
                                      const unsigned maximum_cycles,
                                      const DspPeripheralObserver observer) {
  check_owner();
  if (!native_regions() || machine_.clock_.hardware_reset_active() || machine_.advancing_effect_ ||
      horizon < horizon_ ||
      (boundary != OwnDspBoundary::before && boundary != OwnDspBoundary::through) ||
      (horizon == horizon_ && boundary_ == OwnDspBoundary::through &&
       boundary == OwnDspBoundary::before)) {
    throw Fault("DSP native region requires its healthy monotonic finite-time owner");
  }
  try {
    if (maximum_cycles == 0 &&
        (cycle_deadline() < horizon ||
         (cycle_deadline() == horizon && boundary == OwnDspBoundary::through))) {
      throw Fault("DSP native region exhausted its period cycle budget");
    }
    auto capacity = native_capacity(horizon, boundary, maximum_cycles);
    if (!machine_.bus_.native_dma_quiescent(inputs)) { capacity = std::min(capacity, 1U); }
    if (capacity == 0) {
      horizon_ = horizon;
      boundary_ = boundary;
      return std::nullopt;
    }
    const auto start = epoch_ticks_ + expected_ticks_;

    struct Observation {
      OwnDspTimeline &timeline;
      DspPeripheralObserver target;
    } observation{.timeline = *this, .target = observer};

    const DspPeripheralObserver connection{.context = &observation,
                                           .preparing =
                                               [](void *pointer, const unsigned pc) {
                                                 const auto &call =
                                                     *static_cast<Observation *>(pointer);
                                                 if (call.target.preparing != nullptr) {
                                                   call.target.preparing(call.target.context, pc);
                                                 }
                                               },
                                           .completed =
                                               [](void *pointer, const unsigned pc) {
                                                 const auto &call =
                                                     *static_cast<Observation *>(pointer);
                                                 call.timeline.publish_native_boundary();
                                                 if (call.target.completed != nullptr) {
                                                   call.target.completed(call.target.context, pc);
                                                 }
                                               }};
    const NativeObservation observations(
        machine_.advancing_effect_, machine_.io_.peripheral_observer_, connection);
    const auto execution = execute_native_region({.horizon = horizon,
                                                  .boundary = boundary,
                                                  .inputs = inputs,
                                                  .maximum_cycles = maximum_cycles,
                                                  .capacity = capacity});
    faulted_ = execution.fault.kind != dsp56300::FaultKind::none;
    return OwnDspRegionResult{.start_ticks = start,
                              .completion_ticks = epoch_ticks_ + expected_ticks_,
                              .execution = execution};
  } catch (const std::exception &error) {
    faulted_ = true;
    throw Fault("DSP native region pc=" + hex(machine_.cpu_.registers().pc, 6) +
                " tick=" + std::to_string(epoch_ticks_ + expected_ticks_) + ": " + error.what());
  }
}
}
