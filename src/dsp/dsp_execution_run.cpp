// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_execution.hpp"
#include "dsp/dsp_io.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp56300/core.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <optional>

namespace blofeld {
void DspExecution::native_time(unsigned cycles) {
  while (cycles != 0) {
    const auto previous = machine_.bus_.last_dma_cycle();
    auto count = cycles;
    if (const auto boundary = machine_.clock_.next_native_boundary_ticks(machine_.bus_)) {
      const auto period = machine_.clock_.next_step_ticks(machine_.bus_, 1);
      count = static_cast<unsigned>(std::min<std::uint64_t>(count, *boundary / period));
    }
    const bool quiet = previous && machine_.bus_.native_dma_quiescent(inputs_);
    if (!quiet) { count = 1; }
    if (count == 0 || (previous && count > std::numeric_limits<std::uint64_t>::max() - *previous)) {
      throw Fault("DSP device span cannot advance its DMA cursor");
    }
    inputs_.cycle = previous ? *previous + count : 0;
    const auto regs = machine_.cpu_.registers();
    machine_.clock_.begin_step(machine_.bus_, {.pc = regs.pc, .omr = regs.omr});
    if (quiet) {
      machine_.clock_.finish_native_region(machine_.bus_, count, inputs_);
    } else {
      static_cast<void>(machine_.clock_.finish_native_cycle(machine_.bus_, inputs_));
    }
    cycles -= count;
  }
}

void DspExecution::synchronize_prefix(const unsigned pc) {
  const auto cycles = machine_.cpu_.progress().estimated_cycles;
  if (cycles < accounted_base_ || cycles - accounted_base_ > available_cycles_) {
    throw Fault("DSP I/O prefix crossed its device deadline pc=" + hex(pc, 6));
  }
  const auto elapsed = static_cast<unsigned>(cycles - accounted_base_);
  native_time(elapsed);
  accounted_base_ += elapsed;
  available_cycles_ -= elapsed;
  if (observer_.preparing != nullptr) { observer_.preparing(observer_.context, pc); }
}

void DspExecution::peripheral_completed(const unsigned pc) {
  if (observer_.completed != nullptr) { observer_.completed(observer_.context, pc); }
  machine_.cpu_.request_yield();
}

std::optional<DspExecutionResult>
DspExecution::advance_native_region(const std::uint64_t horizon,
                                    const DspBoundary boundary,
                                    const DspDmaInputs inputs,
                                    const DspPeripheralObserver observer) {
  validate_boundary();
  auto count = capacity(horizon, boundary);
  if (count == 0) { return std::nullopt; }
  inputs_ = inputs;
  observer_ = observer;
  const auto start = now();
  try {
    if (remaining_cycles_ != 0) {
      count = static_cast<unsigned>(std::min<std::uint64_t>(count, remaining_cycles_));
      native_time(count);
      accounted_base_ += count;
      remaining_cycles_ -= count;
      if (remaining_cycles_ == 0) { machine_.bus_.observe_omr(machine_.cpu_.registers().omr); }
      return DspExecutionResult{
          .start_ticks = start, .completion_ticks = now(), .execution = std::nullopt};
    }
    static_cast<void>(machine_.interrupts_.admit());
    available_cycles_ = count;
    const DspPeripheralObserver effects{
        .context = this,
        .preparing =
            [](void *self, const unsigned pc) {
              static_cast<DspExecution *>(self)->synchronize_prefix(pc);
            },
        .completed =
            [](void *self, const unsigned pc) {
              static_cast<DspExecution *>(self)->peripheral_completed(pc);
            }};
    if (count > std::numeric_limits<std::uint64_t>::max() - accounted_base_) {
      throw Fault("DSP instruction timeline exhausted its base-cycle count");
    }
    const auto execution = machine_.execute_until(accounted_base_ + count, effects);
    if (execution.fault.kind == dsp56300::FaultKind::none) {
      const auto elapsed = execution.progress.estimated_cycles - accounted_base_;
      const auto paid = static_cast<unsigned>(std::min<std::uint64_t>(available_cycles_, elapsed));
      native_time(paid);
      accounted_base_ += paid;
      remaining_cycles_ = elapsed - paid;
      if (remaining_cycles_ == 0) { machine_.bus_.observe_omr(machine_.cpu_.registers().omr); }
      if (elapsed == 0 && execution.state != dsp56300::ExecutionState::running) {
        native_time(available_cycles_);
      }
    }
    return DspExecutionResult{
        .start_ticks = start, .completion_ticks = now(), .execution = execution};
  } catch (...) {
    faulted_ = true;
    throw;
  }
}
}
