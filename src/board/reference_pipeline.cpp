// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_fetch_schedule.hpp"
#include "board/mcu_dsp_runtime.hpp"
#include "board/mcu_dsp_schedule.hpp"
#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <optional>
#include <ostream>

namespace blofeld::reference {
namespace {
McuWriteOrder write_order(const McuDspTieOrder order) {
  switch (order) {
    case McuDspTieOrder::dsp_before_mcu: return McuWriteOrder::before_mcu;
    case McuDspTieOrder::mcu_before_dsp: return McuWriteOrder::after_mcu;
    case McuDspTieOrder::unqualified: return McuWriteOrder::unqualified;
    default: throw Fault("Invalid MCU/DSP runtime write order");
  }
}

DspPeerOrder event_order(const DspExternalOrder order) {
  switch (order) {
    case DspExternalOrder::dsp_before_external: return DspPeerOrder::dsp_first;
    case DspExternalOrder::external_before_dsp: return DspPeerOrder::peer_first;
    case DspExternalOrder::unqualified:
    case DspExternalOrder::reject_tie: return DspPeerOrder::reject_tie;
    default: throw Fault("Invalid DSP external event order");
  }
}
}

DspRunClockInputs dsp_clock_inputs(const Config &config) {
  if (config.pipeline) {
    const auto &clock = config.pipeline->schedule.dsp;
    return {.extal_hz = clock.extal_hz,
            .timebase_hz = clock.timebase_hz,
            .switch_timing = DspClockSwitch::reference_cycle_boundary};
  }
  auto result = config.dsp.clock;
  if (result.switch_timing != DspClockSwitch::reference_instruction_boundary &&
      result.switch_timing != DspClockSwitch::reference_cycle_boundary) {
    throw Fault("Regular DSP runtime requires explicit reference clock boundaries");
  }
  result.switch_timing = DspClockSwitch::reference_cycle_boundary;
  return result;
}

void Board::prepare_dsp() {
  constexpr unsigned bootstrap_base = 0xff0000;
  if (!execution_.rom.empty()) {
    native_->dsp.bus().install_rom({.start = bootstrap_base, .words = execution_.rom});
  }
}

void Board::connect_pipeline() {
  McuDspRuntimeInputs inputs;
  inputs.timing = OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts;
  inputs.initial_reset = OwnDspResetInputs{
      .mode_known = 15, .mode_levels = 5, .pinit = config_.pinit, .recovery_cycles = 2};
  inputs.maximum_cycles_per_period = config_.schedule.maximum_dsp_steps_per_cycle;
  inputs.writes = write_order(config_.schedule.shared_write_order);
  inputs.wait_dsp_first = config_.schedule.wait_order == McuDspWaitOrder::dsp_before_mcu;
  inputs.reset_dsp_first = true;
  if (config_.pipeline) {
    const auto &timing = *config_.pipeline;
    if (!timing.schedule.initial_reset || timing.schedule.epoch_ticks != 0 ||
        timing.schedule.dsp.pipeline != DspFetchPipeline::automatic_memory) {
      throw Fault("Device pipeline requires time-zero held reset and automatic memory");
    }
    const auto &reset = *timing.schedule.initial_reset;
    inputs.initial_reset = OwnDspResetInputs{.mode_known = reset.mode_known,
                                             .mode_levels = reset.mode_levels,
                                             .pinit = reset.pinit,
                                             .recovery_cycles = reset.recovery_cycles};
    inputs.maximum_cycles_per_period = timing.schedule.maximum_dsp_cycles_per_period;
    inputs.writes = write_order(timing.schedule.shared_write_order);
    inputs.wait_dsp_first = timing.schedule.wait_order == McuDspWaitOrder::dsp_before_mcu;
    inputs.reset_dsp_first = timing.schedule.reset_order == McuDspResetOrder::dsp_before_mcu;
    switch (timing.schedule.external_write_order) {
      case McuExternalWriteOrder::external_before_mcu:
        inputs.external_writes = McuWriteOrder::before_mcu;
        break;
      case McuExternalWriteOrder::mcu_before_external:
        inputs.external_writes = McuWriteOrder::after_mcu;
        break;
      case McuExternalWriteOrder::unqualified: break;
      default: throw Fault("Invalid runtime external write order");
    }
  }
  control_.dsp_initial_pending = true;
  native_->runtime =
      std::make_unique<McuDspRuntime>(inputs, native_->mcu, native_->dsp, runtime_events());
}

std::uint64_t Board::common_hz() const {
  return native_->dsp.clock().timebase_hz();
}

std::uint64_t Board::dsp_ticks() const {
  return native_->dsp.clock().elapsed_ticks();
}

std::uint64_t Board::dsp_steps() const {
  return native_->dsp.cpu().progress().instructions;
}

std::uint64_t Board::schedule_ticks() const {
  return native_->runtime ? native_->runtime->end_ticks() : ticks();
}

std::uint64_t Board::schedule_ties() const {
  return native_->runtime ? native_->runtime->ties() : 0;
}

bool Board::dsp_reset_active() const {
  return native_->dsp.clock().hardware_reset_active();
}

bool Board::timing_faulted() const {
  return control_.faulted || native_->mcu.timing_faulted() || native_->dsp.clock().faulted() ||
         (native_->runtime && native_->runtime->faulted());
}

McuDspRuntime::CycleInputs Board::runtime_inputs() const {
  return [this] {
    return DspDmaInputs{.timing = DspDmaTiming::reference_sampled_cycles,
                        .grant = config_.pipeline ? config_.pipeline->dma_grant
                                                  : DspDmaGrant::available};
  };
}

McuDspRuntime::Observer Board::runtime_observer() {
  return [this](const DspRuntimeCycle &cycle) {
    if (cycle.result.native && cycle.result.native->retired) { ++control_.dsp_retirements; }
    control_.return_boundary = cycle.completion_ticks;
    update_wiring();
    control_.return_boundary.reset();
    observe_pcm(cycle.completion_ticks, "pipeline");
    if (execution_.trace != nullptr) {
      *execution_.trace << "DSP-PIPELINE start=" << cycle.start_ticks
                        << " end=" << cycle.completion_ticks << " cycle=" << cycle.cycle
                        << " f1=" << cycle.result.issued_address.value_or(0)
                        << " f1-known=" << cycle.result.issued_address.has_value()
                        << " retirement=" << (cycle.result.native && cycle.result.native->retired)
                        << '\n';
    }
  };
}

DspRuntimeEvents Board::runtime_events() {
  return {.identity = input_identity_,
          .next_ticks = [this]() -> std::optional<std::uint64_t> {
            if (control_.dsp_initial_pending) { return 0; }
            const auto next = next_dsp_input();
            return next ? std::optional{next->input.ticks} : std::nullopt;
          },
          .observe =
              [this](const std::uint64_t at) {
                if (control_.dsp_initial_pending) {
                  if (at != 0) { throw Fault("Initial DSP inputs require time zero"); }
                  for (const auto &input : config_.dsp_initial) { apply_dsp(input); }
                  control_.dsp_initial_pending = false;
                  const auto next = next_dsp_input();
                  if (!next || next->input.ticks != at) { return; }
                }
                observe_dsp_input(at);
              },
          .order = event_order(config_.dsp_input_order),
          .maximum_events_per_period = std::max(1U, config_.maximum_dsp_inputs_per_advance)};
}
}
