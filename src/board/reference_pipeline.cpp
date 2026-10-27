// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_fetch_schedule.hpp"
#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_fetch_schedule.hpp"
#include "dsp/dsp_pipeline.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <ostream>

namespace blofeld::reference {
void Board::prepare_dsp() {
  constexpr unsigned bootstrap_base = 0xff0000;
  if (!execution_.rom.empty()) {
    native_->dsp.bus().install_rom({.start = bootstrap_base, .words = execution_.rom});
  }
  if (!config_.pipeline) {
    native_->dsp.bus().pll().observe_pinit(config_.pinit);
    native_->dsp.cpu().reset_spi_boot();
  }
}

void Board::connect_pipeline() {
  if (config_.pipeline) {
    const auto &timing = *config_.pipeline;
    if (!timing.schedule.initial_reset || timing.schedule.epoch_ticks != 0 ||
        timing.schedule.dsp.pipeline != DspFetchPipeline::automatic_memory) {
      throw Fault("Reference pipeline requires time-zero held reset and automatic memory");
    }
    control_.dsp_initial_pending = true;
    native_->fetch = std::make_unique<McuDspFetchSchedule>(
        timing.schedule, native_->mcu, native_->dsp, fetch_events());
  } else {
    for (const auto &entry : config_.dsp_initial) { apply_dsp(entry); }
  }
}

std::uint64_t Board::common_hz() const {
  return config_.pipeline ? config_.pipeline->schedule.dsp.timebase_hz
                          : native_->dsp.run_clock().timebase_hz();
}

std::uint64_t Board::dsp_ticks() const {
  return native_->fetch ? native_->fetch->run().elapsed_ticks()
                        : native_->dsp.run_clock().elapsed_ticks();
}

std::uint64_t Board::dsp_steps() const {
  if (native_->fetch) { return control_.dsp_retirements; }
  return native_->schedule ? native_->schedule->completed_steps() : 0;
}

std::uint64_t Board::schedule_ticks() const {
  if (native_->fetch) { return native_->fetch->end_ticks(); }
  return native_->schedule ? native_->schedule->end_ticks() : ticks();
}

std::uint64_t Board::schedule_ties() const {
  if (native_->fetch) { return native_->fetch->ties(); }
  return native_->schedule ? native_->schedule->ties() : 0;
}

bool Board::dsp_reset_active() const {
  return native_->fetch ? native_->fetch->run().hardware_reset_active() : !native_->schedule;
}

bool Board::timing_faulted() const {
  return control_.faulted || native_->mcu.timing_faulted() || native_->dsp.timing_faulted() ||
         (native_->schedule && native_->schedule->faulted()) ||
         (native_->fetch && native_->fetch->faulted());
}

McuDspFetchSchedule::CycleInputs Board::fetch_inputs() const {
  if (!config_.pipeline) { throw Fault("Missing reference pipeline contract"); }
  return [this] {
    // External IRQA..IRQD DMA requests stay unknown. The bus derives supported
    // internal DAX/ESAI requests from actual peripheral state.
    return DspFetchCycleInputs{.grant = config_.pipeline->dma_grant};
  };
}

McuDspFetchSchedule::Observer Board::fetch_observer() {
  return [this](const DspFetchCycleResult &cycle) {
    if (cycle.pipeline && cycle.pipeline->retirement_published) { ++control_.dsp_retirements; }
    control_.return_boundary = cycle.completion_ticks;
    update_wiring();
    control_.return_boundary.reset();
    observe_pcm(cycle.completion_ticks, "pipeline");
    if (execution_.trace != nullptr) {
      const auto &pipeline = cycle.pipeline;
      *execution_.trace << "DSP-PIPELINE start=" << cycle.start_ticks
                        << " end=" << cycle.completion_ticks << " cycle=" << cycle.completed_cycles
                        << " f1=" << cycle.issued_address.value_or(0)
                        << " f1-known=" << cycle.issued_address.has_value()
                        << " decode=" << (pipeline && pipeline->decode_accepted)
                        << " e2=" << (pipeline && pipeline->retirement_published) << '\n';
    }
  };
}

DspFetchExternalEvents Board::fetch_events() {
  return {.identity = input_identity_,
          .next_ticks = [this]() -> std::optional<std::uint64_t> {
            if (control_.dsp_initial_pending) { return 0; }
            const auto next = next_dsp_input();
            return next ? std::optional{next->input.ticks} : std::nullopt;
          },
          .observe =
              [this](const std::uint64_t at) {
                if (control_.dsp_initial_pending) {
                  if (at != 0) { throw Fault("Initial pipeline inputs require time zero"); }
                  for (const auto &input : config_.dsp_initial) { apply_dsp(input); }
                  control_.dsp_initial_pending = false;
                  // Initial observations precede a timed event at zero, as on the
                  // legacy Board. A single composite offer avoids duplicate timestamps.
                  const auto next = next_dsp_input();
                  if (!next || next->input.ticks != at) { return; }
                }
                observe_dsp_input(at);
              },
          .order = config_.dsp_input_order,
          .maximum_events = config_.maximum_dsp_inputs_per_advance};
}
}
