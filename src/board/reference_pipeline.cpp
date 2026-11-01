// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "board/nand_peer.hpp"
#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_shi.hpp"
#include "storage/nand.hpp"
#include "storage/nand_image.hpp"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <optional>
#include <ostream>

namespace blofeld::reference {
DspRunClockInputs dsp_clock_inputs(const Config &config) {
  if (config.dsp_clock.switch_timing != DspClockSwitch::reference_cycle_boundary) {
    throw Fault("Regular DSP runtime requires explicit reference cycle boundaries");
  }
  return config.dsp_clock;
}

void Board::prepare_dsp() {
  constexpr unsigned bootstrap_base = 0xff0000;
  if (!execution_.rom.empty()) {
    native_->dsp.bus().install_rom({.start = bootstrap_base, .words = execution_.rom});
  }
  if (config_.nand) {
    const auto ready = [](const DspInput &input) {
      return input.kind == DspInputKind::tio && input.index == 1;
    };
    if (std::ranges::any_of(config_.dsp_initial, ready) ||
        std::ranges::any_of(config_.dsp_events, ready)) {
      throw Fault("NAND Ready owns DSP timer1 input; a second input source is unsupported");
    }
    native_->dsp.bus().set_write_contract(DspWriteContract::functional_decode);
    native_->dsp.bus().shi().set_reset_contract(ShiResetContract::selected_quiescent_reference);
    constexpr unsigned secondary_clock = 2;
    const auto configuration =
        config_.audio_clock && (config_.audio_clock->destinations & secondary_clock) != 0
            ? DspEsaiConfigurationContract::cold_before_bit_clock_reference
            : DspEsaiConfigurationContract::cold_unclocked_reference;
    native_->dsp.bus().set_esai_configuration_contract(DspEsaiInstance::secondary, configuration);
    for (const auto port : {DspPort::c, DspPort::e, DspPort::f, DspPort::d}) {
      native_->dsp.bus().gpio(port).set_contract(DspGpioContract::logical_pins);
    }
    native_->nand = std::make_unique<NandPeer>(
        native_->dsp, Nand(config_.nand->image, config_.nand->continuation));
  }
}

void Board::connect_runtime() {
  control_.dsp_initial_pending = true;
  native_->runtime = std::make_unique<McuDspRuntime>(
      config_.dsp, native_->mcu, native_->dsp, runtime_events(), nand_events(*native_));
}

unsigned Board::runtime_event_budget() const {
  if (config_.maximum_dsp_inputs_per_advance != 0) {
    return config_.maximum_dsp_inputs_per_advance;
  }
  return config_.nand ? 8U : 1U;
}

DspRuntimePeripheralEvents Board::nand_events(Owners &owners) const {
  if (!owners.nand) { return {}; }
  const auto events = owners.nand->events();
  return {.identity = events.identity,
          .next_ticks = events.next_ticks,
          .observe = events.observe,
          .order = DspPeerOrder::peer_first,
          .maximum_events_per_period = runtime_event_budget()};
}

std::optional<NandPeerDetailed> Board::nand_status() const {
  require_idle();
  return native_->nand ? std::optional{native_->nand->detailed()} : std::nullopt;
}

std::optional<NandImage> Board::nand_image() const {
  require_idle();
  if (!native_->nand) { return std::nullopt; }
  auto cold = native_->nand->protocol_copy();
  cold.power_loss();
  return cold.persistent_image();
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
    if (native_->nand) { native_->nand->before_native(native_->dsp.clock().elapsed_ticks()); }
    return DspDmaInputs{.timing = DspDmaTiming::reference_sampled_cycles,
                        .grant = config_.dma_grant};
  };
}

McuDspRuntime::Observer Board::runtime_observer() {
  return [this](const DspRuntimeCycle &cycle) {
    if (native_->nand) {
      native_->nand->after_native(cycle.completion_ticks, cycle.result.native_pc);
    }
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
                        << " native-pc=" << hex(cycle.result.native_pc.value_or(0), 6)
                        << " native-pc-known=" << cycle.result.native_pc.has_value()
                        << " retirement=" << (cycle.result.native && cycle.result.native->retired)
                        << '\n';
    }
  };
}

DspRuntimeEvents Board::runtime_events() {
  auto order = config_.dsp_input_order;
  if (order == DspPeerOrder::unqualified) {
    order = config_.nand ? DspPeerOrder::peer_first : DspPeerOrder::reject_tie;
  }
  return {.identity = input_identity_,
          .next_ticks = [this]() -> std::optional<std::uint64_t> {
            if (control_.dsp_initial_pending) { return ticks(); }
            const auto next = next_dsp_input();
            return next ? std::optional{next->input.ticks} : std::nullopt;
          },
          .observe =
              [this](const std::uint64_t at) {
                if (control_.dsp_initial_pending) {
                  if (at != ticks()) {
                    throw Fault("Initial DSP inputs require the native attachment boundary");
                  }
                  for (const auto &input : config_.dsp_initial) { apply_dsp(input); }
                  control_.dsp_initial_pending = false;
                  const auto next = next_dsp_input();
                  if (!next || next->input.ticks != at) { return; }
                }
                observe_dsp_input(at);
              },
          .order = order,
          .maximum_events_per_period = runtime_event_budget()};
}
}
