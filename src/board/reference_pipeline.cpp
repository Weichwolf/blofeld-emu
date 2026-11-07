// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "board/nand_bus_decode.hpp"
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
#include "storage/nand_write_store.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <ostream>
#include <utility>

namespace blofeld::reference {
DspRunClockInputs dsp_clock_inputs(const Config &config) {
  if (config.dsp_clock.switch_timing != DspClockSwitch::functional_region_boundary) {
    throw Fault("DSP runtime requires functional device event boundaries");
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
        std::ranges::any_of(config_.dsp_events, ready) ||
        (config_.midi_dsp && config_.midi_dsp->index == 1)) {
      throw Fault("NAND Ready owns DSP timer1 input; a second input source is unsupported");
    }
    native_->dsp.bus().set_write_contract(DspWriteContract::functional_decode);
    // Nominal virtual decoder word, not a measured reserved-bus value.
    native_->dsp.bus().set_unselected_io_read_word(0);
    // Separate vacant-data decoder hypothesis; selected unknown cells still fail.
    native_->dsp.bus().set_unselected_data_read_word(0);
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
    if (config_.nand->backing == NandBacking::live_store) {
      if (!config_.nand->source_path || !config_.nand->image.pages.empty() ||
          !config_.nand->image.blocks.empty()) {
        throw Fault("Live NAND requires an existing source file without a second cold image");
      }
      auto store = std::make_unique<NandWriteStore>(*config_.nand->source_path,
                                                    config_.nand->continuation,
                                                    common_hz(),
                                                    config_.nand->cell_profile);
      store->observe_wp(config_.nand->write_protect_n);
      native_->nand = std::make_unique<NandPeer>(native_->dsp,
                                                 std::move(store),
                                                 NandBusContract::functional_transactions,
                                                 config_.nand->timing);
    } else if (config_.nand->backing == NandBacking::cold_image) {
      Nand device(config_.nand->image, config_.nand->continuation);
      device.observe_wp(config_.nand->write_protect_n);
      native_->nand = std::make_unique<NandPeer>(native_->dsp,
                                                 std::move(device),
                                                 config_.nand->cell_profile,
                                                 NandBusContract::functional_transactions,
                                                 config_.nand->timing);
    } else {
      throw Fault("Invalid NAND backing contract");
    }
    native_->nand->set_telemetry(execution_.trace);
  }
}

void Board::connect_runtime() {
  control_.dsp_initial_pending = true;
  native_->runtime = std::make_unique<McuDspRuntime>(
      config_.dsp, native_->mcu, native_->dsp, runtime_events(), peripheral_events(*native_));
}

DspRuntimePeripheralEvents Board::peripheral_events(Owners &owners) {
  if (!owners.nand && !transmitter_clock_events()) { return {}; }
  const auto events = owners.nand ? owners.nand->events() : NandPeerEvents{};
  if (transmitter_clock_events()) {
    return {.identity = clock_identity_,
            .next_ticks = [this, events]() -> std::optional<std::uint64_t> {
              const auto nand = events.next_ticks ? events.next_ticks() : std::nullopt;
              const auto audio = next_audio_clock_event();
              return audio && (!nand || *audio < *nand) ? audio : nand;
            },
            .observe =
                [this, events](const std::uint64_t at) {
                  synchronize_audio_clock(at);
                  bool observed = false;
                  // Ready and audio are disjoint devices. A simultaneous batch
                  // retains explicit NAND-ready, then transmitter ordering.
                  if (events.next_ticks && events.next_ticks() == at) {
                    events.observe(at);
                    observed = true;
                  }
                  if (next_audio_clock_event() == at) {
                    observe_audio_clock(at);
                    observed = true;
                  }
                  if (!observed) { throw Fault("No device owns the offered peripheral time"); }
                },
            .order = DspPeerOrder::peer_first};
  }
  return {.identity = events.identity,
          .next_ticks = events.next_ticks,
          .observe = events.observe,
          .order = DspPeerOrder::peer_first};
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
  return native_->dsp.cpu().state().progress.instructions;
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

McuDspRuntime::CycleInputs Board::runtime_inputs() {
  return [this] {
    if (transmitter_clock_events()) {
      const auto elapsed = dsp_ticks();
      if (elapsed > std::numeric_limits<std::uint64_t>::max() - config_.dsp.epoch_ticks) {
        throw Fault("Periodic audio synchronization exhausted common time");
      }
      synchronize_audio_clock(config_.dsp.epoch_ticks + elapsed);
    }
    observe_held_dsp_tio();
    if (native_->nand) { native_->nand->before_native(native_->dsp.clock().elapsed_ticks()); }
    return DspDmaInputs{.timing = DspDmaTiming::reference_sampled_cycles,
                        .grant = config_.dma_grant};
  };
}

McuDspRuntime::Observer Board::runtime_observer() {
  return [this](const DspRuntimeCycle &cycle) {
    synchronize_audio_clock(cycle.completion_ticks);
    if (native_->nand) { native_->nand->after_native(cycle.completion_ticks, cycle.native_pc); }
    if (cycle.region) { control_.dsp_retirements += cycle.region->retired; }
    control_.return_boundary = cycle.completion_ticks;
    update_wiring();
    control_.return_boundary.reset();
    observe_pcm(cycle.completion_ticks, "native");
    if (execution_.trace != nullptr) {
      *execution_.trace << (cycle.phase == DspRuntimePhase::region ? "DSP-REGION" : "DSP-IO")
                        << " start=" << cycle.start_ticks << " end=" << cycle.completion_ticks
                        << " cycle=" << cycle.cycle << " cycle-known=" << cycle.cycle_known
                        << " pc=" << hex(cycle.native_pc.value_or(0), 6)
                        << " retired=" << (cycle.region ? cycle.region->retired : 0)
                        << " pc-known=" << cycle.native_pc.has_value() << '\n';
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
                synchronize_audio_clock(at);
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
          .order = order};
}
}
