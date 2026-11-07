// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_shi.hpp"
#include "runtime/instrument.hpp"
#include "runtime/mcu_dsp_runtime.hpp"
#include "runtime/nand_peer.hpp"
#include "storage/nand_write_store.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <ostream>
#include <utility>

namespace blofeld::runtime {
DspRunClockInputs dsp_clock_inputs(const Config &config) {
  if (config.dsp_clock.switch_timing != DspClockSwitch::functional_region_boundary) {
    throw Fault("DSP runtime requires functional device event boundaries");
  }
  return config.dsp_clock;
}

void Instrument::prepare_dsp() {
  constexpr unsigned bootstrap_base = 0xff0000;
  if (!execution_.rom.empty()) {
    native_->dsp.bus().install_rom({.start = bootstrap_base, .words = execution_.rom});
  }
  if (config_.nand) {
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
    if (!config_.nand->source_path) { throw Fault("NAND requires a persistent image path"); }
    auto store = std::make_unique<NandWriteStore>(*config_.nand->source_path,
                                                  config_.nand->continuation,
                                                  common_hz(),
                                                  config_.nand->cell_profile);
    store->observe_wp(config_.nand->write_protect_n);
    native_->nand = std::make_unique<NandPeer>(native_->dsp, std::move(store));
    native_->nand->set_telemetry(execution_.trace);
  }
}

void Instrument::connect_runtime() {
  control_.dsp_initial_pending = true;
  native_->runtime = std::make_unique<McuDspRuntime>(
      config_.dsp, native_->mcu, native_->dsp, runtime_events(), peripheral_events(*native_));
}

DspRuntimePeripheralEvents Instrument::peripheral_events(Owners &owners) {
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

std::uint64_t Instrument::common_hz() const {
  return native_->dsp.clock().timebase_hz();
}

std::uint64_t Instrument::dsp_ticks() const {
  return native_->dsp.clock().elapsed_ticks();
}

std::uint64_t Instrument::dsp_steps() const {
  return native_->dsp.cpu().progress().instructions;
}

std::uint64_t Instrument::schedule_ticks() const {
  return native_->runtime ? native_->runtime->end_ticks() : ticks();
}

bool Instrument::dsp_reset_active() const {
  return native_->dsp.clock().hardware_reset_active();
}

bool Instrument::timing_faulted() const {
  return control_.faulted || native_->mcu.timing_faulted() || native_->dsp.clock().faulted() ||
         (native_->runtime && native_->runtime->faulted());
}

McuDspRuntime::CycleInputs Instrument::runtime_inputs() {
  return [this] {
    return DspDmaInputs{.timing = DspDmaTiming::reference_sampled_cycles,
                        .grant = config_.dma_grant};
  };
}

McuDspRuntime::Observer Instrument::runtime_observer() {
  return [this](const DspRuntimeCycle &cycle) {
    if (cycle.phase == DspRuntimePhase::preparing) {
      synchronize_audio_clock(cycle.completion_ticks);
      return;
    }
    // DMA can also write register I/O. Publish actual changes from either core
    // execution or DMA; ordinary native regions require only this revision read.
    const auto revision = native_->dsp.bus().io_revision();
    if (revision != control_.io_revision) {
      control_.io_revision = revision;
      observe_held_dsp_tio();
      synchronize_audio_clock(cycle.completion_ticks);
      if (native_->nand) { native_->nand->after_native(cycle.completion_ticks, cycle.native_pc); }
      control_.return_boundary = cycle.completion_ticks;
      publish_io();
      control_.return_boundary.reset();
    }
    if (execution_.trace != nullptr) {
      *execution_.trace << (cycle.phase == DspRuntimePhase::region ? "DSP-REGION" : "DSP-IO")
                        << " start=" << cycle.start_ticks << " end=" << cycle.completion_ticks
                        << " pc=" << hex(cycle.native_pc.value_or(0), 6)
                        << " retired=" << (cycle.region ? cycle.region->retired : 0) << '\n';
    }
  };
}

void Instrument::observe_held_dsp_tio() const {
  native_->dsp.bus().timers().set_tio_drive(0, true);
  native_->dsp.bus().timers().observe_synchronized_tio(0, true);
}

DspRuntimeEvents Instrument::runtime_events() {
  return {.identity = input_identity_,
          .next_ticks = [this]() -> std::optional<std::uint64_t> {
            return control_.dsp_initial_pending ? std::optional{ticks()} : std::nullopt;
          },
          .observe =
              [this](const std::uint64_t at) {
                if (!control_.dsp_initial_pending || at != ticks()) {
                  throw Fault("Initial DSP inputs require the native attachment boundary");
                }
                synchronize_audio_clock(at);
                observe_held_dsp_tio();
                control_.dsp_initial_pending = false;
              },
          .order = DspPeerOrder::peer_first};
}
}
