// SPDX-License-Identifier: GPL-3.0-only
#include "board/reference_board.hpp"
#include "board/reference_context.hpp"
#include "board/reference_square_clock.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dax.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_scheduled_run.hpp"

#include <cstdint>
#include <ostream>
#include <string>

namespace blofeld::reference {
void Board::prepare_audio_clock() {
  if (!config_.audio_clock) { return; }
  const auto &route = *config_.audio_clock;
  constexpr unsigned destinations = 7;
  if (route.destinations == 0 || (route.destinations & ~destinations) != 0) {
    throw Fault("Invalid reference audio clock destinations");
  }
  if (config_.dsp_input_order == DspExternalOrder::unqualified ||
      config_.maximum_dsp_inputs_per_advance == 0) {
    throw Fault("Reference audio clock requires explicit DSP input ordering/budget");
  }
  if (!config_.pipeline && route.epoch_ticks < config_.release_ticks) {
    throw Fault("Reference audio clock precedes the legacy DSP release");
  }
  for (const auto &clock : config_.dsp_clocks) {
    if (clock.index > 1 || (route.destinations & (1U << clock.index)) != 0) {
      throw Fault("Conflicting reference audio/ESAI periodic clock routes");
    }
  }
  for (const auto *inputs : {&config_.dsp_initial, &config_.dsp_events}) {
    for (const auto &input : *inputs) {
      if (input.kind == DspInputKind::esai &&
          (input.index > 1 || (route.destinations & (1U << input.index)) != 0)) {
        throw Fault("Conflicting reference audio clock and explicit ESAI input snapshot");
      }
    }
  }
  control_.audio_clock.emplace(SquareClockConfig{.timebase_hz = common_hz(),
                                                 .frequency_hz = route.frequency_hz,
                                                 .epoch_ticks = route.epoch_ticks,
                                                 .initial_level = route.initial_level});
}

std::uint64_t Board::audio_core_hz() const {
  // The clock is supplied even during reset, when the core rate is unknown.
  // Inactive DAX observations retain that explicit unknown without emitting data.
  if (dsp_reset_active()) { return 0; }
  const auto extal =
      config_.pipeline ? config_.pipeline->schedule.dsp.extal_hz : config_.dsp.clock.extal_hz;
  const auto ratio = native_->dsp.bus().pll().target_ratio();
  const auto numerator = std::uint64_t{extal} * ratio.numerator;
  if (ratio.denominator == 0 || numerator == 0 || numerator % ratio.denominator != 0) {
    throw Fault("Reference DAX rate is not an exact current PLL/EXTAL frequency at X:0xfffffd");
  }
  return numerator / ratio.denominator;
}

void Board::observe_audio_clock(const std::uint64_t at) {
  if (!config_.audio_clock || !control_.audio_clock) {
    throw Fault("Missing reference audio clock source/route");
  }
  const auto &route = *config_.audio_clock;
  auto &source = *control_.audio_clock;
  const bool baseline = !control_.audio_clock_started;
  const auto event = baseline ? source.current() : source.next();
  if (event.ticks != at) { throw Fault("Reference audio clock event changed its timestamp"); }
  constexpr unsigned dax_aci = 4;
  std::uint64_t core_hz = 0;
  try {
    for (unsigned index = 0; index < 2; ++index) {
      if ((route.destinations & (1U << index)) == 0) { continue; }
      const auto instance = index == 0 ? DspEsaiInstance::primary : DspEsaiInstance::secondary;
      constexpr unsigned hckt_pad = 0x20;
      native_->dsp.bus()
          .gpio(index == 0 ? DspPort::c : DspPort::e)
          .update_drive(hckt_pad, {.mask = hckt_pad, .levels = event.level ? hckt_pad : 0U});
      native_->dsp.bus().observe_esai_transmit(
          instance,
          {.high_clock = event.level, .timing = DspEsaiClockTiming::reference_reset_edges});
    }
    if ((route.destinations & dax_aci) != 0) {
      constexpr unsigned aci_pad = 1;
      native_->dsp.bus()
          .gpio(DspPort::d)
          .update_drive(aci_pad, {.mask = aci_pad, .levels = event.level ? aci_pad : 0U});
      core_hz = audio_core_hz();
      native_->dsp.bus().observe_dax({.aci = event.level,
                                      .rates = {.core_hz = core_hz, .aci_hz = route.frequency_hz},
                                      .timing = DspDaxTiming::reference_rising_edges});
    }
  } catch (const Fault &error) {
    throw Fault("Reference audio clock input at=" + std::to_string(at) + " destinations=" +
                hex(route.destinations, 2) + " mcu-pc=" + mcu_pc_context(native_->mcu) +
                " dsp-pc=" + hex(native_->dsp.cpu().state().pc, 6) + ": " + error.what());
  }
  // A partial fanout fault quarantines the Board; only a joint restore may retry.
  if (!baseline) { source.consume(at); }
  control_.audio_clock_started = true;
  trace_audio_clock(route, event, baseline, core_hz);
  observe_pcm(at, "audio-clock");
}

void Board::trace_audio_clock(const AudioClockRoute &route,
                              const SquareClockLevel event,
                              const bool baseline,
                              const std::uint64_t core_hz) {
  if (execution_.trace == nullptr) { return; }
  *execution_.trace << "DSP-AUDIO-CLOCK requested=" << event.ticks << " actual=" << event.ticks
                    << " baseline=" << baseline << " level=" << event.level
                    << " destinations=" << hex(route.destinations, 2) << '\n';
  for (unsigned index = 0; index < 2; ++index) {
    if ((route.destinations & (1U << index)) == 0) { continue; }
    trace_dsp_input(
        {.ticks = event.ticks,
         .kind = DspInputKind::esai,
         .index = index,
         .esai = {.high_clock = event.level, .timing = DspEsaiClockTiming::reference_reset_edges}},
        event.ticks);
  }
  constexpr unsigned dax_aci = 4;
  if ((route.destinations & dax_aci) != 0) {
    const auto output = native_->dsp.bus().dax_output();
    *execution_.trace << "DSP-DAX requested=" << event.ticks << " actual=" << event.ticks
                      << " aci=" << event.level << " core-hz=" << core_hz
                      << " driven=" << output.driven << " level=" << output.level << '\n';
  }
}
}
