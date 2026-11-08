// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dax.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_pll.hpp"
#include "vi/vi.hpp"
#include "vi/vi_time.hpp"

#include <cstdint>
#include <optional>
#include <ostream>
#include <string>

namespace blofeld::vi {
void VirtualInstrument::prepare_audio_clock() {
  if (!config_.audio_clock) { return; }
  const auto &route = *config_.audio_clock;
  constexpr unsigned destinations = 7;
  if (route.destinations == 0 || (route.destinations & ~destinations) != 0) {
    throw Fault("Invalid reference audio clock destinations");
  }
  if (!config_.power_on && route.epoch_ticks < config_.release_ticks) {
    throw Fault("Reference audio clock precedes the nominal DSP release");
  }
  vm_.audio_clock.emplace(SquareClockConfig{.timebase_hz = common_hz(),
                                            .frequency_hz = route.frequency_hz,
                                            .epoch_ticks = route.epoch_ticks,
                                            .initial_level = route.initial_level});
}

std::uint64_t VirtualInstrument::audio_core_hz() const {
  // The clock is supplied even during reset, when the core rate is unknown.
  // Inactive DAX observations retain that explicit unknown without emitting data.
  if (dsp_reset_active()) { return 0; }
  const auto extal = config_.dsp_clock.extal_hz;
  const auto ratio = vm_.dsp.bus().pll().target_ratio();
  const auto numerator = std::uint64_t{extal} * ratio.numerator;
  if (ratio.denominator == 0 || numerator == 0 || numerator % ratio.denominator != 0) {
    throw Fault("Reference DAX rate is not an exact current PLL/EXTAL frequency at X:0xfffffd");
  }
  return numerator / ratio.denominator;
}

void VirtualInstrument::observe_audio_clock(const std::uint64_t at) {
  if (!config_.audio_clock || !vm_.audio_clock) {
    throw Fault("Missing reference audio clock source/route");
  }
  auto &source = *vm_.audio_clock;
  const auto &route = *config_.audio_clock;
  vm_.audio_deadline.valid = false;
  const bool baseline = !vm_.audio_clock_started;
  if (!baseline && batch_audio_clock()) {
    advance_audio_clock_span(at);
    return;
  }
  const auto event = baseline ? source.current() : source.next();
  if (event.ticks != at) { throw Fault("Reference audio clock event changed its timestamp"); }
  constexpr unsigned dax_aci = 4;
  std::uint64_t core_hz = 0;
  try {
    for (unsigned index = 0; index < 2; ++index) {
      if ((route.destinations & (1U << index)) == 0) { continue; }
      const auto instance = index == 0 ? DspEsaiInstance::primary : DspEsaiInstance::secondary;
      constexpr unsigned hckt_pad = 0x20;
      vm_.dsp.bus()
          .gpio(index == 0 ? DspPort::c : DspPort::e)
          .update_drive(hckt_pad, {.mask = hckt_pad, .levels = event.level ? hckt_pad : 0U});
      vm_.dsp.bus().observe_esai_transmit(
          instance,
          {.high_clock = event.level, .timing = DspEsaiClockTiming::reference_reset_edges});
    }
    if ((route.destinations & dax_aci) != 0) {
      constexpr unsigned aci_pad = 1;
      vm_.dsp.bus()
          .gpio(DspPort::d)
          .update_drive(aci_pad, {.mask = aci_pad, .levels = event.level ? aci_pad : 0U});
      core_hz = audio_core_hz();
      vm_.dsp.bus().observe_dax({.aci = event.level,
                                 .rates = {.core_hz = core_hz, .aci_hz = route.frequency_hz},
                                 .timing = DspDaxTiming::reference_rising_edges});
    }
  } catch (const Fault &error) {
    throw Fault("Reference audio clock input at=" + std::to_string(at) +
                " destinations=" + hex(route.destinations, 2) + " mcu-pc=" + mcu_pc_context() +
                " dsp-pc=" + hex(vm_.dsp.cpu().registers().pc, 6) + ": " + error.what());
  }
  // A partial fanout fault quarantines the VirtualInstrument; only a joint restore may retry.
  if (!baseline) { source.consume(at); }
  vm_.audio_clock_started = true;
  trace_audio_clock(route, event, baseline);
  observe_pcm(at, "audio-clock");
}

bool VirtualInstrument::transmitter_clock_events() const {
  return config_.audio_clock &&
         config_.audio_clock->scheduling == AudioClockScheduling::transmitter_events;
}

bool VirtualInstrument::batch_audio_clock() const {
  return config_.audio_clock && transmitter_clock_events() && vm_.audio_clock_started;
}

std::optional<std::uint64_t> VirtualInstrument::next_audio_clock_event() const {
  if (!config_.audio_clock || !transmitter_clock_events() || !vm_.audio_clock) {
    return std::nullopt;
  }
  const auto &source = *vm_.audio_clock;
  if (!vm_.audio_clock_started) { return source.current().ticks; }
  if (!batch_audio_clock()) { return source.next().ticks; }
  auto &cached = vm_.audio_deadline;
  const auto revision = vm_.dsp.bus().peripheral_revision();
  if (cached.valid && cached.revision == revision) { return cached.ticks; }
  std::optional<std::uint64_t> edges;
  for (unsigned index = 0; index < 2; ++index) {
    if ((config_.audio_clock->destinations & (1U << index)) == 0) { continue; }
    const auto instance = index == 0 ? DspEsaiInstance::primary : DspEsaiInstance::secondary;
    const auto next = vm_.dsp.bus().esai_high_clock_edges_until_event(instance);
    if (next && (!edges || *next < *edges)) { edges = next; }
  }
  constexpr unsigned dax_aci = 4;
  if ((config_.audio_clock->destinations & dax_aci) != 0) {
    const auto dax = vm_.dsp.bus().dax_aci_edges_until_event();
    if (dax && (!edges || *dax < *edges)) { edges = dax; }
  }
  cached.ticks = edges ? std::optional{source.after(*edges).ticks} : std::nullopt;
  cached.revision = revision;
  cached.valid = true;
  return cached.ticks;
}

void VirtualInstrument::synchronize_audio_clock(const std::uint64_t at) {
  if (!batch_audio_clock() || !vm_.audio_clock) { return; }
  const auto &source = *vm_.audio_clock;
  auto through = at;
  const auto event = next_audio_clock_event();
  // A due status event belongs to the calendar's declared tie order. Only its
  // preceding quiet transitions may be consumed during native preparation.
  if (event && *event <= at) {
    if (*event == 0) { return; }
    through = *event - 1;
  }
  if (through <= source.current().ticks) { return; }
  const auto edges = (through - source.current().ticks) / source.half_period_ticks();
  if (edges != 0) { advance_audio_clock_span(source.after(edges).ticks); }
}

void VirtualInstrument::advance_audio_clock_span(const std::uint64_t at) {
  try {
    if (!config_.audio_clock || !batch_audio_clock() || !vm_.audio_clock) {
      throw Fault("Periodic audio span has no functional clock owner");
    }
    auto &source = *vm_.audio_clock;
    const auto start = source.current().ticks;
    const auto half_period = source.half_period_ticks();
    if (at <= start || (at - start) % half_period != 0) {
      throw Fault("Periodic audio span must end on a later HCKT transition");
    }
    const auto edges = (at - start) / half_period;
    const auto event = source.after(edges);
    for (unsigned index = 0; index < 2; ++index) {
      if ((config_.audio_clock->destinations & (1U << index)) == 0) { continue; }
      const auto instance = index == 0 ? DspEsaiInstance::primary : DspEsaiInstance::secondary;
      constexpr unsigned hckt_pad = 0x20;
      vm_.dsp.bus()
          .gpio(index == 0 ? DspPort::c : DspPort::e)
          .update_drive(hckt_pad, {.mask = hckt_pad, .levels = event.level ? hckt_pad : 0U});

      vm_.dsp.bus().advance_esai_high_clock(instance, edges);
    }
    constexpr unsigned dax_aci = 4;
    if ((config_.audio_clock->destinations & dax_aci) != 0) {
      constexpr unsigned aci_pad = 1;
      vm_.dsp.bus()
          .gpio(DspPort::d)
          .update_drive(aci_pad, {.mask = aci_pad, .levels = event.level ? aci_pad : 0U});
      vm_.dsp.bus().advance_dax_aci(
          edges, {.core_hz = audio_core_hz(), .aci_hz = config_.audio_clock->frequency_hz});
    }
    source.consume_span(at);
    observe_pcm(at, "transmitter-span");
  } catch (const Fault &error) {
    throw Fault("Periodic DSP audio clock operation=advance at=" + std::to_string(at) +
                " mcu-pc=" + mcu_pc_context() + " dsp-pc=" + hex(vm_.dsp.cpu().registers().pc, 6) +
                ": " + error.what());
  }
}

void VirtualInstrument::trace_audio_clock(const AudioClockRoute &route,
                                          const SquareClockLevel event,
                                          const bool baseline) const {
  if (execution_.trace == nullptr) { return; }
  *execution_.trace << "DSP-AUDIO-CLOCK requested=" << event.ticks << " actual=" << event.ticks
                    << " baseline=" << baseline << " level=" << event.level
                    << " destinations=" << hex(route.destinations, 2) << '\n';
}
}
