// SPDX-License-Identifier: GPL-3.0-only
#include "board/reference_board.hpp"
#include "board/reference_context.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_esai.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace blofeld::reference {
namespace {
constexpr unsigned maximum_sources = 6;

unsigned input_targets(const DspInput &input,
                       const bool audio,
                       const bool sci,
                       const unsigned audio_destinations) {
  constexpr unsigned esai_shift = 2;
  constexpr unsigned sci_target = 32;
  if (audio) { return audio_destinations << esai_shift; }
  if (sci) { return sci_target; }
  if (input.index > 1) { throw Fault("Invalid reference DSP input target index"); }
  switch (input.kind) {
    case DspInputKind::tio: return 1U << input.index;
    case DspInputKind::esai: return 1U << (esai_shift + input.index);
    default: throw Fault("Invalid reference DSP input target kind");
  }
}
}

void Board::select_dsp_input(std::optional<DspOffer> &selected, const DspOffer candidate) {
  if (!selected || candidate.input.ticks < selected->input.ticks) { selected = candidate; }
}

std::optional<Board::DspOffer> Board::next_dsp_input() const {
  std::optional<DspOffer> result;
  unsigned targets = 0;
  const auto destinations = config_.audio_clock ? config_.audio_clock->destinations : 0;
  const auto offer = [&](const DspOffer candidate) {
    if (result && candidate.input.ticks > result->input.ticks) { return; }
    const auto used =
        input_targets(candidate.input, candidate.audio_clock, candidate.sci_midi, destinations);
    if (!result || candidate.input.ticks < result->input.ticks) {
      targets = used;
      select_dsp_input(result, candidate);
      return;
    }
    result->collision = result->collision || (targets & used) != 0;
    targets |= used;
  };
  if (control_.next_dsp_event < config_.dsp_events.size()) {
    offer({.input = config_.dsp_events.at(control_.next_dsp_event)});
  }
  for (const auto &clock : config_.dsp_clocks) {
    const auto edge = control_.next_esai_edge.at(clock.index);
    if (edge == clock.edges) { continue; }
    const auto at = clock.start_ticks + (edge * clock.half_period_ticks);
    offer(DspOffer{.input = {.ticks = at,
                             .kind = DspInputKind::esai,
                             .index = clock.index,
                             .esai = {.high_clock = edge % 2 == 0,
                                      .timing = DspEsaiClockTiming::reference_reset_edges}},
                   .clock_index = clock.index});
  }
  if (control_.audio_clock && !transmitter_clock_events()) {
    const auto at = control_.audio_clock_started ? control_.audio_clock->next().ticks
                                                 : control_.audio_clock->current().ticks;
    offer(DspOffer{.input = {.ticks = at}, .audio_clock = true});
  }
  if (control_.midi_dsp) {
    if (!config_.midi_dsp) { throw Fault("DSP MIDI source has no configured route"); }
    const auto event = control_.midi_dsp->next_event();
    offer(DspOffer{.input = {.ticks = event.ticks,
                             .index = config_.midi_dsp->index,
                             .level = event.level,
                             .synchronized = config_.midi_dsp->synchronized},
                   .midi = true});
  }
  if (control_.midi_sci) {
    const auto event = control_.midi_sci->next_event();
    offer(DspOffer{.input = {.ticks = event.ticks}, .sci_midi = true});
  }
  return result;
}

void Board::observe_dsp_input(const std::uint64_t at) {
  auto next = next_dsp_input();
  if (!next || at != next->input.ticks) { throw Fault("DSP input source changed its event time"); }
  unsigned targets = 0;
  const auto destinations = config_.audio_clock ? config_.audio_clock->destinations : 0;
  for (unsigned count = 0; count < maximum_sources && next && next->input.ticks == at; ++count) {
    const auto used = input_targets(next->input, next->audio_clock, next->sci_midi, destinations);
    if (next->collision || (targets & used) != 0) {
      throw Fault("Simultaneous reference DSP input sources at=" + std::to_string(at) +
                  " mcu-pc=" + mcu_pc_context(native_->mcu) +
                  " dsp-pc=" + hex(native_->dsp.cpu().registers().pc, 6));
    }
    targets |= used;
    if (next->audio_clock) {
      observe_audio_clock(at);
      next = next_dsp_input();
      continue;
    }
    if (next->sci_midi) {
      observe_sci_midi(at);
      next = next_dsp_input();
      continue;
    }
    apply_dsp(next->input);
    if (next->midi) {
      trace_dsp_midi(at);
    } else {
      trace_dsp_input(next->input, at);
    }
    observe_pcm(at, "external");
    if (next->midi) {
      if (!control_.midi_dsp) { throw Fault("Scheduled DSP MIDI event has no source"); }
      control_.midi_dsp->consume(at);
    } else if (next->clock_index.has_value()) {
      ++control_.next_esai_edge.at(next->clock_index.value());
    } else {
      ++control_.next_dsp_event;
    }
    next = next_dsp_input();
  }
  if (next && next->input.ticks == at) {
    throw Fault("Reference DSP input batch exceeds its bounded source count at=" +
                std::to_string(at));
  }
}
}
