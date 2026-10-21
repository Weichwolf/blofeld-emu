// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_esai.hpp"
#include "blofeld/hc12.hpp"
#include "reference_board.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace blofeld::reference {
std::optional<Board::DspOffer> Board::next_dsp_input() const {
  std::optional<DspOffer> result;
  if (control_.next_dsp_event < config_.dsp_events.size()) {
    result = DspOffer{.input = config_.dsp_events.at(control_.next_dsp_event)};
  }
  for (const auto &clock : config_.dsp_clocks) {
    const auto edge = control_.next_esai_edge.at(clock.index);
    if (edge == clock.edges) { continue; }
    const auto at = clock.start_ticks + (edge * clock.half_period_ticks);
    if (result && at > result->input.ticks) { continue; }
    if (result && at == result->input.ticks) {
      result->collision = true;
      continue;
    }
    result = DspOffer{.input = {.ticks = at,
                                .kind = DspInputKind::esai,
                                .index = clock.index,
                                .esai = {.high_clock = edge % 2 == 0,
                                         .timing = DspEsaiClockTiming::reference_reset_edges}},
                      .clock_index = clock.index};
  }
  if (control_.midi_dsp) {
    if (!config_.midi_dsp) { throw Fault("DSP MIDI source has no configured route"); }
    const auto event = control_.midi_dsp->next_event();
    if (result && event.ticks == result->input.ticks) {
      result->collision = true;
    } else if (!result || event.ticks < result->input.ticks) {
      result = DspOffer{.input = {.ticks = event.ticks,
                                  .index = config_.midi_dsp->index,
                                  .level = event.level,
                                  .synchronized = config_.midi_dsp->synchronized},
                        .midi = true};
    }
  }
  return result;
}

void Board::observe_dsp_input(const std::uint64_t at) {
  const auto next = next_dsp_input();
  if (!next || at != next->input.ticks) { throw Fault("DSP input source changed its event time"); }
  if (next->collision) {
    throw Fault("Simultaneous reference DSP input sources at=" + std::to_string(at) +
                " mcu-pc=" + hex(native_->mcu.cpu().state().pc) +
                " dsp-pc=" + hex(native_->dsp.cpu().state().pc, 6));
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
}
}
