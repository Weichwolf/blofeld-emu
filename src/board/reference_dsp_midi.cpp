// SPDX-License-Identifier: GPL-3.0-only
#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "mcu/hc12.hpp"
#include "midi/midi_timed.hpp"

#include <cstdint>
#include <ostream>

namespace blofeld::reference {
void validate_dsp_midi(const Config &config) {
  if (config.midi_dsp_reference && !config.midi_dsp) {
    throw Fault("DSP MIDI reference period requires midi-dsp-tio");
  }
  if (!config.midi_dsp) { return; }
  const auto route = *config.midi_dsp;
  if (config.midi || config.midi_reference || config.midi_sci_timed) {
    throw Fault("Host MIDI routes must be exclusive");
  }
  if (route.index > 1 || route.after_ticks < config.release_ticks ||
      config.dsp_input_order == DspExternalOrder::unqualified ||
      config.maximum_dsp_inputs_per_advance == 0) {
    throw Fault(
        "DSP MIDI requires a valid timer, post-release start and explicit event order/budget");
  }
  MidiWireTiming timing{.timebase_hz = config.pipeline ? config.pipeline->schedule.dsp.timebase_hz
                                                       : config.dsp.clock.timebase_hz,
                        .start_ticks = route.after_ticks};
  if (config.midi_dsp_reference) {
    timing.reference_bit_ticks = config.midi_dsp_reference->bit_ticks;
    timing.reference_idle_bits = config.midi_dsp_reference->idle_bits;
  }
  static_cast<void>(MidiTimedInput(timing));
  for (const auto &input : config.dsp_events) {
    if (input.kind == DspInputKind::tio && input.index == route.index) {
      throw Fault("DSP MIDI timer input conflicts with a static timed source");
    }
  }
  for (const auto &input : config.dsp_initial) {
    if (input.kind == DspInputKind::tio && input.index == route.index && input.level != true) {
      throw Fault("Initial DSP MIDI observation must be known idle-high");
    }
  }
}

void Board::trace_dsp_midi(const std::uint64_t at) {
  if (execution_.trace == nullptr) { return; }
  if (!control_.midi_dsp || !config_.midi_dsp) {
    throw Fault("DSP MIDI trace requires its source and configured route");
  }
  const auto event = control_.midi_dsp->next_event();
  auto &trace = *execution_.trace;
  trace << "DSP-MIDI-BIT requested=" << event.ticks << " actual=" << at
        << " tio=" << config_.midi_dsp->index << " level=" << event.level << " bit=";
  if (event.frame_bit) {
    trace << *event.frame_bit;
  } else {
    trace << "idle";
  }
  trace << " synchronized=" << config_.midi_dsp->synchronized
        << " mcu-pc=" << hex(native_->mcu.cpu().state().pc)
        << " dsp-pc=" << hex(native_->dsp.cpu().state().pc, 6) << '\n';
  if (!trace) { throw Fault("Reference DSP MIDI trace output failed"); }
}
}
