// SPDX-License-Identifier: GPL-3.0-only
#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_midi_output.hpp"
#include "midi/midi_serial_receiver.hpp"

#include <cstdint>
#include <optional>
#include <ostream>

namespace blofeld::reference {
void validate_midi_output(const Config &config) {
  if (!config.midi_output) { return; }
  const McuMidiOutput validator(*config.midi_output, config.dsp_clock.timebase_hz);
  constexpr std::uint8_t ps5 = 32;
  const bool large = *config.midi_output == SciTransmitPin::pr3;
  const auto port = large ? McuPort::r : McuPort::s;
  const auto mask = static_cast<std::uint8_t>(large ? 8 : ps5);
  const auto overlaps = [&](const auto &route) {
    return route && route->destination == port && (route->mask & mask) != 0;
  };
  if (overlaps(config.hreq) || overlaps(config.miso) ||
      (config.loopback && config.loopback->destination == port &&
       (config.loopback->destination_mask & mask) != 0)) {
    throw Fault("MIDI output pad conflicts with a board return/loopback route");
  }
  const auto driven = [&](const Input &input) {
    return input.kind == InputKind::drive && input.port == port &&
           ((input.drive.mask | input.drive.floating | input.drive.unknown) & mask) != 0;
  };
  for (const auto &input : config.initial) {
    if (driven(input)) { throw Fault("MIDI output pad has an initial external drive"); }
  }
  for (const auto &input : config.events) {
    if (driven(input)) { throw Fault("MIDI output pad has a timed external drive"); }
  }
}

void Board::observe_midi_output() {
  if (control_.midi_output) {
    control_.midi_output->observe(native_->mcu.bus(), ticks(), control_.context);
  }
}

MidiOutputWindow Board::midi_output_window() const {
  require_healthy();
  return {.ticks = ticks(),
          .queued_bytes = control_.midi_output ? control_.midi_output->queued_bytes() : 0,
          .transmitted_bytes = control_.midi_output ? control_.midi_output->transmitted_bytes() : 0,
          .configured = control_.midi_output.has_value(),
          .owner = identity_,
          .restore_revision = usb_restore_revision_};
}

std::optional<MidiSerialByte> Board::pop_midi_output() {
  require_healthy();
  if (!control_.midi_output) { throw Fault("Reference MIDI output is not configured"); }
  try {
    const auto result = control_.midi_output->pop();
    if (result && execution_.trace != nullptr) {
      auto &stream = *execution_.trace;
      stream << "MIDI-OUTPUT start=" << result->start_ticks
             << " stop-sample=" << result->stop_sample_ticks << " value=" << hex(result->value, 2)
             << '\n';
      if (!stream) { throw Fault("MIDI output trace failed after queue retirement"); }
    }
    return result;
  } catch (...) {
    mark_fault();
    throw;
  }
}
}
