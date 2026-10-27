// SPDX-License-Identifier: GPL-3.0-only
#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_midi.hpp"
#include "mcu/mcu_sci.hpp"
#include "midi/midi_serial.hpp"
#include "midi/midi_timed.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <ostream>
#include <span>

namespace blofeld::reference {
namespace {
struct Pad {
  McuPort port;
  std::uint8_t mask;
};

Pad pad(const SciReceivePin pin) {
  switch (pin) {
    case SciReceivePin::pr2: return {.port = McuPort::r, .mask = 4};
    case SciReceivePin::ps4: return {.port = McuPort::s, .mask = 16};
    default: throw Fault("Unknown reference MIDI receive pin");
  }
}
}

void validate_midi(const Config &config) {
  validate_dsp_midi(config);
  if (config.midi_reference) {
    if (!config.midi && !config.midi_sci_timed) {
      throw Fault("MIDI transfer reference requires an SCI route");
    }
    McuSci validator;
    validator.set_receive_reference(config.midi_reference);
  }
  if (config.midi_sci_timed) {
    if (!config.pipeline || config.midi ||
        config.dsp_input_order == DspExternalOrder::unqualified ||
        config.maximum_dsp_inputs_per_advance == 0) {
      throw Fault(
          "Timed SCI MIDI requires pipeline ownership, exclusive route and event order/budget");
    }
    static_cast<void>(MidiTimedInput({.timebase_hz = config.pipeline->schedule.dsp.timebase_hz,
                                      .start_ticks = config.midi_sci_timed->after_ticks}));
  }
  if (!config.midi && !config.midi_sci_timed) { return; }
  if (config.midi && config.midi->bus_hz < MidiSerialInput::baud) {
    throw Fault("Reference MIDI requires at least one bus period per wire bit");
  }
  const auto receive = pad(config.midi ? config.midi->pin : config.midi_sci_timed->pin);
  const auto overlaps = [&](const auto &route) {
    return route && route->destination == receive.port && (route->mask & receive.mask) != 0;
  };
  if (overlaps(config.hreq) || overlaps(config.miso) ||
      (config.loopback && config.loopback->destination == receive.port &&
       (config.loopback->destination_mask & receive.mask) != 0)) {
    throw Fault("Reference MIDI receive pad conflicts with a board return/loopback route");
  }
  for (const auto &input : config.events) {
    if (input.kind == InputKind::drive && input.port == receive.port &&
        ((input.drive.mask | input.drive.floating | input.drive.unknown) & receive.mask) != 0) {
      throw Fault("Reference MIDI receive pad conflicts with a timed external drive");
    }
  }
  for (const auto &input : config.initial) {
    if (input.kind == InputKind::drive && input.port == receive.port &&
        ((input.drive.mask | input.drive.floating | input.drive.unknown) & receive.mask) != 0 &&
        ((input.drive.mask & input.drive.levels & receive.mask) != receive.mask)) {
      throw Fault("Initial MIDI receive-pad observation must be known idle-high");
    }
  }
}

void Board::start_midi() {
  if (!config_.midi || control_.midi || ticks() < config_.midi->after_ticks) { return; }
  if (config_.midi_reference) { native_->mcu.set_sci_receive_reference(config_.midi_reference); }
  control_.midi.emplace(config_.midi->bus_hz, config_.midi->pin, native_->mcu.run_clock());
  if (execution_.trace != nullptr) {
    *execution_.trace << "MIDI-START requested=" << config_.midi->after_ticks
                      << " actual=" << ticks() << " bus-hz=" << config_.midi->bus_hz;
    if (config_.midi_reference) {
      *execution_.trace << " reference-sbr=" << config_.midi_reference->baud_divisor
                        << " reference-delay=" << config_.midi_reference->transfer_delay_bus_cycles;
    }
    *execution_.trace << '\n';
  }
}

void Board::observe_midi(const ExecutionContext context) {
  if (control_.midi) {
    control_.midi->observe(native_->mcu.bus(), native_->mcu.run_clock(), context);
  }
}

void Board::synchronize_midi() {
  if (control_.midi) {
    const auto cpu = native_->mcu.cpu().state();
    control_.midi->synchronize(native_->mcu.run_clock(), {.pc = cpu.pc, .ccr = cpu.ccr});
  }
}

MidiInputWindow Board::midi_window() const {
  require_healthy();
  MidiInputWindow result{.ticks = ticks(),
                         .configured = config_.midi.has_value() ||
                                       config_.midi_sci_timed.has_value() ||
                                       config_.midi_dsp.has_value(),
                         .owner = identity_,
                         .restore_revision = usb_restore_revision_};
  result.started =
      control_.midi.has_value() || ((control_.midi_sci || control_.midi_dsp) && control_.started);
  if (result.started) { result.available = MidiSerialInput::capacity - midi_queue_size(); }
  return result;
}

std::size_t Board::midi_queue_size() const {
  if (control_.midi_sci) { return control_.midi_sci->queued_bytes(); }
  if (control_.midi_dsp) { return control_.midi_dsp->queued_bytes(); }
  return control_.midi ? control_.midi->queued_bytes() : 0;
}

void Board::enqueue_midi(const std::span<const std::uint8_t> bytes) {
  require_healthy();
  if (!control_.midi && ((!control_.midi_dsp && !control_.midi_sci) || !control_.started)) {
    throw Fault("Reference MIDI input precedes its configured start boundary");
  }
  if (bytes.size() > std::numeric_limits<std::uint64_t>::max() - control_.midi_bytes) {
    throw Fault("Reference MIDI byte counter exhausted");
  }
  if (control_.midi_sci) {
    control_.midi_sci->enqueue(bytes);
  } else if (control_.midi_dsp) {
    control_.midi_dsp->enqueue(bytes);
  } else {
    control_.midi->enqueue(bytes);
  }
  control_.midi_bytes += bytes.size();
  try {
    if (execution_.trace != nullptr && !bytes.empty()) {
      *execution_.trace << "MIDI-INPUT actual=" << ticks() << " operation=" << control_.operations
                        << " bytes=";
      for (const auto byte : bytes) { *execution_.trace << hex(byte, 2).substr(2); }
      *execution_.trace << '\n';
      if (!*execution_.trace) { throw Fault("Reference MIDI input trace output failed"); }
    }
  } catch (...) {
    mark_fault();
    throw;
  }
}
}
