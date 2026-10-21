// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_midi.hpp"
#include "blofeld/mcu_sci.hpp"
#include "blofeld/midi_serial.hpp"
#include "reference_board.hpp"

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
  if (config.midi_reference) {
    if (!config.midi) { throw Fault("MIDI transfer reference requires midi-sci"); }
    McuSci validator;
    validator.set_receive_reference(config.midi_reference);
  }
  if (!config.midi) { return; }
  if (config.midi->bus_hz < MidiSerialInput::baud) {
    throw Fault("Reference MIDI requires at least one bus period per wire bit");
  }
  const auto receive = pad(config.midi->pin);
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
        ((input.drive.mask | input.drive.floating) & receive.mask) != 0) {
      throw Fault("Reference MIDI receive pad conflicts with a timed external drive");
    }
  }
  for (const auto &input : config.initial) {
    if (input.kind == InputKind::drive && input.port == receive.port &&
        ((input.drive.mask | input.drive.floating) & receive.mask) != 0 &&
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
  return {.ticks = ticks(),
          .available =
              control_.midi ? MidiSerialInput::capacity - control_.midi->queued_bytes() : 0,
          .started = control_.midi.has_value()};
}

void Board::enqueue_midi(const std::span<const std::uint8_t> bytes) {
  require_healthy();
  if (!control_.midi) {
    throw Fault("Reference MIDI input precedes its configured start boundary");
  }
  if (bytes.size() > std::numeric_limits<std::uint64_t>::max() - control_.midi_bytes) {
    throw Fault("Reference MIDI byte counter exhausted");
  }
  control_.midi->enqueue(bytes);
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
    control_.faulted = true;
    throw;
  }
}
}
