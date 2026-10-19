// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/midi_serial.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace blofeld {

enum class SciReceivePin : std::uint8_t { pr2, ps4 };

// Explicit ideal pad connection, not a measured Blofeld wiring/clock profile.
// No bus/CPU references are retained, so a copy can accompany an MCU checkpoint.
class McuMidiInput {
public:
  McuMidiInput(std::uint32_t bus_hz, SciReceivePin pin);
  void enqueue(std::span<const std::uint8_t> bytes);
  // Only SCI time advances; the caller schedules the CPU and all other clocks.
  void advance_sci(McuBus &bus, ExecutionContext context, unsigned cycles);

  [[nodiscard]] std::size_t queued_bytes() const { return serial_.queued_bytes(); }

  [[nodiscard]] bool faulted() const { return faulted_; }

private:
  MidiSerialInput serial_;
  SciReceivePin pin_;
  bool faulted_ = false;
};

}
