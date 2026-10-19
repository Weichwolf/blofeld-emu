// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_midi.hpp"
#include "blofeld/mcu_run_clock.hpp"
#include "blofeld/midi_serial.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>

namespace blofeld {

// Caller-owned fixed-rate wire peer. Does not advance any MCU clock or retain
// MCU references. Copy together with a healthy MCU checkpoint after synchronize.
class McuClockedMidiInput {
public:
  McuClockedMidiInput(std::uint32_t bus_hz, SciReceivePin pin, const McuRunClock &clock);
  void enqueue(std::span<const std::uint8_t> bytes);
  // Before each enabled IPBus period, from both CPU and WAI observers. Repeated
  // observations at an unadvanced interrupt boundary consume no wire time.
  void observe(McuBus &bus, const McuRunClock &clock, ExecutionContext context);
  // After each owner operation, before enqueue or coordinated snapshot/copy.
  // Commits the last completed period and cancels an unused observation.
  void synchronize(const McuRunClock &clock, ExecutionContext context);

  [[nodiscard]] std::size_t queued_bytes() const { return serial_.queued_bytes(); }

  [[nodiscard]] bool faulted() const { return faulted_; }

private:
  void commit(const McuRunClock &clock);
  [[noreturn]] void fail(ExecutionContext context, const std::string &message);
  MidiSerialInput serial_;
  SciReceivePin pin_;
  std::uint32_t bus_hz_;
  std::uint64_t timebase_hz_;
  std::uint64_t last_cycles_;
  std::uint64_t last_ticks_;
  std::uint64_t offered_ticks_ = 0;
  bool offered_ = false;
  bool faulted_ = false;
};

}
