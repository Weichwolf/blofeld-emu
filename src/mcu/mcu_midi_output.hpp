// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "mcu/mcu_bus.hpp"
#include "midi/midi_serial_receiver.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

namespace blofeld {
enum class SciTransmitPin : std::uint8_t { pr3, ps5 };

// Passive fixed-pad MIDI capture. Observe every resolved wire change in a
// declared timebase; neither reading pads nor draining bytes advances the MCU.
// Copies retain the partial frame and bounded output queue together.
class McuMidiOutput {
public:
  static constexpr std::size_t capacity = 256;
  McuMidiOutput(SciTransmitPin pin, std::uint64_t timebase_hz);
  void observe(const McuBus &bus, std::uint64_t ticks, ExecutionContext context);
  [[nodiscard]] std::optional<MidiSerialByte> pop();

  [[nodiscard]] std::size_t queued_bytes() const noexcept { return size_; }

  [[nodiscard]] std::uint64_t transmitted_bytes() const noexcept {
    return serial_.received_bytes();
  }

  [[nodiscard]] bool faulted() const noexcept { return faulted_; }

private:
  [[noreturn]] void fail(std::uint64_t ticks, ExecutionContext context, std::string_view reason);
  SciTransmitPin pin_;
  MidiSerialReceiver serial_;
  std::array<MidiSerialByte, capacity> queue_{};
  std::size_t head_ = 0;
  std::size_t size_ = 0;
  bool faulted_ = false;
};
}
