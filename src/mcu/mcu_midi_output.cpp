// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/mcu_midi_output.hpp"

#include "common/bus.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "midi/midi_serial_receiver.hpp"

#include <cstdint>
#include <exception>
#include <optional>
#include <string>
#include <string_view>

namespace blofeld {
namespace {
struct TransmitPad {
  McuPort port;
  std::uint8_t mask;
  std::uint16_t address;
};

TransmitPad pad(const SciTransmitPin pin) {
  constexpr std::uint8_t pr3 = 8;
  constexpr std::uint8_t ps5 = 32;
  switch (pin) {
    case SciTransmitPin::pr3: return {.port = McuPort::r, .mask = pr3, .address = hc12::ptr};
    case SciTransmitPin::ps5: return {.port = McuPort::s, .mask = ps5, .address = hc12::pts};
    default: throw Fault("Unknown MIDI transmit pad");
  }
}
}

McuMidiOutput::McuMidiOutput(const SciTransmitPin pin, const std::uint64_t timebase_hz)
    : pin_(pin), serial_(timebase_hz) {
  static_cast<void>(pad(pin));
}

void McuMidiOutput::observe(const McuBus &bus,
                            const std::uint64_t ticks,
                            const ExecutionContext context) {
  if (faulted_) { fail(ticks, context, "requires coordinated recovery"); }
  try {
    const auto route = pad(pin_);
    const auto pins = bus.observe_pads(route.port, route.mask);
    std::optional<bool> level;
    if (pins.known == route.mask) { level = (pins.levels & route.mask) != 0; }
    if (const auto byte = serial_.observe(ticks, level)) {
      if (size_ == capacity) { throw Fault("bounded output queue overflow"); }
      queue_.at((head_ + size_) % capacity) = *byte;
      ++size_;
    }
  } catch (const std::exception &error) { fail(ticks, context, error.what()); }
}

std::optional<MidiSerialByte> McuMidiOutput::pop() {
  if (faulted_) { throw Fault("MIDI output pop requires coordinated recovery"); }
  if (size_ == 0) { return std::nullopt; }
  const auto result = queue_.at(head_);
  head_ = (head_ + 1) % capacity;
  --size_;
  return result;
}

void McuMidiOutput::fail(const std::uint64_t ticks,
                         const ExecutionContext context,
                         const std::string_view reason) {
  faulted_ = true;
  throw Fault("MIDI output operation=observe address=" + hex(pad(pin_).address) + " mask=" +
              hex(pad(pin_).mask, 2) + " pc=" + hex(context.pc) + " ccr=" + hex(context.ccr, 2) +
              " ticks=" + std::to_string(ticks) + ": " + std::string(reason));
}
}
