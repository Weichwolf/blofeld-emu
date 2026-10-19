// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/mcu_midi.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/midi_serial.hpp"

#include <cstdint>
#include <exception>
#include <span>

namespace blofeld {
namespace {
struct ReceivePin {
  McuPort port;
  std::uint8_t mask;
  std::uint16_t address;
};

ReceivePin receive_pin(const SciReceivePin pin) {
  switch (pin) {
    case SciReceivePin::pr2: return {.port = McuPort::r, .mask = 4, .address = hc12::ptr};
    case SciReceivePin::ps4: return {.port = McuPort::s, .mask = 16, .address = hc12::pts};
    default: throw Fault("Unknown MCU MIDI receive pin");
  }
}
}

McuMidiInput::McuMidiInput(const std::uint32_t bus_hz, const SciReceivePin pin)
    : serial_(bus_hz), pin_(pin) {
  static_cast<void>(receive_pin(pin));
}

void McuMidiInput::enqueue(const std::span<const std::uint8_t> bytes) {
  if (faulted_) {
    throw Fault("MIDI receive source requires coordinated restore after a rejected clock slice");
  }
  serial_.enqueue(bytes);
}

void McuMidiInput::advance_sci(McuBus &bus, const ExecutionContext context, const unsigned cycles) {
  if (faulted_) {
    throw Fault("MIDI receive source requires coordinated restore after a rejected clock slice");
  }
  constexpr unsigned maximum_cycles = 1048576;
  if (cycles > maximum_cycles) { throw Fault("MIDI receive clock slice exceeds cycle limit"); }
  if (cycles == 0) { return; }
  const auto pin = receive_pin(pin_);
  unsigned remaining = cycles;
  bus.begin_instruction(context);
  try {
    while (remaining != 0) {
      const auto slice = serial_.next_slice(remaining);
      bus.update_pin_drive(
          pin.port,
          pin.mask,
          {.mask = pin.mask, .levels = static_cast<std::uint8_t>(slice.level ? pin.mask : 0)});
      // Resolve known contention before any clock advances; unknown neighbors stay unknown.
      const auto observed = bus.observe_pads(pin.port, pin.mask);
      if (observed.known != pin.mask) { throw Fault("Unknown MIDI receive pad state"); }
      bus.advance_sci(slice.bus_cycles);
      serial_.consume(slice.bus_cycles);
      remaining -= slice.bus_cycles;
    }
  } catch (const std::exception &error) {
    faulted_ = true;
    throw Fault("MIDI receive clock address=" + hex(pin.address) + " pc=" + hex(context.pc) +
                " ccr=" + hex(context.ccr, 2) +
                " pin=" + (pin_ == SciReceivePin::pr2 ? "PR2" : "PS4") + ": " + error.what());
  }
}

}
