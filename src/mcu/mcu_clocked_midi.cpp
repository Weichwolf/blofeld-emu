// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/mcu_clocked_midi.hpp"

#include "common/bus.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_clock.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_midi.hpp"
#include "mcu/mcu_run_clock.hpp"

#include <cstdint>
#include <exception>
#include <span>
#include <string>

namespace blofeld {
namespace {
struct ReceiveRoute {
  McuPort port;
  std::uint8_t mask;
  std::uint16_t address;
};

ReceiveRoute receive_route(const SciReceivePin pin) {
  switch (pin) {
    case SciReceivePin::pr2: return {.port = McuPort::r, .mask = 4, .address = hc12::ptr};
    case SciReceivePin::ps4: return {.port = McuPort::s, .mask = 16, .address = hc12::pts};
    default: throw Fault("Unknown clocked MIDI receive pin");
  }
}
}

McuClockedMidiInput::McuClockedMidiInput(const std::uint32_t bus_hz,
                                         const SciReceivePin pin,
                                         const McuRunClock &clock)
    : serial_(bus_hz),
      pin_(pin),
      bus_hz_(bus_hz),
      timebase_hz_(clock.timebase_hz()),
      last_cycles_(clock.bus_cycles()),
      last_ticks_(clock.elapsed_ticks()) {
  static_cast<void>(receive_route(pin));
  if (clock.faulted()) { throw Fault("Clocked MIDI requires a healthy initial MCU clock"); }
}

void McuClockedMidiInput::enqueue(const std::span<const std::uint8_t> bytes) {
  if (faulted_) { throw Fault("Clocked MIDI requires coordinated restore after failure"); }
  if (offered_) { throw Fault("Clocked MIDI enqueue requires synchronization after observation"); }
  serial_.enqueue(bytes);
}

void McuClockedMidiInput::commit(const McuRunClock &clock) {
  if (faulted_ || clock.faulted()) { throw Fault("Clocked MIDI requires coordinated recovery"); }
  const auto cycles = clock.bus_cycles();
  const auto ticks = clock.elapsed_ticks();
  if (clock.timebase_hz() != timebase_hz_ || cycles < last_cycles_ || ticks < last_ticks_) {
    throw Fault("Clocked MIDI peer has an incompatible or restored MCU clock");
  }
  const auto elapsed = cycles - last_cycles_;
  if (elapsed == 0) {
    if (ticks != last_ticks_) { throw Fault("Clocked MIDI time advanced without an IPBus period"); }
    return;
  }
  if (elapsed != 1 || !offered_ || ticks - last_ticks_ != offered_ticks_) {
    throw Fault("Clocked MIDI missed an observed IPBus period or changed its declared rate");
  }
  serial_.consume(1);
  last_cycles_ = cycles;
  last_ticks_ = ticks;
  offered_ = false;
}

void McuClockedMidiInput::observe(McuBus &bus,
                                  const McuRunClock &clock,
                                  const ExecutionContext context) {
  try {
    commit(clock);
    const auto route = bus.run_clock_route();
    const auto source =
        route.source == McuClockSource::usb_phy ? McuRunClock::phy_hz : clock.oscillator_hz();
    if (source != static_cast<std::uint64_t>(bus_hz_) * route.bus_divider ||
        timebase_hz_ % bus_hz_ != 0) {
      throw Fault("Clocked MIDI fixed bus rate does not match native clock selection/divider");
    }
    const auto pin = receive_route(pin_);
    const auto slice = serial_.next_slice(1);
    bus.update_pin_drive(
        pin.port,
        pin.mask,
        {.mask = pin.mask, .levels = static_cast<std::uint8_t>(slice.level ? pin.mask : 0)});
    if (bus.observe_pads(pin.port, pin.mask).known != pin.mask) {
      throw Fault("Unknown clocked MIDI receive pad");
    }
    offered_ticks_ = timebase_hz_ / bus_hz_;
    offered_ = true;
  } catch (const std::exception &error) { fail(context, error.what()); }
}

void McuClockedMidiInput::synchronize(const McuRunClock &clock, const ExecutionContext context) {
  try {
    commit(clock);
    offered_ = false;
  } catch (const std::exception &error) { fail(context, error.what()); }
}

void McuClockedMidiInput::fail(const ExecutionContext context, const std::string &message) {
  faulted_ = true;
  throw Fault("Clocked MIDI input address=" + hex(receive_route(pin_).address) +
              " pc=" + hex(context.pc) + " ccr=" + hex(context.ccr, 2) + ": " + message);
}

}
