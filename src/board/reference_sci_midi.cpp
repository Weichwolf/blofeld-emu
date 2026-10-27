// SPDX-License-Identifier: GPL-3.0-only
#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_midi.hpp"

#include <cstdint>
#include <optional>
#include <ostream>
#include <string>

namespace blofeld::reference {
void Board::observe_sci_midi(const std::uint64_t at) {
  if (!control_.midi_sci || !config_.midi_sci_timed) {
    throw Fault("Timed SCI MIDI event has no source/route");
  }
  const auto event = control_.midi_sci->next_event();
  if (event.ticks != at) { throw Fault("Timed SCI MIDI event changed its deadline"); }
  const bool small = config_.midi_sci_timed->pin == SciReceivePin::ps4;
  const auto port = small ? McuPort::s : McuPort::r;
  const auto mask = static_cast<std::uint8_t>(small ? 16 : 4);
  const auto address = small ? hc12::pts : hc12::ptr;
  try {
    auto &bus = native_->mcu.bus();
    bus.update_pin_drive(
        port, mask, {.mask = mask, .levels = static_cast<std::uint8_t>(event.level ? mask : 0)});
    if (bus.observe_pads(port, mask).known != mask) {
      throw Fault("Unknown timed SCI MIDI receive pad");
    }
    if (execution_.trace != nullptr) {
      auto &trace = *execution_.trace;
      trace << "SCI-MIDI-BIT requested=" << event.ticks << " actual=" << at
            << " level=" << event.level << " bit=";
      if (event.frame_bit) {
        trace << *event.frame_bit;
      } else {
        trace << "idle";
      }
      trace << " held-reset=" << native_->mcu.system_reset_active() << '\n';
      if (!trace) { throw Fault("Timed SCI MIDI trace failed"); }
    }
    control_.midi_sci->consume(at);
  } catch (const Fault &error) {
    throw Fault("Timed SCI MIDI input address=" + hex(address) + " pc=" + hex(control_.context.pc) +
                " ccr=" + hex(control_.context.ccr, 2) + " ticks=" + std::to_string(at) + ": " +
                error.what());
  }
}
}
