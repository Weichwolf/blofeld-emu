// SPDX-License-Identifier: GPL-3.0-only
#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_shi.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"

#include <cstdint>
#include <ostream>

namespace blofeld::reference {
namespace {
bool same(const SpiInputs &first, const SpiInputs &second) {
  return first.selected == second.selected && first.clock == second.clock &&
         first.mosi == second.mosi;
}
}

SpiInputs Board::sample() const {
  const auto mask = static_cast<std::uint8_t>(config_.select_n | config_.clock | config_.mosi);
  const auto pins = native_->mcu.bus().observe_pads(McuPort::b, mask);
  if (pins.known != mask) {
    throw Fault("Unknown reference SPI pad read at 0x0001 mask=" + hex(mask, 2) +
                " known=" + hex(pins.known, 2) + " mcu-pc=" + hex(native_->mcu.cpu().state().pc) +
                " dsp-pc=" + hex(native_->dsp.cpu().state().pc, 6));
  }
  return {.selected = (pins.levels & config_.select_n) == 0,
          .clock = (pins.levels & config_.clock) != 0,
          .mosi = (pins.levels & config_.mosi) != 0};
}

void Board::check_signals() const {
  if (control_.release_epoch && !same(sample(), control_.previous)) {
    throw Fault("Unobserved reference SPI pad change at 0x0001 mcu-pc=" +
                hex(native_->mcu.cpu().state().pc) +
                " dsp-pc=" + hex(native_->dsp.cpu().state().pc, 6));
  }
}

void Board::update_loopback() {
  if (!config_.loopback) { return; }
  const auto &route = *config_.loopback;
  const auto input = native_->mcu.bus().observe_pads(route.source, route.source_mask);
  PinDrive output;
  if (input.known == route.source_mask) {
    output.mask = route.destination_mask;
    output.levels = (input.levels & route.source_mask) != 0 ? route.destination_mask : 0;
  }
  // An unknown pad does not establish a high-impedance source. Preserve
  // unknownness rather than allowing a receiver pull to invent a known level.
  native_->mcu.bus().update_pin_drive(route.destination, route.destination_mask, output);
}

void Board::transition(const SpiInputs pins) {
  if (control_.previous.selected != pins.selected && control_.previous.clock != pins.clock) {
    ++control_.ordered_pairs;
    switch (config_.pad_order) {
      case PadOrder::reject:
        throw Fault("Unqualified simultaneous SS/SCK transition at 0x0001 mcu-pc=" +
                    hex(control_.context.pc) + " dsp-pc=" + hex(native_->dsp.cpu().state().pc, 6));
      case PadOrder::clock_before_select: {
        auto intermediate = pins;
        intermediate.selected = control_.previous.selected;
        native_->dsp.bus().shi().observe_spi(intermediate);
        break;
      }
      case PadOrder::select_before_clock: {
        auto intermediate = control_.previous;
        intermediate.selected = pins.selected;
        intermediate.mosi = pins.mosi;
        native_->dsp.bus().shi().observe_spi(intermediate);
        break;
      }
      default: throw Fault("Invalid reference SPI pad order");
    }
  }
  native_->dsp.bus().shi().observe_spi(pins);
  update_wiring();
  if (control_.previous.selected && !pins.selected) { ++control_.frames; }
  control_.previous = pins;
}

void Board::on_write(const BusEvent &event) {
  if (!event.write) { return; }
  update_wiring();
  if (execution_.trace != nullptr) {
    *execution_.trace << "WRITE cycle=" << native_->mcu.run_clock().bus_cycles()
                      << " address=" << hex(event.address) << " value=" << hex(event.value, 2)
                      << " mcu-pc=" << hex(control_.context.pc) << '\n';
  }
  if (!native_->schedule && !native_->fetch) { return; }
  constexpr unsigned port_b = 1;
  if (event.address != port_b) {
    check_signals();
    return;
  }
  if (native_->fetch) {
    native_->fetch->observe_write(
        event, [&] { observe_spi_write(event); }, fetch_inputs(), fetch_observer());
  } else {
    native_->schedule->observe_write(
        event,
        [&] { observe_spi_write(event); },
        [this](const auto &step) { before_dsp(step); },
        return_observer(),
        dsp_inputs());
  }
  update_wiring();
}

void Board::observe_spi_write(const BusEvent &event) {
  if (!control_.release_epoch) { return; }
  try {
    transition(sample());
  } catch (const Fault &fault) {
    throw Fault("Reference SPI write at 0x0001 mcu-pc=" + hex(control_.context.pc) +
                " dsp-pc=" + hex(native_->dsp.cpu().state().pc, 6) + ": " + fault.what());
  }
  ++control_.writes;
  if (execution_.trace != nullptr) {
    *execution_.trace << "SPI end=" << schedule_ticks() << " mcu-pc=" << hex(control_.context.pc)
                      << " value=" << hex(event.value, 2) << " frames=" << control_.frames << '\n';
  }
}

}
