// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_shi.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "reference_board.hpp"

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
  const auto pins = mcu_.bus().observe_pads(McuPort::b, mask);
  if (pins.known != mask) {
    throw Fault("Unknown reference SPI pad read at 0x0001 mask=" + hex(mask, 2) +
                " known=" + hex(pins.known, 2) + " mcu-pc=" + hex(mcu_.cpu().state().pc) +
                " dsp-pc=" + hex(dsp_.cpu().state().pc, 6));
  }
  return {.selected = (pins.levels & config_.select_n) == 0,
          .clock = (pins.levels & config_.clock) != 0,
          .mosi = (pins.levels & config_.mosi) != 0};
}

void Board::check_signals() const {
  if (schedule_ && !same(sample(), previous_)) {
    throw Fault("Unobserved reference SPI pad change at 0x0001 mcu-pc=" +
                hex(mcu_.cpu().state().pc) + " dsp-pc=" + hex(dsp_.cpu().state().pc, 6));
  }
}

void Board::update_loopback() {
  if (!config_.loopback) { return; }
  const auto &route = *config_.loopback;
  const auto input = mcu_.bus().observe_pads(route.source, route.source_mask);
  PinDrive output;
  if (input.known == route.source_mask) {
    output.mask = route.destination_mask;
    output.levels = (input.levels & route.source_mask) != 0 ? route.destination_mask : 0;
  }
  // An unknown pad does not establish a high-impedance source. Preserve
  // unknownness rather than allowing a receiver pull to invent a known level.
  mcu_.bus().update_pin_drive(route.destination, route.destination_mask, output);
}

void Board::transition(const SpiInputs pins) {
  if (previous_.selected != pins.selected && previous_.clock != pins.clock) {
    ++ordered_pairs_;
    switch (config_.pad_order) {
      case PadOrder::reject:
        throw Fault("Unqualified simultaneous SS/SCK transition at 0x0001 mcu-pc=" +
                    hex(context_.pc) + " dsp-pc=" + hex(dsp_.cpu().state().pc, 6));
      case PadOrder::clock_before_select: {
        auto intermediate = pins;
        intermediate.selected = previous_.selected;
        dsp_.bus().shi().observe_spi(intermediate);
        break;
      }
      case PadOrder::select_before_clock: {
        auto intermediate = previous_;
        intermediate.selected = pins.selected;
        intermediate.mosi = pins.mosi;
        dsp_.bus().shi().observe_spi(intermediate);
        break;
      }
      default: throw Fault("Invalid reference SPI pad order");
    }
  }
  dsp_.bus().shi().observe_spi(pins);
  if (previous_.selected && !pins.selected) { ++frames_; }
  previous_ = pins;
}

void Board::on_write(const BusEvent &event) {
  if (!event.write) { return; }
  update_loopback();
  if (execution_.trace != nullptr) {
    *execution_.trace << "WRITE cycle=" << mcu_.run_clock().bus_cycles()
                      << " address=" << hex(event.address) << " value=" << hex(event.value, 2)
                      << " mcu-pc=" << hex(context_.pc) << '\n';
  }
  if (!schedule_) { return; }
  constexpr unsigned port_b = 1;
  if (event.address != port_b) {
    check_signals();
    return;
  }
  schedule_->observe_write(
      event,
      [&] {
        try {
          transition(sample());
        } catch (const Fault &fault) {
          throw Fault("Reference SPI write at 0x0001 mcu-pc=" + hex(context_.pc) +
                      " dsp-pc=" + hex(dsp_.cpu().state().pc, 6) + ": " + fault.what());
        }
        ++writes_;
        if (execution_.trace != nullptr) {
          *execution_.trace << "SPI end=" << schedule_->end_ticks()
                            << " mcu-pc=" << hex(context_.pc) << " value=" << hex(event.value, 2)
                            << " frames=" << frames_ << '\n';
        }
      },
      [this](const auto &step) { before_dsp(step); });
}
}
