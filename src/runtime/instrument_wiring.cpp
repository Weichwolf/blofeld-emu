// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_shi.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "runtime/instrument.hpp"
#include "runtime/instrument_context.hpp"

#include <cstdint>
#include <ostream>

namespace blofeld::runtime {
namespace {
bool same(const SpiInputs &first, const SpiInputs &second) {
  return first.selected == second.selected && first.clock == second.clock &&
         first.mosi == second.mosi;
}
}

SpiInputs Instrument::sample() const {
  const auto mask = static_cast<std::uint8_t>(config_.select_n | config_.clock | config_.mosi);
  const auto pins = native_->mcu.bus().observe_pads(McuPort::b, mask);
  if (pins.known != mask) {
    throw Fault("Unknown reference SPI pad read at 0x0001 mask=" + hex(mask, 2) +
                " known=" + hex(pins.known, 2) + " conflicts=" + hex(pins.conflicts, 2) +
                " mcu-pc=" + mcu_pc_context(native_->mcu) +
                " dsp-pc=" + hex(native_->dsp.cpu().registers().pc, 6));
  }
  return {.selected = (pins.levels & config_.select_n) == 0,
          .clock = (pins.levels & config_.clock) != 0,
          .mosi = (pins.levels & config_.mosi) != 0};
}

void Instrument::check_signals() const {
  if (control_.release_epoch && !same(sample(), control_.previous)) {
    throw Fault(
        "Unobserved reference SPI pad change at 0x0001 mcu-pc=" + mcu_pc_context(native_->mcu) +
        " dsp-pc=" + hex(native_->dsp.cpu().registers().pc, 6));
  }
}

void Instrument::update_loopback() {
  if (!config_.loopback) { return; }
  const auto &route = *config_.loopback;
  auto &projection = control_.loopback_projection;
  const auto epoch = native_->mcu.run_clock().native_epoch();
  const auto revision = native_->mcu.bus().gpio_observation_revision(route.source);
  if (!revision || projection.epoch != epoch || projection.revision != revision) {
    const auto input = native_->mcu.bus().observe_pads(route.source, route.source_mask);
    projection.drive = {};
    if (input.known == route.source_mask) {
      projection.drive.mask = route.destination_mask;
      projection.drive.levels =
          (input.levels & route.source_mask) != 0 ? route.destination_mask : 0;
    }
    // Retain the pre-write token: feedback must expose any newly opposed input
    // on the next observation. Unversioned external readers are sampled every time.
    projection.revision = revision;
    projection.epoch = epoch;
  }
  native_->mcu.bus().update_pin_drive(route.destination, route.destination_mask, projection.drive);
}

void Instrument::transition(const SpiInputs pins) {
  if (control_.previous.selected != pins.selected && control_.previous.clock != pins.clock) {
    auto intermediate = pins;
    intermediate.selected = control_.previous.selected;
    native_->dsp.bus().shi().observe_spi(intermediate);
  }
  native_->dsp.bus().shi().observe_spi(pins);
  publish_io();
  control_.previous = pins;
}

void Instrument::on_write(const BusEvent &event) {
  if (!event.write) { return; }
  publish_io();
  observe_lcd(event.address);
  if (execution_.trace != nullptr) {
    *execution_.trace << "WRITE cycle=" << native_->mcu.run_clock().bus_cycles()
                      << " address=" << hex(event.address) << " value=" << hex(event.value, 2)
                      << " mcu-pc=" << hex(control_.context.pc) << '\n';
  }
  constexpr unsigned port_b = 1;
  if (event.address != port_b && !u0_release_write(event.address) &&
      !gpio_peer_write(event.address) && !b_direction_write(event.address)) {
    check_signals();
    return;
  }
  native_->runtime->observe_write(
      event,
      [&] {
        if (event.address == port_b) {
          observe_spi_write(event);
        } else if (b_direction_write(event.address)) {
          observe_b_direction();
        } else if (u0_release_write(event.address)) {
          observe_u0_release();
        }
      },
      dma_inputs_,
      dsp_events_);
  publish_io();
}

void Instrument::observe_spi_write(const BusEvent &event) {
  if (!control_.release_epoch) { return; }
  try {
    transition(sample());
  } catch (const Fault &fault) {
    throw Fault("Reference SPI write at 0x0001 mcu-pc=" + hex(control_.context.pc) +
                " dsp-pc=" + hex(native_->dsp.cpu().registers().pc, 6) + ": " + fault.what());
  }
  if (execution_.trace != nullptr) {
    *execution_.trace << "SPI end=" << schedule_ticks() << " mcu-pc=" << hex(control_.context.pc)
                      << " value=" << hex(event.value, 2) << '\n';
  }
}

}
