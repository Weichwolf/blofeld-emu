// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "runtime/gpio_pads.hpp"
#include "runtime/instrument.hpp"

#include <array>
#include <cstdint>

namespace blofeld::runtime {

void Instrument::set_b_drive(const PinDrive drive) {
  if ((drive.mask & drive.floating) != 0 || (drive.levels & ~drive.mask) != 0 ||
      (drive.unknown & (drive.mask | drive.floating)) != 0) {
    throw Fault("Invalid external source for Port B passive network");
  }
  control_.b_external = drive;
}

GpioPadSample Instrument::sample_b_pulls(const Owners &owners, const std::uint32_t mask) const {
  if (!config_.port_b_pulls) { throw Fault("Missing Port B passive network"); }
  const auto output = owners.mcu.bus().gpio_outputs(McuPort::b);
  const auto bias = owners.mcu.bus().gpio_biases(McuPort::b);
  const auto pulls = config_.port_b_pulls.value();
  const auto external = control_.b_external;
  const std::array drivers{
      GpioNetDriver{.active = output.mask & mask,
                    .known = output.known & mask,
                    .levels = output.levels & mask,
                    .reduced = output.reduced_drive & mask,
                    .pull_up = bias.up & mask,
                    .pull_down = bias.down & mask},
      GpioNetDriver{.active = (external.mask | external.unknown) & mask,
                    .known = external.mask & mask,
                    .levels = external.levels & mask},
      GpioNetDriver{.pull_up = pulls.up & mask, .pull_down = pulls.down & mask}};
  return resolve_gpio_net(mask, drivers);
}

void Instrument::attach_b_pulls(Owners &owners) {
  if (!config_.port_b_pulls) { return; }
  owners.mcu.bus().attach_gpio_pad_source(
      McuPort::b, [this, &owners](const auto mask) { return sample_b_pulls(owners, mask); });
}

bool Instrument::b_direction_write(const unsigned address, const unsigned size) const {
  return config_.port_b_pulls && address <= hc12::ddrb && hc12::ddrb - address < size;
}

void Instrument::observe_b_direction() {
  if (!control_.release_epoch) { return; }
  // A declared resolved net supplies actual post-write levels. DDR changes
  // therefore use the same ordered SPI edges as data-latch writes; they are
  // not counted as PORTB writes and cannot supply a receive word themselves.
  transition(sample());
}
}
