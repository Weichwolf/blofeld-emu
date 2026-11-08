// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_shi.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "runtime/gpio_pads.hpp"
#include "runtime/mcu_dsp_gpio_peer.hpp"
#include "vi/vi.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <ostream>
#include <string>

namespace blofeld::vi {
namespace {
bool same(const SpiInputs &first, const SpiInputs &second) {
  return first.selected == second.selected && first.clock == second.clock &&
         first.mosi == second.mosi;
}
}

SpiInputs VirtualInstrument::sample() const {
  const auto mask = static_cast<std::uint8_t>(config_.select_n | config_.clock | config_.mosi);
  const auto pins = vm_.mcu.bus().observe_pads(McuPort::b, mask);
  if (pins.known != mask) {
    throw Fault("Unknown reference SPI pad read at 0x0001 mask=" + hex(mask, 2) +
                " known=" + hex(pins.known, 2) + " conflicts=" + hex(pins.conflicts, 2) +
                " mcu-pc=" + mcu_pc_context() + " dsp-pc=" + hex(vm_.dsp.cpu().registers().pc, 6));
  }
  return {.selected = (pins.levels & config_.select_n) == 0,
          .clock = (pins.levels & config_.clock) != 0,
          .mosi = (pins.levels & config_.mosi) != 0};
}

void VirtualInstrument::check_signals() const {
  if (vm_.release_epoch && !same(sample(), vm_.previous)) {
    throw Fault("Unobserved reference SPI pad change at 0x0001 mcu-pc=" + mcu_pc_context() +
                " dsp-pc=" + hex(vm_.dsp.cpu().registers().pc, 6));
  }
}

void VirtualInstrument::update_loopback() {
  if (!config_.loopback) { return; }
  const auto &route = *config_.loopback;
  auto &projection = vm_.loopback_projection;
  const auto epoch = vm_.mcu.run_clock().native_epoch();
  const auto revision = vm_.mcu.bus().gpio_observation_revision(route.source);
  if (!revision || projection.epoch != epoch || projection.revision != revision) {
    const auto input = vm_.mcu.bus().observe_pads(route.source, route.source_mask);
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
  vm_.mcu.bus().update_pin_drive(route.destination, route.destination_mask, projection.drive);
}

void VirtualInstrument::transition(const SpiInputs pins) {
  if (vm_.previous.selected != pins.selected && vm_.previous.clock != pins.clock) {
    auto intermediate = pins;
    intermediate.selected = vm_.previous.selected;
    vm_.dsp.bus().shi().observe_spi(intermediate);
  }
  vm_.dsp.bus().shi().observe_spi(pins);
  publish_io();
  vm_.previous = pins;
}

void VirtualInstrument::on_write(const BusEvent &event) {
  if (!event.write) { return; }
  publish_io();
  observe_lcd(event.address);
  if (execution_.trace != nullptr) {
    *execution_.trace << "WRITE cycle=" << vm_.mcu.run_clock().bus_cycles()
                      << " address=" << hex(event.address) << " value=" << hex(event.value, 2)
                      << " mcu-pc=" << hex(vm_.context.pc) << '\n';
  }
  constexpr unsigned port_b = 1;
  if (event.address != port_b && !u0_release_write(event.address) &&
      !gpio_peer_write(event.address) && !b_direction_write(event.address)) {
    check_signals();
    return;
  }
  vm_.runtime->observe_write(
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
      vm_.dma_inputs,
      vm_.dsp_events);
  publish_io();
}

void VirtualInstrument::observe_spi_write(const BusEvent &event) {
  if (!vm_.release_epoch) { return; }
  try {
    transition(sample());
  } catch (const Fault &fault) {
    throw Fault("Reference SPI write at 0x0001 mcu-pc=" + hex(vm_.context.pc) +
                " dsp-pc=" + hex(vm_.dsp.cpu().registers().pc, 6) + ": " + fault.what());
  }
  if (execution_.trace != nullptr) {
    *execution_.trace << "SPI end=" << schedule_ticks() << " mcu-pc=" << hex(vm_.context.pc)
                      << " value=" << hex(event.value, 2) << '\n';
  }
}

bool VirtualInstrument::gpio_peer_write(const unsigned address, const unsigned size) const {
  if (!config_.gpio_peer) { return false; }
  constexpr unsigned last = std::numeric_limits<std::uint16_t>::max();
  for (unsigned offset = 0; offset < size && address <= last && offset <= last - address;
       ++offset) {
    if (McuDspGpioPeer::shared_mcu_write(static_cast<std::uint16_t>(address + offset))) {
      return true;
    }
  }
  return false;
}

void VirtualInstrument::publish_io() {
  update_u0_drive();
  observe_lcd_epoch();
  update_loopback();
  if (vm_.gpio_peer) { vm_.gpio_peer->synchronize(); }
}

namespace {
constexpr unsigned reset_mask = 1;
constexpr unsigned u_data = 0x278;
constexpr std::array u_registers{u_data, 0x27aU, 0x27bU, 0x27cU, 0x27dU};

}

bool VirtualInstrument::u0_release_write(const unsigned address, const unsigned size) const {
  return config_.mcu_u0_release && std::ranges::any_of(u_registers, [&](const unsigned reg) {
           return address <= reg && reg - address < size;
         });
}

void VirtualInstrument::update_u0_drive() {
  if (!config_.mcu_u0_release) { return; }
  auto &projection = vm_.u0_projection;
  const auto epoch = vm_.mcu.run_clock().native_epoch();
  const auto revision = vm_.mcu.bus().gpio_observation_revision(McuPort::u);
  if (!revision || projection.epoch != epoch || projection.revision != revision) {
    const auto output = vm_.mcu.bus().gpio_outputs(McuPort::u);
    const auto bias = vm_.mcu.bus().gpio_biases(McuPort::u);
    const auto resolved = resolve_gpio_net(reset_mask,
                                           {.active = output.mask & reset_mask,
                                            .known = output.known & reset_mask,
                                            .levels = output.levels & reset_mask,
                                            .reduced = output.reduced_drive & reset_mask,
                                            .pull_up = bias.up & reset_mask,
                                            .pull_down = bias.down & reset_mask},
                                           {.pull_up = reset_mask});
    // Logical passive-source projection: drive the released input only. An active
    // MCU output remains the sole strong source; unknown local latches stay unknown.
    projection.drive = {.floating = reset_mask};
    if ((output.mask & reset_mask) == 0) {
      projection.drive =
          resolved.known == reset_mask
              ? PinDrive{.mask = reset_mask, .levels = static_cast<std::uint8_t>(resolved.levels)}
              : PinDrive{.unknown = reset_mask};
    }
    projection.epoch = epoch;
    projection.revision = revision;
  }
  vm_.mcu.bus().update_pin_drive(McuPort::u, reset_mask, projection.drive);
}

void VirtualInstrument::observe_u0_release() {
  update_u0_drive();
  const auto sample = vm_.mcu.bus().observe_pads(McuPort::u, reset_mask);
  const auto context = [&] {
    return " at=" + hex(u_data) + " mcu-pc=" + mcu_pc_context() +
           " dsp-pc=" + hex(vm_.dsp.cpu().registers().pc, 6) +
           " tick=" + std::to_string(schedule_ticks());
  };
  if (sample.known != reset_mask) {
    throw Fault("Unknown MCU U0 DSP reset input operation=read" + context());
  }
  if ((sample.levels & reset_mask) == 0) {
    if (vm_.release_epoch) {
      if (!config_.dsp.initial_reset) {
        throw Fault("Missing MCU U0 DSP reset inputs operation=assert" + context());
      }
      try {
        synchronize_audio_clock(schedule_ticks());
        vm_.runtime->begin_dsp_reset(config_.dsp.initial_reset.value());
        if (vm_.pcm) { vm_.pcm->reset(); }
      } catch (const Fault &error) {
        throw Fault("MCU U0 DSP reset operation=assert" + context() + ": " + error.what());
      }
      vm_.release_epoch.reset();
      vm_.u0_asserted = false;
    }
    if (!vm_.u0_asserted && execution_.trace != nullptr) {
      *execution_.trace << "DSP-U0-ASSERT" << context() << '\n';
    }
    vm_.u0_asserted = true;
  } else if (vm_.u0_asserted && !vm_.release_epoch) {
    release(schedule_ticks());
  }
}

void VirtualInstrument::set_b_drive(const PinDrive drive) {
  if ((drive.mask & drive.floating) != 0 || (drive.levels & ~drive.mask) != 0 ||
      (drive.unknown & (drive.mask | drive.floating)) != 0) {
    throw Fault("Invalid external source for Port B passive network");
  }
  vm_.b_external = drive;
}

GpioPadSample VirtualInstrument::sample_b_pulls(const VmContext &owners,
                                                const std::uint32_t mask) const {
  if (!config_.port_b_pulls) { throw Fault("Missing Port B passive network"); }
  const auto output = owners.mcu.bus().gpio_outputs(McuPort::b);
  const auto bias = owners.mcu.bus().gpio_biases(McuPort::b);
  const auto pulls = config_.port_b_pulls.value();
  const auto external = vm_.b_external;
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

void VirtualInstrument::attach_b_pulls(VmContext &owners) {
  if (!config_.port_b_pulls) { return; }
  owners.mcu.bus().attach_gpio_pad_source(
      McuPort::b, [this, &owners](const auto mask) { return sample_b_pulls(owners, mask); });
}

bool VirtualInstrument::b_direction_write(const unsigned address, const unsigned size) const {
  return config_.port_b_pulls && address <= hc12::ddrb && hc12::ddrb - address < size;
}

void VirtualInstrument::observe_b_direction() {
  if (!vm_.release_epoch) { return; }
  // A declared resolved net supplies actual post-write levels. DDR changes
  // therefore use the same ordered SPI edges as data-latch writes; they are
  // not counted as PORTB writes and cannot supply a receive word themselves.
  transition(sample());
}
}
