// SPDX-License-Identifier: GPL-3.0-only
#include "runtime/mcu_dsp_gpio_peer.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "runtime/gpio_pads.hpp"
#include "runtime/mcu_dsp_peer.hpp"

#include <array>
#include <cstdint>
#include <memory>

namespace blofeld {
namespace {
constexpr std::uint8_t data_mask = 0xff;
constexpr unsigned port_c_mask = 0xfff;
constexpr unsigned strobe = 1;
constexpr unsigned notification_bit = 9;
constexpr unsigned notification_pin = 1U << notification_bit;
constexpr unsigned xirq_pin = 1;
}

McuDspGpioPeer::McuDspGpioPeer(McuMachine &mcu, DspMachine &dsp, const McuDspGpioWiring wiring)
    : mcu_(mcu), dsp_(dsp), wiring_(wiring) {
  if (wiring.notification_pull != GpioPeerPull::none &&
      wiring.notification_pull != GpioPeerPull::up &&
      wiring.notification_pull != GpioPeerPull::down) {
    throw Fault("Invalid MCU/DSP GPIO notification pull");
  }
  rebind();
}

bool McuDspGpioPeer::shared_mcu_write(const std::uint16_t address) {
  constexpr unsigned mebi_end = 0x10;
  constexpr unsigned port_j = 0x250;
  constexpr unsigned port_q = 0x260;
  constexpr unsigned port_registers = 8;
  return address < mebi_end || (address >= port_j && address < port_j + port_registers) ||
         (address >= port_q && address < port_q + port_registers);
}

void McuDspGpioPeer::capture_outputs() {
  port_c_ = dsp_.bus().gpio(DspPort::c).outputs();
  drivers_ = mcu_dsp_peer_drivers(data_mask,
                                  mcu_.bus().gpio_outputs(McuPort::q),
                                  mcu_.bus().gpio_biases(McuPort::q),
                                  port_c_,
                                  wiring_.data_pull_down);
}

GpioPadSample McuDspGpioPeer::data_pads(const bool dsp_side, const std::uint32_t mask) const {
  const auto allowed = dsp_side ? port_c_mask : data_mask;
  if ((mask & ~allowed) != 0) { throw Fault("Invalid MCU/DSP GPIO data pad read mask"); }
  auto result = (mask & data_mask) != 0 ? resolve_gpio_net(data_mask, drivers_.mcu, drivers_.dsp)
                                        : GpioPadSample{};
  result.known &= mask;
  result.levels &= mask;
  result.conflicts &= mask;
  if (dsp_side) {
    const auto high = mask & ~static_cast<unsigned>(data_mask);
    // Only the low byte crosses cores. Local control bits must reflect the
    // current register write, including reads nested inside its I/O callback.
    const auto local = dsp_.bus().gpio(DspPort::c).outputs();
    result.known |= local.known & high;
    result.levels |= local.levels & high;
  }
  return result;
}

GpioPadSample McuDspGpioPeer::notification() const {
  const auto source = dsp_.bus().gpio(DspPort::e).outputs();
  const auto target = mcu_.bus().gpio_outputs(McuPort::e);
  const auto bias = mcu_.bus().gpio_biases(McuPort::e);
  const std::array<GpioNetDriver, 3> drivers{
      GpioNetDriver{.active =
                        ((source.mask | source.peripheral) & notification_pin) >> notification_bit,
                    .known = (source.known & notification_pin) >> notification_bit,
                    .levels = (source.levels & notification_pin) >> notification_bit},
      GpioNetDriver{.active = target.mask & xirq_pin,
                    .known = target.known & xirq_pin,
                    .levels = target.levels & xirq_pin,
                    .reduced = target.reduced_drive & xirq_pin,
                    .pull_up = bias.up & xirq_pin,
                    .pull_down = bias.down & xirq_pin},
      GpioNetDriver{.pull_up = wiring_.notification_pull == GpioPeerPull::up ? xirq_pin : 0U,
                    .pull_down = wiring_.notification_pull == GpioPeerPull::down ? xirq_pin : 0U}};
  return resolve_gpio_net(xirq_pin, drivers);
}

void McuDspGpioPeer::deliver_signals() {
  const auto source = mcu_.bus().gpio_outputs(McuPort::j);
  const auto bias = mcu_.bus().gpio_biases(McuPort::j);
  dsp_.bus().observe_irqa_driver({.active = source.mask & strobe,
                                  .known = source.known & strobe,
                                  .levels = source.levels & strobe,
                                  .reduced = source.reduced_drive & strobe,
                                  .pull_up = bias.up & strobe,
                                  .pull_down = bias.down & strobe});
  const auto ready = notification();
  // PE0 is an input. Publish the resolved voltage, including native MCU pulls,
  // rather than manufacturing an interrupt request or an acknowledgement.
  mcu_.bus().update_pin_drive(McuPort::e,
                              xirq_pin,
                              {.mask = static_cast<std::uint8_t>(ready.known),
                               .levels = static_cast<std::uint8_t>(ready.levels),
                               .unknown = static_cast<std::uint8_t>(xirq_pin & ~ready.known)});
}

void McuDspGpioPeer::bind_mcu() {
  const std::weak_ptr<const bool> owner = lifetime_;
  mcu_.bus().attach_gpio_pad_source(McuPort::q, [this, owner](const std::uint32_t mask) {
    if (owner.expired()) { throw Fault("Stale MCU/DSP GPIO pad read at MCU Q"); }
    return data_pads(false, mask);
  });
  mcu_epoch_ = mcu_.run_clock().native_epoch();
}

void McuDspGpioPeer::rebind() {
  capture_outputs();
  bind_mcu();
  const std::weak_ptr<const bool> owner = lifetime_;
  dsp_.bus().gpio(DspPort::c).attach_pad_source([this, owner](const std::uint32_t mask) {
    if (owner.expired()) { throw Fault("Stale MCU/DSP GPIO pad read at DSP C"); }
    return data_pads(true, mask);
  });
  deliver_signals();
}

void McuDspGpioPeer::synchronize() {
  // MCU system reset replaces its bus and drops callbacks; DSP reset retains
  // the GPIO owner and its existing binding. Trusted restores rebind explicitly.
  if (mcu_.run_clock().native_epoch() != mcu_epoch_) { bind_mcu(); }
  capture_outputs();
  deliver_signals();
}
}
