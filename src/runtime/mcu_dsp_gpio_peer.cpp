// SPDX-License-Identifier: GPL-3.0-only
#include "runtime/mcu_dsp_gpio_peer.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "runtime/gpio_pads.hpp"
#include "runtime/mcu_dsp_peer.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>

namespace blofeld {
namespace {
constexpr std::uint8_t data_mask = 0xff;
constexpr unsigned port_c_mask = 0xfff;
constexpr unsigned strobe = 1;
constexpr unsigned notification_bit = 9;
constexpr unsigned notification_pin = 1U << notification_bit;
constexpr unsigned xirq_pin = 1;

bool same_outputs(const DspGpioOutputs first, const DspGpioOutputs second) {
  return std::array{first.mask, first.known, first.levels, first.peripheral} ==
         std::array{second.mask, second.known, second.levels, second.peripheral};
}
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

bool McuDspGpioPeer::outputs_changed() const {
  return mcu_revision_ != mcu_.bus().gpio_output_revision() ||
         !same_outputs(port_c_, dsp_.bus().gpio(DspPort::c).outputs()) ||
         !same_outputs(port_e_, dsp_.bus().gpio(DspPort::e).outputs());
}

void McuDspGpioPeer::capture_outputs() {
  port_c_ = dsp_.bus().gpio(DspPort::c).outputs();
  port_e_ = dsp_.bus().gpio(DspPort::e).outputs();
  drivers_ = mcu_dsp_peer_drivers(data_mask,
                                  mcu_.bus().gpio_outputs(McuPort::q),
                                  mcu_.bus().gpio_biases(McuPort::q),
                                  port_c_,
                                  wiring_.data_pull_down);
  data_ = resolve_gpio_net(data_mask, drivers_.mcu, drivers_.dsp);
  mcu_revision_ = mcu_.bus().gpio_output_revision();
  capture_response();
}

void McuDspGpioPeer::reset_response() {
  produced_ = consumed_ = 0;
  response_active_ = transmitting_ = false;
  strobe_level_.reset();
  dsp_revision_ = dsp_.bus().io_revision();
}

void McuDspGpioPeer::begin_response() {
  constexpr unsigned report_command = 13;
  constexpr unsigned bulk_command = 14;
  constexpr unsigned command_mask = 0x3f;
  const auto command = port_c_.levels & command_mask;
  if ((port_c_.mask & port_c_.known & data_mask) != data_mask ||
      (command != report_command && command != bulk_command)) {
    return;
  }
  if (response_active_ || transmitting_) { throw Fault("Overlapping MCU/DSP response request"); }
  produced_ = consumed_ = 0;
  response_active_ = transmitting_ = true;
}

void McuDspGpioPeer::capture_response() {
  const auto revision = dsp_.bus().io_revision();
  if (revision == dsp_revision_) { return; }
  dsp_revision_ = revision;
  if (!transmitting_) { return; }
  if ((port_c_.mask & data_mask) != data_mask) {
    if (produced_ != response_.size()) { throw Fault("Incomplete DSP response block"); }
    transmitting_ = false;
    return;
  }
  constexpr unsigned port_c_data = 0xffffbd;
  const auto &write = dsp_.bus().io_write();
  if (!write || write->space != DspSpace::x || write->address != port_c_data) { return; }
  if (produced_ == response_.size()) { throw Fault("DSP response buffer overflow"); }
  if ((port_c_.known & data_mask) != data_mask) { throw Fault("Unknown DSP response byte"); }
  response_.at(produced_++) = static_cast<std::uint8_t>(port_c_.levels & data_mask);
}

GpioPadSample McuDspGpioPeer::read_data(const std::uint32_t mask) {
  if (!response_active_) { return data_pads(false, mask); }
  if (drivers_.mcu.active != 0) { throw Fault("MCU response read while driving data at 0x0260"); }
  if (mask != data_mask) { throw Fault("Partial MCU response read at 0x0260"); }
  if (consumed_ == produced_) { throw Fault("MCU/DSP response buffer underrun at 0x0260"); }
  const auto value = response_.at(consumed_++);
  if (consumed_ == response_.size()) { response_active_ = false; }
  return {.known = data_mask, .levels = value};
}

GpioPadSample McuDspGpioPeer::data_pads(const bool dsp_side, const std::uint32_t mask) const {
  const auto allowed = dsp_side ? port_c_mask : data_mask;
  if ((mask & ~allowed) != 0) { throw Fault("Invalid MCU/DSP GPIO data pad read mask"); }
  auto result = data_;
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
  const auto source = port_e_;
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
  const auto level = (source.mask & source.known & strobe) != 0
                         ? std::optional{(source.levels & strobe) != 0}
                         : std::nullopt;
  if (strobe_level_ == true && level == false) { begin_response(); }
  strobe_level_ = level;
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
  mcu_.bus().attach_gpio_pad_source(
      McuPort::q,
      [this, owner](const std::uint32_t mask) {
        if (owner.expired()) { throw Fault("Stale MCU/DSP GPIO pad read at MCU Q"); }
        return data_pads(false, mask);
      },
      [this, owner](const std::uint32_t mask) {
        if (owner.expired()) { throw Fault("Stale MCU/DSP GPIO register read at MCU Q"); }
        return read_data(mask);
      });
  mcu_epoch_ = mcu_.run_clock().native_epoch();
}

void McuDspGpioPeer::rebind() {
  reset_response();
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
  bool rebound = false;
  // MCU system reset replaces its bus and drops callbacks; DSP reset retains
  // the GPIO owner and its existing binding. Trusted restores rebind explicitly.
  if (mcu_.run_clock().native_epoch() != mcu_epoch_) {
    reset_response();
    bind_mcu();
    rebound = true;
  }
  const bool resetting = dsp_.clock().hardware_reset_active();
  if (resetting) { reset_response(); }
  if (!rebound && !resetting && !outputs_changed()) {
    // Repeated PDRC writes are distinct producer bytes even when outputs match.
    capture_response();
    return;
  }
  capture_outputs();
  deliver_signals();
}
}
