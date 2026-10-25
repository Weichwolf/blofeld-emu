// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_esai_registers.hpp"
#include "dsp/dsp_gpio.hpp"

#include <cstddef>

namespace blofeld {
namespace {
std::size_t index(const DspEsaiInstance instance) {
  if (instance != DspEsaiInstance::primary && instance != DspEsaiInstance::secondary) {
    throw Fault("Invalid DSP ESAI instance");
  }
  return static_cast<std::size_t>(instance);
}

DspPort port(const DspEsaiInstance instance) {
  static_cast<void>(index(instance));
  return instance == DspEsaiInstance::primary ? DspPort::c : DspPort::e;
}

DspEsaiOutputs qualify(const DspGpio &gpio, DspEsaiOutputs output) {
  const auto selected = gpio.outputs().peripheral;
  unsigned pads = 0;
  unsigned levels = 0;
  unsigned lanes = 0;
  constexpr unsigned first_pad = 11;
  for (unsigned lane = 0; lane < 6; ++lane) {
    const auto pad = 1U << (first_pad - lane);
    const auto mask = 1U << lane;
    if ((selected & pad) == 0) { continue; }
    lanes |= mask;
    if ((output.known & mask) != 0) {
      pads |= pad;
      if ((output.levels & mask) != 0) { levels |= pad; }
    }
  }
  gpio.validate_peripheral_drive({.mask = pads, .levels = levels});
  output.driven &= lanes;
  output.known &= lanes;
  output.levels &= lanes;
  return output;
}

DspEsaiClockOutputs qualify_clock(const DspGpio &gpio, DspEsaiClockOutputs output) {
  const auto selected = gpio.outputs().peripheral;
  unsigned lanes = 0;
  unsigned pads = 0;
  unsigned levels = 0;
  constexpr unsigned bit_clock_pad = 3;
  for (unsigned lane = 0; lane < 2; ++lane) {
    const auto pad = 1U << (bit_clock_pad + lane);
    const auto mask = 1U << lane;
    if ((selected & pad) == 0) { continue; }
    lanes |= mask;
    if ((output.known & mask) != 0) {
      pads |= pad;
      if ((output.levels & mask) != 0) { levels |= pad; }
    }
  }
  gpio.validate_peripheral_drive({.mask = pads, .levels = levels});
  output.driven &= lanes;
  output.known &= lanes;
  output.levels &= lanes;
  return output;
}
}

DspEsai &DspBus::sync_esai(const DspEsaiInstance instance) {
  auto &serial = esai_.at(index(instance));
  serial.set_active(gpio(port(instance)).outputs().peripheral != 0);
  return serial;
}

void DspBus::sync_esai_port(const DspPort changed) {
  if (changed == DspPort::c) { static_cast<void>(sync_esai(DspEsaiInstance::primary)); }
  if (changed == DspPort::e) { static_cast<void>(sync_esai(DspEsaiInstance::secondary)); }
}

DspEsai *DspBus::esai_register(const DspAddress access) {
  constexpr unsigned primary_base = 0xffffa0;
  constexpr unsigned secondary_base = 0xffff80;
  const bool primary = access.space == DspSpace::x;
  if (!primary && access.space != DspSpace::y) { return nullptr; }
  const auto base = primary ? primary_base : secondary_base;
  if (access.address < base || access.address - base > esai::rsmb ||
      !DspEsai::handles(access.address - base)) {
    return nullptr;
  }
  return &sync_esai(primary ? DspEsaiInstance::primary : DspEsaiInstance::secondary);
}

void DspBus::observe_esai_transmit(const DspEsaiInstance instance, const DspEsaiInputs inputs) {
  auto &serial = sync_esai(instance);
  const auto &gpio_port = gpio(port(instance));
  const auto selected = gpio_port.outputs().peripheral;
  const auto required = serial.required_clock_pads();
  constexpr unsigned external_pads = 0x18;
  if ((selected & required) != required) {
    if (required != external_pads) {
      throw Fault(
          instance == DspEsaiInstance::primary
              ? "DSP ESAI clock input at X:0xffffbf requires HCKT/FST peripheral mux selection"
              : "DSP ESAI_1 clock input at Y:0xffff9f requires HCKT/FST peripheral mux selection");
    }
    throw Fault(instance == DspEsaiInstance::primary
                    ? "DSP ESAI input at X:0xffffbf requires SCKT/FST peripheral mux selection"
                    : "DSP ESAI_1 input at Y:0xffff9f requires SCKT/FST peripheral mux selection");
  }
  auto next = serial;
  next.observe(inputs);
  static_cast<void>(qualify(gpio_port, next.outputs()));
  static_cast<void>(qualify_clock(gpio_port, next.clock_outputs()));
  serial = next;
}

DspEsaiOutputs DspBus::esai_transmit_outputs(const DspEsaiInstance instance) const {
  const auto &gpio_port = gpio_.at(static_cast<std::size_t>(port(instance)));
  return qualify(gpio_port, esai_.at(index(instance)).outputs());
}

DspEsaiClockOutputs DspBus::esai_clock_outputs(const DspEsaiInstance instance) const {
  const auto &gpio_port = gpio_.at(static_cast<std::size_t>(port(instance)));
  return qualify_clock(gpio_port, esai_.at(index(instance)).clock_outputs());
}
}
