// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_dax.hpp"
#include "blofeld/dsp_gpio.hpp"

namespace blofeld {
namespace {
DspDaxOutput qualify(const DspGpio &gpio, DspDaxOutput output) {
  constexpr unsigned ado = 2;
  if ((gpio.outputs().peripheral & ado) == 0) { return {}; }
  if (output.driven) {
    gpio.validate_peripheral_drive({.mask = ado, .levels = output.level ? ado : 0U});
  }
  return output;
}
}

void DspBus::sync_dax() {
  dax_.set_active(gpio(DspPort::d).outputs().peripheral != 0);
}

void DspBus::observe_dax(const DspDaxInputs inputs) {
  sync_dax();
  const auto &port = gpio(DspPort::d);
  constexpr unsigned aci = 1;
  if (dax_.external_clock() && port.outputs().peripheral != 0 &&
      (port.outputs().peripheral & aci) == 0) {
    throw Fault("DSP DAX ACI observation requires Port D peripheral input at X:0xffffd7");
  }
  auto next = dax_;
  next.observe(inputs);
  static_cast<void>(qualify(port, next.output()));
  dax_ = next;
}

DspDaxOutput DspBus::dax_output() const {
  return qualify(gpio_.at(static_cast<unsigned>(DspPort::d)), dax_.output());
}
}
