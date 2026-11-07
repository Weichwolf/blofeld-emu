// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dax.hpp"
#include "dsp/dsp_gpio.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
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
  dax_ = next;
}

void DspBus::advance_dax_aci(const std::uint64_t transitions, const DspDaxRates rates) {
  sync_dax();
  dax_.advance_aci(transitions, rates);
}

std::optional<std::uint64_t> DspBus::dax_aci_edges_until_event() const {
  return dax_.aci_edges_until_event();
}
}
