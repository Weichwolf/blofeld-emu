// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "board/gpio_pads.hpp"
#include "dsp/dsp_gpio.hpp"
#include "mcu/mcu_gpio.hpp"

#include <cstdint>

namespace blofeld {
struct McuDspPeerDrivers {
  GpioNetDriver mcu;
  GpioNetDriver dsp;
};

// One-to-one candidate nets only. Pull-downs are explicit board hypotheses;
// this conversion does not establish that a physical route exists.
[[nodiscard]] inline McuDspPeerDrivers mcu_dsp_peer_drivers(const std::uint8_t mask,
                                                            const PinOutputs mcu,
                                                            const PinBiases bias,
                                                            const DspGpioOutputs dsp,
                                                            const std::uint8_t dsp_idle_pull_down) {
  const std::uint32_t selected = mask;
  const auto active = (dsp.mask | dsp.peripheral) & selected;
  return {.mcu = {.active = mcu.mask & selected,
                  .known = mcu.known & selected,
                  .levels = mcu.levels & selected,
                  .reduced = mcu.reduced_drive & selected,
                  .pull_up = bias.up & selected,
                  .pull_down = bias.down & selected},
          .dsp = {.active = active,
                  .known = dsp.known & selected,
                  .levels = dsp.levels & selected,
                  .pull_down = dsp_idle_pull_down & selected & ~active}};
}
}
