// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "runtime/gpio_pads.hpp"
#include "runtime/mcu_dsp_peer.hpp"

#include <cstdint>
#include <memory>

namespace blofeld {
class McuMachine;
class DspMachine;

enum class GpioPeerPull : std::uint8_t { none, up, down };

// Declared functional direct nets, not measured board wiring or edge latency.
struct McuDspGpioWiring {
  std::uint8_t data_pull_down = 0;
  GpioPeerPull notification_pull = GpioPeerPull::none;
};

// Q[7:0]<->C[7:0], J0->IRQA and PE9->PE0/XIRQ. Only native pad values cross
// the connection; no command decoding, byte queues or synthesized replies.
// Both chips must outlive this owner. Rebind after a trusted chip restore.
class McuDspGpioPeer {
public:
  McuDspGpioPeer(McuMachine &mcu, DspMachine &dsp, McuDspGpioWiring wiring);
  McuDspGpioPeer(const McuDspGpioPeer &) = delete;
  McuDspGpioPeer &operator=(const McuDspGpioPeer &) = delete;
  void rebind();
  // Call at the actual common-clock write/effect boundary. Readers use this
  // published wire state, never GPIO values from a later native operation.
  void synchronize();
  [[nodiscard]] static bool shared_mcu_write(std::uint16_t address);

private:
  void bind_mcu();
  void capture_outputs();
  void deliver_signals();
  [[nodiscard]] GpioPadSample data_pads(bool dsp_side, std::uint32_t mask) const;
  [[nodiscard]] GpioPadSample notification() const;
  McuMachine &mcu_;
  DspMachine &dsp_;
  McuDspGpioWiring wiring_;
  const std::shared_ptr<const bool> lifetime_ = std::make_shared<const bool>(true);
  McuDspPeerDrivers drivers_;
  DspGpioOutputs port_c_;
  std::uint64_t mcu_epoch_ = 0;
};
}
