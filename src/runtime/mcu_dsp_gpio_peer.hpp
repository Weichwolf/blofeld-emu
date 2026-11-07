// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "runtime/gpio_pads.hpp"
#include "runtime/mcu_dsp_peer.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>

namespace blofeld {
class McuMachine;
class DspMachine;

enum class GpioPeerPull : std::uint8_t { none, up, down };

// Declared functional direct nets, not measured board wiring or edge latency.
struct McuDspGpioWiring {
  std::uint8_t data_pull_down = 0;
  GpioPeerPull notification_pull = GpioPeerPull::none;
};

// Q/C data registers, J0 requests and E9/XIRQ readiness. A requested report/bulk
// block retains the sixteen bytes actually written by the DSP until MCU reads.
// Both chips must outlive this owner; no response contents are synthesized.
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
  [[nodiscard]] bool outputs_changed() const;
  void deliver_signals();
  void begin_response();
  void capture_response();
  void reset_response();
  [[nodiscard]] GpioPadSample read_data(std::uint32_t mask);
  [[nodiscard]] GpioPadSample data_pads(bool dsp_side, std::uint32_t mask) const;
  [[nodiscard]] GpioPadSample notification() const;
  McuMachine &mcu_;
  DspMachine &dsp_;
  McuDspGpioWiring wiring_;
  const std::shared_ptr<const bool> lifetime_ = std::make_shared<const bool>(true);
  McuDspPeerDrivers drivers_;
  DspGpioOutputs port_c_;
  DspGpioOutputs port_e_;
  GpioPadSample data_;
  std::uint64_t mcu_revision_ = 0;
  std::uint64_t mcu_epoch_ = 0;
  std::uint64_t dsp_revision_ = 0;
  std::optional<bool> strobe_level_;
  std::array<std::uint8_t, 16> response_{};
  unsigned produced_ = 0;
  unsigned consumed_ = 0;
  bool response_active_ = false;
  bool transmitting_ = false;
};
}
