// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "mcu/mcu_usb.hpp"
#include "usb/usb_device.hpp"
#include "usb/usb_host.hpp"
#include "usb/usb_transfer.hpp"

#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace blofeld {
class McuBus;

// Host-side control enumeration and bounded USB-MIDI bulk transfers. The caller
// attaches the host link, runs the MCU between polls and consumes IN completions.
class UsbMidiHost {
public:
  UsbMidiHost(std::uint64_t frame_ticks, std::uint64_t timeout_ticks);
  UsbMidiHost(const UsbMidiHost &) = delete;
  UsbMidiHost &operator=(const UsbMidiHost &) = delete;
  UsbMidiHost(UsbMidiHost &&) = delete;
  UsbMidiHost &operator=(UsbMidiHost &&) = delete;
  [[nodiscard]] std::vector<UsbTransferCompletion>
  poll(McuBus &bus, ExecutionContext context, std::uint64_t ticks);
  [[nodiscard]] std::uint32_t send_packets(std::span<const std::uint8_t> packets);

  [[nodiscard]] bool ready() const { return stage_ == Stage::ready && !faulted_; }

  [[nodiscard]] const UsbDeviceDescriptors &descriptors() const { return probe_.descriptors(); }

private:
  enum class Stage : std::uint8_t { descriptors, address, configuration, readback, ready };
  void select_endpoints();
  void submit_control();
  void complete_control(McuBus &bus, const UsbTransferCompletion &completion);
  void validate_configuration(const McuBus &bus) const;
  [[nodiscard]] bool due(const McuBus &bus, std::uint64_t ticks);
  UsbDeviceProbe probe_;
  UsbHostQueue queue_;
  std::uint64_t frame_ticks_;
  std::uint64_t timeout_ticks_;
  std::optional<std::uint64_t> started_;
  std::optional<std::uint64_t> previous_;
  std::optional<std::uint64_t> frame_;
  std::optional<std::uint64_t> addressed_at_;
  UsbHostEvidence evidence_;
  UsbEndpointInfo input_;
  UsbEndpointInfo output_;
  std::uint8_t configuration_ = 0;
  std::uint8_t interface_ = 0;
  std::uint32_t sequence_ = 8;
  Stage stage_ = Stage::descriptors;
  bool submitted_ = false;
  bool receiving_ = false;
  bool faulted_ = false;
};
}
