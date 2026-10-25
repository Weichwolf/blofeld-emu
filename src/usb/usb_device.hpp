// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "usb/usb_host.hpp"
#include "usb/usb_transfer.hpp"

#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace blofeld {
class McuBus;

struct UsbInterfaceDescription {
  std::uint8_t number = 0;
  std::uint8_t alternate = 0;
  std::uint8_t device_class = 0;
  std::uint8_t subclass = 0;
  std::uint8_t protocol = 0;
  bool operator==(const UsbInterfaceDescription &) const = default;
};

// Only UsbDeviceProbe can construct this immutable descriptor capture. Configuration
// descriptors describe capabilities; they do not establish current configuration/alternates.
class UsbDeviceDescriptors {
public:
  [[nodiscard]] std::span<const std::uint8_t> device() const { return device_; }

  [[nodiscard]] std::span<const std::uint8_t> configuration() const { return configuration_; }

  [[nodiscard]] std::span<const UsbInterfaceDescription> alternatives() const {
    return alternatives_;
  }

  [[nodiscard]] std::uint8_t interface_count() const { return configuration_.at(4); }

private:
  friend class UsbDeviceProbe;
  UsbDeviceDescriptors(std::vector<std::uint8_t> device, std::vector<std::uint8_t> configuration);
  std::vector<std::uint8_t> device_;
  std::vector<std::uint8_t> configuration_;
  std::vector<UsbInterfaceDescription> alternatives_;
};

// The caller runs the actual MCU between polls. No guest registers, readiness,
// address/configuration or control status are supplied by this probe.
class UsbDeviceProbe {
public:
  [[nodiscard]] bool poll(McuBus &bus, ExecutionContext context);

  [[nodiscard]] bool ready() const { return descriptors_.has_value(); }

  [[nodiscard]] bool faulted() const { return faulted_; }

  [[nodiscard]] const UsbDeviceDescriptors &descriptors() const;
  // Discard host capture/queue state only. The board separately owns USB reset.
  void reset();

private:
  enum class Stage : std::uint8_t { device, header, configuration, ready };
  [[nodiscard]] UsbTransferRequest request() const;
  void complete(const UsbTransferCompletion &completion, unsigned maximum_packet);
  UsbHostQueue queue_;
  Stage stage_ = Stage::device;
  bool submitted_ = false;
  bool faulted_ = false;
  std::vector<std::uint8_t> device_;
  std::vector<std::uint8_t> header_;
  std::optional<UsbDeviceDescriptors> descriptors_;
};
}
