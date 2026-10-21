// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/mcu_usb.hpp"
#include "blofeld/usb_device.hpp"
#include "blofeld/usbip_export.hpp"
#include "blofeld/usbip_tcp.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace blofeld {
class McuBus;

struct UsbipRuntimeOptions {
  UsbipTcpOptions tcp;
  UsbipLocation location;
  std::uint64_t after_ticks = 0;
  std::uint64_t period_ticks = 0;
  std::uint64_t probe_timeout_ticks = 0;
};

struct UsbipRuntimeBoundary {
  std::shared_ptr<const void> owner;
  std::uint64_t restore_revision = 0;
  std::uint64_t ticks = 0;
};

// Non-real-time, single-owner adapter. The board runs the actual MCU between
// polls and supplies an explicit reference clock; no guest initialization occurs.
// Reset/state loss retires this whole adapter rather than repairing guest state.
class UsbipRuntime {
public:
  explicit UsbipRuntime(UsbipRuntimeOptions options);
  UsbipRuntime(const UsbipRuntime &) = delete;
  UsbipRuntime &operator=(const UsbipRuntime &) = delete;
  UsbipRuntime(UsbipRuntime &&) = delete;
  UsbipRuntime &operator=(UsbipRuntime &&) = delete;
  [[nodiscard]] std::vector<UsbipConnectionEvent>
  poll(McuBus &bus, ExecutionContext context, const UsbipRuntimeBoundary &boundary);
  void retire();

  [[nodiscard]] std::uint16_t port() const { return port_; }

  [[nodiscard]] bool started() const { return admitted_.has_value(); }

  [[nodiscard]] bool published() const { return publication_ != nullptr; }

  [[nodiscard]] bool faulted() const { return faulted_; }

private:
  void validate_boundary(const UsbipRuntimeBoundary &boundary);
  void validate_evidence(const McuBus &bus) const;
  [[nodiscard]] bool due(std::uint64_t ticks);
  void probe(McuBus &bus, ExecutionContext context, std::uint64_t ticks);
  UsbipRuntimeOptions options_;
  std::optional<UsbipTcpServer> server_;
  UsbDeviceProbe probe_;
  std::shared_ptr<UsbipExport> publication_;
  std::optional<UsbipRuntimeBoundary> previous_;
  std::optional<std::uint64_t> admitted_;
  UsbHostEvidence evidence_;
  UsbConfigurationInfo configuration_;
  std::uint64_t next_ = 0;
  std::uint16_t port_ = 0;
  bool faulted_ = false;
};
}
