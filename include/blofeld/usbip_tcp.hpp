// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/usbip_export.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace blofeld {
inline constexpr auto default_usbip_frame_timeout = std::chrono::seconds{30};
inline constexpr auto default_usbip_output_timeout = std::chrono::seconds{30};

struct UsbipTcpOptions {
  std::uint16_t port = 0; // Zero asks the kernel for a free loopback port.
  std::size_t write_budget = std::size_t{64} * 1024;
  // Host resource policy, independent of USB/MCU time and pending MIDI reads.
  std::chrono::milliseconds frame_timeout{default_usbip_frame_timeout};
  std::chrono::milliseconds output_timeout{default_usbip_output_timeout};
};

struct UsbipConnectionEvent {
  std::uint64_t connection = 0;
  bool error = false;
  bool requires_board_reset = false;
  std::string reason;
};

// Single-threaded, nonblocking IPv4 loopback transport. Kernel USB controller
// provisioning and MCU execution/reset remain separate caller responsibilities.
class UsbipTcpServer {
public:
  explicit UsbipTcpServer(UsbipTcpOptions options);
  ~UsbipTcpServer();
  UsbipTcpServer(const UsbipTcpServer &) = delete;
  UsbipTcpServer &operator=(const UsbipTcpServer &) = delete;
  UsbipTcpServer(UsbipTcpServer &&) noexcept;
  UsbipTcpServer &operator=(UsbipTcpServer &&) noexcept;
  [[nodiscard]] std::uint16_t port() const;
  void publish(std::shared_ptr<UsbipExport> publication);
  void withdraw();
  [[nodiscard]] std::vector<UsbipConnectionEvent>
  poll(McuBus &bus,
       ExecutionContext context,
       std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now());
  [[nodiscard]] std::size_t connections() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
