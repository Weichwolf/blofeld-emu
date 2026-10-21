// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/usbip_export.hpp"
#include "blofeld/usbip_session.hpp"
#include "blofeld/usbip_tcp.hpp"
#include "usbip_socket.hpp"

#include <cstdint>
#include <memory>
#include <vector>

namespace blofeld {
struct UsbipTcpServer::Impl {
  struct Connection {
    usbip_tcp::Socket socket;
    std::unique_ptr<UsbipSession> session;
    std::uint64_t id = 0;
  };

  explicit Impl(UsbipTcpOptions requested);
  void accept(std::vector<UsbipConnectionEvent> &events);
  [[nodiscard]] static bool
  flush(Connection &connection, std::size_t &budget, std::vector<UsbipConnectionEvent> &events);
  [[nodiscard]] bool receive(Connection &connection,
                             std::vector<UsbipConnectionEvent> &events) const;
  UsbipTcpOptions options;
  usbip_tcp::Socket listener;
  std::shared_ptr<UsbipExport> publication;
  std::vector<Connection> channels;
  std::uint64_t next_id = 1;
  bool faulted = false;
};
}
