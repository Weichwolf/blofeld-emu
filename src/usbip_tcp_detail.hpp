// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/usbip_export.hpp"
#include "blofeld/usbip_session.hpp"
#include "blofeld/usbip_tcp.hpp"
#include "usbip_socket.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace blofeld {
struct UsbipTcpServer::Impl {
  using Time = std::chrono::steady_clock::time_point;

  struct Connection {
    usbip_tcp::Socket socket;
    std::unique_ptr<UsbipSession> session;
    std::uint64_t id = 0;
    std::optional<Time> frame_deadline;
  };

  explicit Impl(UsbipTcpOptions requested);
  void accept(Time now, std::vector<UsbipConnectionEvent> &events);
  [[nodiscard]] static bool
  expire(Connection &connection, Time now, std::vector<UsbipConnectionEvent> &events);
  [[nodiscard]] static bool
  flush(Connection &connection, std::size_t &budget, std::vector<UsbipConnectionEvent> &events);
  [[nodiscard]] bool
  receive(Connection &connection, Time now, std::vector<UsbipConnectionEvent> &events) const;
  UsbipTcpOptions options;
  usbip_tcp::Socket listener;
  std::shared_ptr<UsbipExport> publication;
  std::vector<Connection> channels;
  std::uint64_t next_id = 1;
  Time last_poll = Time::min();
  bool faulted = false;
};
}
