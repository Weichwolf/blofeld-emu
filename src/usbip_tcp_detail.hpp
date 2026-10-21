// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/usbip_export.hpp"
#include "blofeld/usbip_session.hpp"
#include "blofeld/usbip_tcp.hpp"
#include "usbip_socket.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace blofeld {
namespace usbip_tcp {
using Time = std::chrono::steady_clock::time_point;

struct Connection {
  usbip_tcp::Socket socket;
  std::unique_ptr<UsbipSession> session;
  std::uint64_t id = 0;
  std::optional<Time> frame_deadline;
  std::optional<Time> output_deadline;
};

[[nodiscard]] bool
expire(Connection &connection, Time now, std::vector<UsbipConnectionEvent> &events);
// The owner validates monotonic time and representable positive deadlines.
[[nodiscard]] bool flush(Connection &connection,
                         std::size_t &budget,
                         Time now,
                         std::chrono::milliseconds timeout,
                         std::vector<UsbipConnectionEvent> &events);
}

struct UsbipTcpServer::Impl {
  using Time = usbip_tcp::Time;
  using Connection = usbip_tcp::Connection;

  explicit Impl(UsbipTcpOptions requested);
  void accept(Time now, std::vector<UsbipConnectionEvent> &events);
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
