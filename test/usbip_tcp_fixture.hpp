// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "mcu/mcu_bus.hpp"
#include "usb/usbip_socket.hpp"
#include "usb/usbip_tcp.hpp"
#include "usb_host_fixture.hpp"

#include <algorithm>
#include <arpa/inet.h>
#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <span>
#include <sys/socket.h>
#include <vector>

namespace blofeld::test::usbip_tcp {
using Bytes = std::vector<std::uint8_t>;
using Socket = blofeld::usbip_tcp::Socket;
inline constexpr unsigned wait_milliseconds = 1000;

inline Socket connect(const std::uint16_t port) {
  Socket socket(::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0));
  usb_host::require(socket.get() >= 0, "Own loopback client socket opens");
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  const auto result =
      ::connect(socket.get(), reinterpret_cast<const sockaddr *>(&address), sizeof(address));
  usb_host::require(result == 0 || errno == EINPROGRESS,
                    "Own TCP client connects only to the loopback listener");
  pollfd ready{.fd = socket.get(), .events = POLLOUT, .revents = 0};
  usb_host::require(::poll(&ready, 1, wait_milliseconds) == 1 && (ready.revents & POLLOUT) != 0,
                    "The real kernel establishes the bounded loopback connection");
  constexpr int enabled = 1;
  usb_host::require(
      ::setsockopt(socket.get(), IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled)) == 0,
      "Own TCP client disables Nagle for deterministic small-frame tests");
  return socket;
}

inline void send(const Socket &socket, const std::span<const std::uint8_t> bytes) {
  std::size_t offset = 0;
  while (offset < bytes.size()) {
    const auto result = ::send(
        socket.get(), bytes.data() + offset, bytes.size() - offset, MSG_DONTWAIT | MSG_NOSIGNAL);
    if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
      pollfd ready{.fd = socket.get(), .events = POLLOUT, .revents = 0};
      usb_host::require(::poll(&ready, 1, wait_milliseconds) == 1,
                        "Own bounded send becomes writable");
      continue;
    }
    usb_host::require(result > 0, "Own TCP client sends the exact independent wire fixture");
    offset += static_cast<std::size_t>(result);
  }
}

struct Received {
  Bytes bytes;
  std::vector<UsbipConnectionEvent> events;
  bool eof = false;
};

inline void read(const Socket &socket, Received &received) {
  constexpr unsigned batch_bytes = 4096;
  std::array<std::uint8_t, batch_bytes> bytes{};
  for (unsigned attempt = 0; attempt < 16 && !received.eof; ++attempt) {
    const auto count = ::recv(socket.get(), bytes.data(), bytes.size(), MSG_DONTWAIT);
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) { return; }
    usb_host::require(count >= 0, "Own TCP client receives no unexpected kernel error");
    if (count == 0) {
      received.eof = true;
      return;
    }
    const auto data = std::span(bytes).first(static_cast<std::size_t>(count));
    received.bytes.insert(received.bytes.end(), data.begin(), data.end());
  }
}

inline void until(UsbipTcpServer &server,
                  McuBus &bus,
                  const Socket &socket,
                  Received &received,
                  const auto &complete,
                  const ExecutionContext context = usb_host::context) {
  constexpr unsigned maximum_polls = 1000;
  for (unsigned attempt = 0; attempt < maximum_polls; ++attempt) {
    const auto events = server.poll(bus, context);
    received.events.insert(received.events.end(), events.begin(), events.end());
    read(socket, received);
    if (complete()) { return; }
    pollfd ready{.fd = socket.get(), .events = POLLIN, .revents = 0};
    static_cast<void>(::poll(&ready, 1, 1));
  }
  throw Fault("Own USB/IP loopback fixture exceeded its bounded poll budget");
}
}
