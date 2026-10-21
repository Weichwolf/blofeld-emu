// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/usbip_session.hpp"
#include "blofeld/usbip_tcp.hpp"
#include "usbip_socket.hpp"
#include "usbip_tcp_detail.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <span>
#include <sys/socket.h>
#include <system_error>
#include <vector>

namespace blofeld {
bool UsbipTcpServer::Impl::flush(Connection &connection,
                                 std::size_t &budget,
                                 std::vector<UsbipConnectionEvent> &events) {
  constexpr unsigned syscall_budget = 16;
  for (unsigned attempt = 0;
       attempt < syscall_budget && budget != 0 && !connection.session->output().empty();
       ++attempt) {
    const auto output =
        connection.session->output().first(std::min(budget, connection.session->output().size()));
    const auto sent =
        ::send(connection.socket.get(), output.data(), output.size(), MSG_DONTWAIT | MSG_NOSIGNAL);
    if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) { return true; }
    if (sent <= 0) {
      events.push_back(
          {.connection = connection.id,
           .error = true,
           .requires_board_reset = connection.session->imported(),
           .reason = "USB/IP send: " + std::error_code(errno, std::generic_category()).message()});
      connection.session->abort();
      return false;
    }
    const auto count = static_cast<std::size_t>(sent);
    connection.session->consume_output(count);
    budget -= count;
  }
  return true;
}

bool UsbipTcpServer::Impl::receive(Connection &connection,
                                   std::vector<UsbipConnectionEvent> &events) const {
  if (!connection.session->wants_input()) { return true; }
  std::array<std::uint8_t, usbip_input_chunk> bytes{};
  const auto received = ::recv(connection.socket.get(), bytes.data(), bytes.size(), MSG_DONTWAIT);
  if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) { return true; }
  if (received < 0) {
    events.push_back(
        {.connection = connection.id,
         .error = true,
         .requires_board_reset = connection.session->imported(),
         .reason = "USB/IP receive: " + std::error_code(errno, std::generic_category()).message()});
    connection.session->abort();
    return false;
  }
  try {
    if (received == 0) {
      connection.session->end_input();
    } else {
      connection.session->receive(std::span(bytes).first(static_cast<std::size_t>(received)),
                                  publication);
    }
    return true;
  } catch (const Fault &fault) {
    events.push_back({.connection = connection.id,
                      .error = true,
                      .requires_board_reset = connection.session->imported(),
                      .reason = fault.what()});
    connection.session->abort();
    return false;
  }
}
}
