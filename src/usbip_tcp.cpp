// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/usbip_tcp.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/usbip_export.hpp"
#include "blofeld/usbip_session.hpp"
#include "usbip_socket.hpp"
#include "usbip_tcp_detail.hpp"

#include <algorithm>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <system_error>
#include <unistd.h>
#include <utility>
#include <vector>

namespace blofeld {
UsbipTcpServer::Impl::Impl(const UsbipTcpOptions requested)
    : options(requested),
      listener(::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0)) {
  constexpr std::size_t maximum_write_budget = std::size_t{64} * 1024;
  if (options.write_budget == 0 || options.write_budget > maximum_write_budget) {
    throw Fault("USB/IP TCP write budget is outside supported bounds");
  }
  if (listener.get() < 0) {
    throw Fault("USB/IP socket: " + std::error_code(errno, std::generic_category()).message());
  }
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(options.port);
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  constexpr int backlog = 4;
  if (::bind(listener.get(), reinterpret_cast<const sockaddr *>(&address), sizeof(address)) != 0 ||
      ::listen(listener.get(), backlog) != 0) {
    throw Fault("USB/IP loopback bind/listen: " +
                std::error_code(errno, std::generic_category()).message());
  }
  socklen_t length = sizeof(address);
  if (::getsockname(listener.get(), reinterpret_cast<sockaddr *>(&address), &length) != 0 ||
      length != sizeof(address)) {
    throw Fault("USB/IP loopback port inspection failed");
  }
  options.port = ntohs(address.sin_port);
}

void UsbipTcpServer::Impl::accept(std::vector<UsbipConnectionEvent> &events) {
  constexpr unsigned maximum_connections = 4;
  constexpr unsigned accept_budget = 4;
  for (unsigned i = 0; i < accept_budget; ++i) {
    usbip_tcp::Socket socket(
        ::accept4(listener.get(), nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC));
    if (socket.get() < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) { return; }
      if (errno == ECONNABORTED || errno == EPROTO) {
        events.push_back(
            {.error = true, .reason = "USB/IP incoming connection aborted before acceptance"});
        continue;
      }
      throw Fault("USB/IP accept: " + std::error_code(errno, std::generic_category()).message());
    }
    if (next_id == std::numeric_limits<std::uint64_t>::max()) {
      throw Fault("USB/IP connection identity exhausted");
    }
    constexpr int enabled = 1;
    if (::setsockopt(socket.get(), IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled)) != 0) {
      throw Fault("USB/IP accepted TCP_NODELAY setup failed");
    }
    const auto id = next_id++;
    if (channels.size() == maximum_connections) {
      events.push_back(
          {.connection = id, .error = true, .reason = "USB/IP connection capacity exhausted"});
    } else {
      channels.push_back(
          {.socket = std::move(socket), .session = std::make_unique<UsbipSession>(), .id = id});
    }
  }
}

UsbipTcpServer::UsbipTcpServer(const UsbipTcpOptions options)
    : impl_(std::make_unique<Impl>(options)) {}

UsbipTcpServer::~UsbipTcpServer() = default;
UsbipTcpServer::UsbipTcpServer(UsbipTcpServer &&) noexcept = default;
UsbipTcpServer &UsbipTcpServer::operator=(UsbipTcpServer &&) noexcept = default;

std::uint16_t UsbipTcpServer::port() const {
  if (!impl_) { throw Fault("Moved USB/IP TCP server has no listener"); }
  return impl_->options.port;
}

std::size_t UsbipTcpServer::connections() const {
  return impl_ ? impl_->channels.size() : 0;
}

void UsbipTcpServer::publish(std::shared_ptr<UsbipExport> publication) {
  if (!impl_ || impl_->faulted || !publication || !publication->available() ||
      std::ranges::any_of(impl_->channels,
                          [](const auto &channel) { return channel.session->imported(); })) {
    throw Fault("USB/IP publication requires an available capture and no imported connection");
  }
  impl_->publication = std::move(publication);
}

void UsbipTcpServer::withdraw() {
  if (!impl_) { return; }
  impl_->channels.clear();
  impl_->publication.reset();
}

std::vector<UsbipConnectionEvent> UsbipTcpServer::poll(McuBus &bus,
                                                       const ExecutionContext context) {
  if (!impl_ || impl_->faulted) {
    throw Fault("Faulted/moved USB/IP TCP server requires a new session");
  }
  std::vector<UsbipConnectionEvent> events;
  try {
    impl_->accept(events);
    for (auto current = impl_->channels.begin(); current != impl_->channels.end();) {
      auto &channel = *current;
      auto budget = impl_->options.write_budget;
      const auto before_events = events.size();
      const bool active = Impl::flush(channel, budget, events) && impl_->receive(channel, events);
      if (active && !channel.session->closed()) { channel.session->poll(bus, context); }
      if (active) { static_cast<void>(Impl::flush(channel, budget, events)); }
      if (channel.session->closed()) {
        if (events.size() == before_events) {
          events.push_back({.connection = channel.id,
                            .requires_board_reset = channel.session->imported(),
                            .reason = "USB/IP connection drained/closed"});
        }
        current = impl_->channels.erase(current);
      } else {
        ++current;
      }
    }
  } catch (const Fault &) {
    impl_->faulted = true;
    withdraw();
    impl_->listener.reset();
    throw;
  }
  return events;
}
}
