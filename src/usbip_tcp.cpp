// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/usbip_tcp.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/usbip_export.hpp"
#include "blofeld/usbip_session.hpp"
#include "usbip_socket.hpp"
#include "usbip_tcp_detail.hpp"

#include <algorithm>
#include <asm-generic/socket.h>
#include <cerrno>
#include <chrono>
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
  constexpr auto maximum_timeout = std::chrono::hours{24};
  if (options.frame_timeout <= std::chrono::milliseconds::zero() ||
      options.frame_timeout > maximum_timeout) {
    throw Fault("USB/IP TCP frame timeout must be positive and at most 24 hours");
  }
  if (options.output_timeout <= std::chrono::milliseconds::zero() ||
      options.output_timeout > maximum_timeout) {
    throw Fault("USB/IP TCP output timeout must be positive and at most 24 hours");
  }
  if (listener.get() < 0) {
    throw Fault("USB/IP socket: " + std::error_code(errno, std::generic_category()).message());
  }
  // Reuse an address retained by a closed connection, while an active listener
  // still owns its port exclusively. SO_REUSEPORT is deliberately not enabled.
  constexpr int reuse_address = 1;
  if (::setsockopt(
          listener.get(), SOL_SOCKET, SO_REUSEADDR, &reuse_address, sizeof(reuse_address)) != 0) {
    throw Fault("USB/IP listener SO_REUSEADDR setup: " +
                std::error_code(errno, std::generic_category()).message());
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

void UsbipTcpServer::Impl::accept(const Time now, std::vector<UsbipConnectionEvent> &events) {
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
      auto session = std::make_unique<UsbipSession>();
      auto &channel = channels.emplace_back();
      channel.session = std::move(session);
      channel.socket = std::move(socket);
      channel.id = id;
      channel.frame_deadline = now + options.frame_timeout;
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

std::vector<UsbipConnectionEvent> UsbipTcpServer::poll(
    McuBus &bus, const ExecutionContext context, const std::chrono::steady_clock::time_point now) {
  if (!impl_ || impl_->faulted) {
    throw Fault("Faulted/moved USB/IP TCP server requires a new session");
  }
  std::vector<UsbipConnectionEvent> events;
  try {
    const auto timeout = std::max(impl_->options.frame_timeout, impl_->options.output_timeout);
    if (now < impl_->last_poll || now > Impl::Time::max() - timeout) {
      throw Fault("USB/IP host clock moved backwards or cannot represent a frame deadline or "
                  "output deadline");
    }
    impl_->last_poll = now;
    impl_->accept(now, events);
    for (auto current = impl_->channels.begin(); current != impl_->channels.end();) {
      auto &channel = *current;
      auto budget = impl_->options.write_budget;
      const auto before_events = events.size();
      const bool active =
          usbip_tcp::expire(channel, now, events) &&
          usbip_tcp::flush(channel, budget, now, impl_->options.output_timeout, events) &&
          impl_->receive(channel, now, events);
      if (active && !channel.session->closed() &&
          (!channel.session->imported() || !bus.usb_host_suspended())) {
        channel.session->poll(bus, context);
      }
      if (active) {
        static_cast<void>(
            usbip_tcp::flush(channel, budget, now, impl_->options.output_timeout, events));
      }
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
