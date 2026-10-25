// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_usb.hpp"
#include "usb/usbip_export.hpp"
#include "usb/usbip_session.hpp"
#include "usb/usbip_tcp.hpp"
#include "usb/usbip_tcp_detail.hpp"
#include "usb_device_fixture.hpp"
#include "usb_device_native_fixture.hpp"
#include "usb_host_fixture.hpp"
#include "usbip_fixture.hpp"
#include "usbip_session_fixture.hpp"
#include "usbip_tcp_fixture.hpp"

#include <algorithm>
#include <array>
#include <asm-generic/socket.h>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <span>
#include <sys/poll.h>
#include <sys/socket.h>
#include <utility>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::usb_host;
namespace io = blofeld::usbip_tcp;
namespace tcp = blofeld::test::usbip_tcp;
namespace session = blofeld::test::usbip_session;
namespace wire = blofeld::test::usbip_fixture;
namespace device = blofeld::test::usb_device;
namespace guest = blofeld::test::usb_device_native;
using Time = std::chrono::steady_clock::time_point;
using Milliseconds = std::chrono::milliseconds;
constexpr Time origin{};
constexpr auto timeout = Milliseconds{10};
constexpr auto maximum_timeout = std::chrono::hours{24};
constexpr auto before_expiry = timeout - Milliseconds{1};
constexpr auto renewed_expiry = before_expiry + timeout;
constexpr auto before_renewed_expiry = renewed_expiry - Milliseconds{1};
constexpr unsigned filler_value = 0x5a;
constexpr std::array<std::uint8_t, 12> absent{1, 0x11, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0};

struct Pair {
  io::Connection channel;
  tcp::Socket peer;
};

Pair connection() {
  std::array<int, 2> descriptors{-1, -1};
  require(
      ::socketpair(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0, descriptors.data()) == 0,
      "A real local stream pair supplies deterministic kernel backpressure");
  tcp::Socket accepted(descriptors[0]);
  tcp::Socket peer(descriptors[1]);
  constexpr int small_buffer = 1024;
  require(
      ::setsockopt(accepted.get(), SOL_SOCKET, SO_SNDBUF, &small_buffer, sizeof(small_buffer)) == 0,
      "A small real send buffer bounds the test without replacing any send call");
  auto state = std::make_unique<UsbipSession>();
  state->receive(session::list);
  return {.channel = {.socket = std::move(accepted),
                      .session = std::move(state),
                      .id = 1,
                      .frame_deadline = std::nullopt,
                      .output_deadline = std::nullopt},
          .peer = std::move(peer)};
}

std::size_t fill_kernel(const tcp::Socket &socket) {
  std::array<std::uint8_t, 4096> filler{};
  filler.fill(filler_value);
  std::size_t total = 0;
  for (unsigned attempt = 0; attempt < 256; ++attempt) {
    const auto count =
        ::send(socket.get(), filler.data(), filler.size(), MSG_DONTWAIT | MSG_NOSIGNAL);
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      require(total != 0,
              "Actual stream writes precede the independently observed kernel backpressure");
      return total;
    }
    if (count < 0 && errno == EINTR) { continue; }
    require(count > 0, "The own kernel-buffer filler has no unrelated send failure");
    total += static_cast<std::size_t>(count);
  }
  throw Fault("Own bounded stream output never reached actual EAGAIN");
}

void drain_filler(Pair &pair, const std::size_t total) {
  std::array<std::uint8_t, 4096> bytes{};
  std::size_t consumed = 0;
  for (unsigned attempt = 0; attempt < 512 && consumed < total; ++attempt) {
    const auto count = ::recv(
        pair.peer.get(), bytes.data(), std::min(bytes.size(), total - consumed), MSG_DONTWAIT);
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
      pollfd ready{.fd = pair.peer.get(), .events = POLLIN, .revents = 0};
      require(::poll(&ready, 1, tcp::wait_milliseconds) == 1,
              "The real queued stream filler becomes readable");
      continue;
    }
    require(count > 0 &&
                std::ranges::all_of(std::span(bytes).first(static_cast<std::size_t>(count)),
                                    [](const auto byte) { return byte == filler_value; }),
            "Only the independently supplied filler is drained");
    consumed += static_cast<std::size_t>(count);
  }
  require(consumed == total, "All actual queued filler bytes were received");
  pollfd ready{.fd = pair.channel.socket.get(), .events = POLLOUT, .revents = 0};
  require(::poll(&ready, 1, tcp::wait_milliseconds) == 1,
          "The real peer read restores socket write readiness");
}

void socket_progress() {
  auto stalled = connection();
  static_cast<void>(fill_kernel(stalled.channel.socket));
  std::vector<UsbipConnectionEvent> events;
  std::size_t budget = 65536;
  require(io::flush(stalled.channel, budget, origin, timeout, events) && budget == 65536 &&
              stalled.channel.session->queued_bytes() == absent.size(),
          "Actual EAGAIN starts a deadline without consuming reply bytes");
  require(io::flush(stalled.channel, budget, origin + before_expiry, timeout, events) &&
              stalled.channel.output_deadline == origin + timeout,
          "A blocked retry cannot renew the no-progress deadline");
  require(!io::expire(stalled.channel, origin + timeout, events) && events.size() == 1 &&
              events[0].error && !events[0].requires_board_reset &&
              events[0].reason.contains("output progress deadline expired") &&
              stalled.channel.session->closed(),
          "Actual stream backpressure expires exactly at its independent host deadline");

  auto progressing = connection();
  const auto filler = fill_kernel(progressing.channel.socket);
  budget = 65536;
  events.clear();
  require(io::flush(progressing.channel, budget, origin, timeout, events),
          "Blocked output remains pending");
  drain_filler(progressing, filler);
  budget = 1;
  require(io::flush(progressing.channel, budget, origin + before_expiry, timeout, events) &&
              budget == 0 && progressing.channel.session->queued_bytes() == absent.size() - 1 &&
              progressing.channel.output_deadline == origin + renewed_expiry &&
              io::expire(progressing.channel, origin + timeout, events),
          "Only a real accepted byte renews the deadline; the preceding expiry no longer applies");
  static_cast<void>(fill_kernel(progressing.channel.socket));
  budget = 65536;
  require(io::flush(progressing.channel, budget, origin + before_renewed_expiry, timeout, events) &&
              !io::expire(progressing.channel, origin + renewed_expiry, events),
          "Later real backpressure does not extend the deadline of the last actual write");

  auto deferred = connection();
  events.clear();
  budget = 0;
  require(io::flush(deferred.channel, budget, origin, timeout, events) &&
              io::flush(deferred.channel, budget, origin + before_expiry, timeout, events) &&
              deferred.channel.session->queued_bytes() == absent.size() &&
              !io::expire(deferred.channel, origin + timeout, events),
          "Output queued after the poll exhausted its budget still arms a nonrenewing deadline");

  auto drained = connection();
  events.clear();
  budget = 65536;
  require(io::flush(drained.channel, budget, origin, timeout, events) &&
              !drained.channel.output_deadline && drained.channel.session->closed() &&
              io::expire(drained.channel, origin + std::chrono::hours{2}, events) && events.empty(),
          "A fully accepted reply clears its timer instead of leaving a stale output deadline");
  tcp::Received received;
  tcp::read(drained.peer, received);
  require(std::ranges::equal(received.bytes, absent),
          "Actual reply bytes preserve the independent DEVLIST encoding");
}

void pump(UsbipTcpServer &server,
          McuBus &bus,
          const tcp::Socket &peer,
          tcp::Received &received,
          const Time now,
          const auto &complete,
          const ExecutionContext current = context,
          const unsigned maximum_polls = 1024) {
  for (unsigned attempt = 0; attempt < maximum_polls; ++attempt) {
    const auto events = server.poll(bus, current, now);
    received.events.insert(received.events.end(), events.begin(), events.end());
    tcp::read(peer, received);
    if (complete()) { return; }
    pollfd ready{.fd = peer.get(), .events = POLLIN, .revents = 0};
    static_cast<void>(::poll(&ready, 1, 1));
  }
  throw Fault("Own fixed-time TCP exchange exceeded its bounded poll count");
}

void management_and_bounds() {
  for (const auto duration :
       {Milliseconds{0}, -Milliseconds{1}, Milliseconds{maximum_timeout} + Milliseconds{1}}) {
    reject([duration] { const UsbipTcpServer server({.output_timeout = duration}); },
           "output timeout must be positive");
  }
  McuBus bus(device::descriptor_flash);
  UsbipTcpServer overflowing({.frame_timeout = Milliseconds{1}, .output_timeout = maximum_timeout});
  reject(
      [&] { static_cast<void>(overflowing.poll(bus, context, Time::max() - maximum_timeout / 2)); },
      "output deadline");
  UsbipTcpServer server({.write_budget = 1, .output_timeout = timeout});
  auto peer = tcp::connect(server.port());
  tcp::send(peer, session::list);
  tcp::Received received;
  pump(server, bus, peer, received, origin, [&] { return !received.bytes.empty(); }, context, 8);
  pump(server, bus, peer, received, origin + timeout, [&] { return received.eof; });
  require(
      !received.bytes.empty() && received.bytes.size() < absent.size() &&
          std::ranges::equal(received.bytes, std::span(absent).first(received.bytes.size())) &&
          received.events.size() == 1 && received.events[0].error &&
          !received.events[0].requires_board_reset &&
          received.events[0].reason.contains("output progress deadline expired") &&
          server.connections() == 0,
      "Public server closes a partially drained management response before sending another byte");
  auto healthy = tcp::connect(server.port());
  tcp::send(healthy, session::list);
  tcp::Received reply;
  pump(server, bus, healthy, reply, origin + timeout, [&] { return reply.eof; });
  require(reply.events.size() == 1 && !reply.events[0].error &&
              std::ranges::equal(reply.bytes, absent),
          "The listener still serves an independent exact response after a peer output timeout");
}

void partial_import() {
  McuBus bus(device::descriptor_flash);
  device::prepare_device(bus);
  auto publication = session::publication(bus);
  UsbipTcpServer server({.write_budget = 1, .output_timeout = timeout});
  server.publish(publication);
  auto peer = tcp::connect(server.port());
  tcp::send(peer, session::import());
  tcp::Received received;
  pump(server, bus, peer, received, origin, [&] { return !received.bytes.empty(); }, context, 8);
  pump(server, bus, peer, received, origin + timeout, [&] { return received.eof; });
  constexpr unsigned import_bytes = 320;
  require(
      !received.bytes.empty() && received.bytes.size() < import_bytes &&
          received.events.size() == 1 && received.events[0].error &&
          received.events[0].requires_board_reset && !publication->available() &&
          received.events[0].reason.contains("output progress deadline expired"),
      "An incomplete import reply still retains the actual exclusive lease and recovery policy");
}

void native_half_close() {
  McuBus bus(guest::firmware());
  constexpr unsigned qram_mapping = 0x21;
  bus.write(hc12::initee, qram_mapping);
  bus.retire_instruction();
  constexpr unsigned ram_begin = 0x1200;
  constexpr unsigned ram_bytes = 5120;
  constexpr unsigned ram_canary = 0xa5;
  constexpr unsigned unlink_sequence = 7;
  constexpr unsigned absent_sequence = 99;
  for (unsigned i = 0; i < ram_bytes; ++i) {
    bus.write(static_cast<std::uint16_t>(ram_begin + i), ram_canary);
  }
  guest::prepare_native(bus);
  Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.set_state(guest::initial);
  const auto descriptors = guest::capture_native(bus, cpu);
  const ExecutionContext current{.pc = cpu.state().pc, .ccr = cpu.state().ccr};
  auto publication = std::make_shared<UsbipExport>(
      descriptors,
      UsbipLocation{.path = "/blofeld-emu/usb/1-2", .busid = "1-2", .bus = 1, .device = 2},
      bus,
      current);
  UsbipTcpServer server({.write_budget = 1, .output_timeout = timeout});
  server.publish(publication);
  auto peer = tcp::connect(server.port());
  tcp::Received received;
  tcp::send(peer, session::import());
  constexpr unsigned imported_bytes = 320;
  pump(
      server,
      bus,
      peer,
      received,
      origin,
      [&] { return received.bytes.size() == imported_bytes; },
      current);
  auto control = request(1, control_input, device::device.size());
  control.setup = device::device_setup;
  tcp::send(peer, wire::submit(control));
  pump(
      server,
      bus,
      peer,
      received,
      origin,
      [&] { return cpu.pending_interrupt().has_value(); },
      current);
  const auto flash = bus.power_off_flash_image();
  const auto evidence = bus.usb_host_evidence();
  const auto configuration = bus.usb_host_configuration();
  std::vector<std::uint8_t> ram;
  ram.reserve(ram_bytes);
  for (unsigned i = 0; i < ram_bytes; ++i) {
    ram.push_back(bus.read(static_cast<std::uint16_t>(ram_begin + i)));
  }
  const auto status_before = word(bus, status);
  tcp::send(
      peer,
      wire::unlink(unlink_sequence,
                   absent_sequence)); // A genuine host reply, without canceling accepted SETUP1.
  require(::shutdown(peer.get(), SHUT_WR) == 0, "A valid input half-close follows complete frames");
  pump(
      server,
      bus,
      peer,
      received,
      origin,
      [&] { return received.bytes.size() > imported_bytes; },
      current,
      16);
  for (unsigned attempt = 0; attempt < 4; ++attempt) {
    static_cast<void>(server.poll(bus, current, origin));
  }
  pump(server, bus, peer, received, origin + timeout, [&] { return received.eof; }, current);
  const auto expected = wire::words({4, unlink_sequence, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  require(
      received.bytes.size() > imported_bytes &&
          received.bytes.size() < imported_bytes + expected.size() &&
          std::ranges::equal(std::span(received.bytes).subspan(imported_bytes),
                             std::span(expected).first(received.bytes.size() - imported_bytes)) &&
          received.events.size() == 1 && received.events[0].error &&
          received.events[0].requires_board_reset &&
          received.events[0].reason.contains("output progress deadline expired") &&
          !publication->available(),
      "Stalled half-close drains no fabricated reply or SETUP completion and retains its exclusive "
      "import");
  const auto after = bus.power_off_flash_image();
  require(after.bytes == flash.bytes && after.known == flash.known && after.fresh == flash.fresh &&
              word(bus, status) == status_before && bus.usb_host_evidence() == evidence &&
              bus.usb_host_configuration() == configuration && cpu.pending_interrupt().has_value(),
          "Host output expiry preserves actual native SETUP/IRQ ownership, link/configuration and "
          "all NVM");
  for (unsigned i = 0; i < ram_bytes; ++i) {
    require(bus.read(static_cast<std::uint16_t>(ram_begin + i)) == ram[i],
            "Output expiry preserves all mapped guest RAM");
  }
  guest::interrupt(cpu, {.vector = guest::setup_vector}, guest::setup_instructions);
  require((word(bus, endpoint_base) & 0x4000U) != 0 && !cpu.pending_interrupt(),
          "Actual own HC12 ISR still executes the accepted SETUP after transport retirement");
}
}

int main() {
  try {
    socket_progress();
    management_and_bounds();
    partial_import();
    native_half_close();
    std::cout
        << "Verified real stream EAGAIN and TCP output progress/deadlines and retained native "
           "SETUP across half-close. No original enumeration or hardware timing claim.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
