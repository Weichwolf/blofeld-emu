// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_usb.hpp"
#include "blofeld/usbip_tcp.hpp"
#include "usb_device_fixture.hpp"
#include "usb_host_fixture.hpp"
#include "usbip_fixture.hpp"
#include "usbip_session_fixture.hpp"
#include "usbip_tcp_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <span>
#include <sys/socket.h>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::usb_host;
namespace device = blofeld::test::usb_device;
namespace session = blofeld::test::usbip_session;
namespace wire = blofeld::test::usbip_fixture;
namespace tcp = blofeld::test::usbip_tcp;

void test_loopback_management_and_budget() {
  McuBus bus(device::descriptor_flash);
  UsbipTcpServer server({.write_budget = 7});
  require(server.port() != 0, "The real kernel assigns a loopback listener port");
  auto client = tcp::connect(server.port());
  tcp::Received received;
  for (const auto byte : session::list) {
    tcp::send(client, std::span(&byte, 1));
    const auto events = server.poll(bus, context);
    received.events.insert(received.events.end(), events.begin(), events.end());
  }
  tcp::until(server, bus, client, received, [&] { return received.eof; });
  constexpr std::array<std::uint8_t, 12> empty{1, 0x11, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0};
  require(
      std::ranges::equal(received.bytes, empty) && server.connections() == 0,
      "A fragmented DEVLIST drains exact zero-device bytes across a small write budget and closes");
  device::prepare_device(bus);
  const auto publication = session::publication(bus);
  server.publish(publication);
  auto wrong = tcp::connect(server.port());
  tcp::send(wrong, session::import("1-3"));
  tcp::Received rejected;
  tcp::until(server, bus, wrong, rejected, [&] { return rejected.eof; });
  constexpr std::array<std::uint8_t, 8> error{1, 0x11, 0, 3, 0, 0, 0, 1};
  require(std::ranges::equal(rejected.bytes, error) && publication->available(),
          "Wrong busid import sends only the exact error header and leaves the real publication "
          "available");
  auto imported = tcp::connect(server.port());
  tcp::send(imported, session::import());
  tcp::Received reply;
  constexpr unsigned import_bytes = 320;
  tcp::until(server, bus, imported, reply, [&] { return reply.bytes.size() >= import_bytes; });
  require(reply.bytes.size() == import_bytes && !reply.eof && !publication->available(),
          "Successful import drains the fixed record without closing its transfer socket");
  reject([&] { server.publish(publication); }, "publication requires");
  require(::shutdown(imported.get(), SHUT_WR) == 0, "Own client half-closes after import");
  tcp::until(server, bus, imported, reply, [&] { return reply.eof; });
  require(std::ranges::any_of(reply.events,
                              [](const auto &event) { return event.requires_board_reset; }) &&
              !publication->available(),
          "Imported close reports required board qualification and never automatically releases "
          "the guest lease");
}

void test_actual_transfer_and_retired_publication() {
  McuBus bus(device::descriptor_flash);
  device::prepare_device(bus);
  const auto publication = session::publication(bus);
  UsbipTcpServer server({});
  server.publish(publication);
  auto client = tcp::connect(server.port());
  auto input = session::import();
  const auto command = wire::submit(request(7, input_address, 8));
  input.insert(input.end(), command.begin(), command.end());
  tcp::send(client, input);
  tcp::Received received;
  constexpr unsigned import_bytes = 320;
  tcp::until(server, bus, client, received, [&] { return received.bytes.size() >= import_bytes; });
  require(received.bytes.size() == import_bytes,
          "The imported socket has no response for an actual NAK");
  constexpr std::array<std::uint8_t, 3> payload{0x19, 0x37, 0xa5};
  prime(bus, UsbBuffer::endpoint2, payload);
  constexpr unsigned completed_bytes = import_bytes + 48 + payload.size();
  tcp::until(
      server, bus, client, received, [&] { return received.bytes.size() >= completed_bytes; });
  auto expected = wire::words({3, 7, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0});
  expected.insert(expected.end(), payload.begin(), payload.end());
  require(
      std::ranges::equal(std::span(received.bytes).subspan(import_bytes), expected) &&
          (word(bus, input_control) & acknowledged) == acknowledged,
      "Actual loopback RET_SUBMIT contains exact MCU data only after the real device ACK callback");
  require(::shutdown(client.get(), SHUT_WR) == 0, "Own client ends its complete transfer stream");
  tcp::until(server, bus, client, received, [&] { return received.eof; });
  auto listed = tcp::connect(server.port());
  tcp::send(listed, session::list);
  tcp::Received listing;
  tcp::until(server, bus, listed, listing, [&] { return listing.eof; });
  constexpr unsigned empty_list_bytes = 12;
  require(listing.bytes.size() == empty_list_bytes && listing.bytes.back() == 0 &&
              !publication->available(),
          "A disconnected imported device is absent until a new qualified publication exists");
}

void test_malformed_peer_isolation_and_capacity() {
  McuBus bus(device::descriptor_flash);
  UsbipTcpServer server({});
  auto malformed = tcp::connect(server.port());
  auto header = session::list;
  constexpr std::uint8_t wrong_version = 0x12;
  header[1] = wrong_version;
  tcp::send(malformed, header);
  tcp::Received bad;
  tcp::until(server, bus, malformed, bad, [&] { return bad.eof; });
  require(bad.bytes.empty() &&
              std::ranges::any_of(bad.events, [](const auto &event) { return event.error; }),
          "Malformed management closes only that peer and records its visible reason");
  std::vector<tcp::Socket> clients;
  for (unsigned i = 0; i < 4; ++i) {
    clients.push_back(tcp::connect(server.port()));
    static_cast<void>(server.poll(bus, context));
  }
  auto excess = tcp::connect(server.port());
  tcp::Received rejected;
  tcp::until(server, bus, excess, rejected, [&] { return rejected.eof; });
  require(
      server.connections() == 4 &&
          std::ranges::any_of(rejected.events,
                              [](const auto &event) { return event.reason.contains("capacity"); }),
      "Real accepted sockets remain bounded while a fifth peer is closed with a capacity event");
  server.withdraw();
  require(server.connections() == 0,
          "Withdrawal closes all socket owners without guest reset or new publication");
}

void test_device_fault_stops_transport() {
  McuBus bus(device::descriptor_flash);
  device::prepare_device(bus);
  const auto publication = session::publication(bus);
  constexpr std::array<std::uint8_t, 1> previous{7};
  require(bus.receive_usb_out({.buffer = UsbBuffer::endpoint3, .data = previous}) ==
              UsbResponse::ack,
          "An actual earlier OUT packet establishes a conflicting host/controller PID");
  release(bus, UsbBuffer::endpoint3);
  UsbipTcpServer server({});
  server.publish(publication);
  auto client = tcp::connect(server.port());
  auto bytes = session::import();
  auto output = request(1, output_address, 1);
  output.output = {9};
  const auto submitted = wire::submit(output);
  bytes.insert(bytes.end(), submitted.begin(), submitted.end());
  tcp::send(client, bytes);
  tcp::Received received;
  constexpr unsigned import_bytes = 320;
  tcp::until(server, bus, client, received, [&] { return received.bytes.size() >= import_bytes; });
  reject([&] { static_cast<void>(server.poll(bus, context)); }, "mcu-pc=0x8123");
  tcp::read(client, received);
  require(server.connections() == 0 && received.eof && received.bytes.size() == import_bytes &&
              !publication->available(),
          "A real model fault closes the listener/connection without a fabricated submission "
          "completion");
  reject([&] { static_cast<void>(server.poll(bus, context)); }, "Faulted/moved");
}

std::size_t descriptors() {
  return static_cast<std::size_t>(std::distance(
      std::filesystem::directory_iterator("/proc/self/fd"), std::filesystem::directory_iterator{}));
}
}

int main() {
  try {
    const auto before = descriptors();
    test_loopback_management_and_budget();
    test_actual_transfer_and_retired_publication();
    test_malformed_peer_isolation_and_capacity();
    test_device_fault_stops_transport();
    require(descriptors() == before,
            "All listener/client/accepted descriptors are closed after real TCP tests");
    std::cout << "Verified actual bounded loopback TCP discovery/import/MCU transfers, partial "
                 "output, peer isolation, EOF/retired leases and descriptor ownership. No kernel "
                 "USB attachment claim.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
