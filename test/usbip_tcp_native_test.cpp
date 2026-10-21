// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/usbip_export.hpp"
#include "blofeld/usbip_tcp.hpp"
#include "usb_device_fixture.hpp"
#include "usb_device_native_fixture.hpp"
#include "usb_host_fixture.hpp"
#include "usbip_fixture.hpp"
#include "usbip_session_fixture.hpp"
#include "usbip_tcp_fixture.hpp"

#include <algorithm>
#include <array>
#include <exception>
#include <iostream>
#include <memory>
#include <span>
#include <sys/socket.h>

namespace {
using namespace blofeld;
using namespace blofeld::test::usb_host;
namespace guest = blofeld::test::usb_device_native;
namespace device = blofeld::test::usb_device;
namespace session = blofeld::test::usbip_session;
namespace wire = blofeld::test::usbip_fixture;
namespace tcp = blofeld::test::usbip_tcp;

void test_own_cpu_through_actual_tcp() {
  const auto flash = guest::firmware();
  McuBus bus(flash);
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
  UsbipTcpServer server({});
  server.publish(publication);
  auto client = tcp::connect(server.port());
  auto request_bytes = session::import();
  auto control = request(1, control_input, device::device.size());
  control.setup = device::device_setup;
  const auto submitted = wire::submit(control);
  request_bytes.insert(request_bytes.end(), submitted.begin(), submitted.end());
  tcp::send(client, request_bytes);
  tcp::Received received;
  constexpr unsigned imported_bytes = 320;
  constexpr unsigned transfer_header_bytes = 48;
  constexpr unsigned descriptor_bytes = device::device.size();
  tcp::until(
      server,
      bus,
      client,
      received,
      [&] { return received.bytes.size() >= imported_bytes; },
      current);
  static_cast<void>(server.poll(bus, current));
  guest::interrupt(cpu, {.vector = guest::setup_vector}, guest::setup_instructions);
  static_cast<void>(server.poll(bus, current));
  guest::interrupt(cpu, {.vector = guest::input_vector}, 3);
  static_cast<void>(server.poll(bus, current));
  guest::interrupt(cpu, {.vector = guest::output_vector}, 3);
  constexpr unsigned control_reply_bytes = transfer_header_bytes + descriptor_bytes;
  tcp::until(
      server,
      bus,
      client,
      received,
      [&] { return received.bytes.size() >= imported_bytes + control_reply_bytes; },
      current);
  auto expected = wire::words({3, 1, 0, 0, 0, 0, descriptor_bytes, 0, 0, 0, 0, 0});
  auto own_descriptor = device::device;
  own_descriptor[7] = guest::full_speed_maximum;
  expected.insert(expected.end(), own_descriptor.begin(), own_descriptor.end());
  require(std::ranges::equal(std::span(received.bytes).subspan(imported_bytes), expected),
          "Real TCP reply contains only the descriptor copied by actual own HC12 code after "
          "setup/data/status IRQs");
  control.sequence = 2;
  tcp::send(client, wire::submit(control));
  tcp::until(
      server, bus, client, received, [&] { return cpu.pending_interrupt().has_value(); }, current);
  tcp::send(client, wire::unlink(3, 2));
  constexpr unsigned cancelled_reply_bytes =
      imported_bytes + control_reply_bytes + transfer_header_bytes;
  tcp::until(
      server,
      bus,
      client,
      received,
      [&] { return received.bytes.size() >= cancelled_reply_bytes; },
      current);
  constexpr unsigned linux_cancelled = 0xffffff98;
  require(std::ranges::equal(std::span(received.bytes).last(transfer_header_bytes),
                             wire::words({4, 3, 0, 0, 0, linux_cancelled, 0, 0, 0, 0, 0, 0})),
          "An unlink after actual SETUP suppresses its submission reply without undoing guest IRQ "
          "ownership");
  guest::interrupt(cpu, {.vector = guest::setup_vector}, guest::setup_instructions);
  constexpr unsigned data_valid = 0x4000;
  require((word(bus, endpoint_base) & data_valid) != 0 && !cpu.pending_interrupt(),
          "The already accepted SETUP still executes its own guest ISR, leaving unacknowledged USB "
          "RAM data");
  require(::shutdown(client.get(), SHUT_WR) == 0, "Own native TCP peer half-closes");
  tcp::until(server, bus, client, received, [&] { return received.eof; }, current);
  require(received.bytes.size() == cancelled_reply_bytes && !publication->available() &&
              (word(bus, endpoint_base) & data_valid) != 0,
          "Disconnect creates no late success, guest reset, buffer rollback or automatic "
          "re-publication");
}
}

int main() {
  try {
    test_own_cpu_through_actual_tcp();
    std::cout
        << "Verified real loopback USB/IP import/control/unlink/EOF through own HC12 guest IRQs "
           "and USB RAM. No original firmware enumeration or kernel USB attachment claim.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
