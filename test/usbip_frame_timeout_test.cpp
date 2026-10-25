// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_usb.hpp"
#include "usb/usbip_export.hpp"
#include "usb/usbip_tcp.hpp"
#include "usb_device_fixture.hpp"
#include "usb_device_native_fixture.hpp"
#include "usb_host_fixture.hpp"
#include "usbip_fixture.hpp"
#include "usbip_session_fixture.hpp"
#include "usbip_tcp_fixture.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <span>

namespace {
using namespace blofeld;
using namespace blofeld::test::usb_host;
using Seconds = std::chrono::seconds;
using Milliseconds = std::chrono::milliseconds;
using Hours = std::chrono::hours;
using Time = std::chrono::steady_clock::time_point;
namespace device = blofeld::test::usb_device;
namespace guest = blofeld::test::usb_device_native;
namespace session = blofeld::test::usbip_session;
namespace wire = blofeld::test::usbip_fixture;
namespace tcp = blofeld::test::usbip_tcp;
constexpr Time origin{};
constexpr auto budget = Seconds{30};
constexpr auto maximum_timeout = Hours{24};
constexpr unsigned import_bytes = 320;

void pump(UsbipTcpServer &server,
          McuBus &bus,
          const tcp::Socket &client,
          tcp::Received &received,
          const Time now,
          const ExecutionContext current = context) {
  // All bounded polls share a time: TCP scheduling cannot consume a test deadline.
  for (unsigned i = 0; i < 16; ++i) {
    const auto events = server.poll(bus, current, now);
    received.events.insert(received.events.end(), events.begin(), events.end());
    tcp::read(client, received);
  }
}

void require_expired(const tcp::Received &received, const bool imported) {
  require(received.eof &&
              std::ranges::any_of(received.events,
                                  [imported](const auto &event) {
                                    return event.error && event.requires_board_reset == imported &&
                                           event.reason.contains("input frame deadline expired");
                                  }),
          "An incomplete frame closes only its peer with an explicit imported recovery policy");
}

void test_initial_deadline_and_isolation() {
  McuBus bus(device::descriptor_flash);
  UsbipTcpServer server({});
  auto idle = tcp::connect(server.port());
  auto trickle = tcp::connect(server.port());
  tcp::send(trickle, std::span(session::list).first(1));
  static_cast<void>(server.poll(bus, context, origin));
  tcp::send(trickle, std::span(session::list).subspan(1, 1));
  static_cast<void>(server.poll(bus, context, origin + budget - Seconds{1}));
  require(server.connections() == 2,
          "The initial management deadline permits idle and fragmented peers before expiry");
  auto healthy = tcp::connect(server.port());
  tcp::send(healthy, session::list);
  tcp::Received good;
  pump(server, bus, healthy, good, origin + budget - Seconds{1});
  constexpr std::array<std::uint8_t, 12> absent{1, 0x11, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0};
  require(good.eof && std::ranges::equal(good.bytes, absent) && server.connections() == 2,
          "Stalled management peers do not prevent an independent exact DEVLIST exchange");
  const auto events = server.poll(bus, context, origin + budget);
  tcp::Received idle_reply{.bytes = {}, .events = events};
  tcp::Received trickle_reply{.bytes = {}, .events = events};
  tcp::read(idle, idle_reply);
  tcp::read(trickle, trickle_reply);
  require_expired(idle_reply, false);
  require_expired(trickle_reply, false);
  require(events.size() == 2 && server.connections() == 0 && idle_reply.bytes.empty() &&
              trickle_reply.bytes.empty(),
          "The exact initial deadline expires both peers without extending on byte trickles");
  require(events.front().connection != events.back().connection &&
              std::ranges::all_of(events,
                                  [](const auto &event) {
                                    return event.error && !event.requires_board_reset &&
                                           event.reason.contains("input frame deadline expired");
                                  }),
          "Both independent management peers report their own non-imported timeout event");
}

void test_idle_midi_and_coalesced_frame_boundary() {
  McuBus bus(device::descriptor_flash);
  device::prepare_device(bus);
  auto publication = session::publication(bus);
  UsbipTcpServer server({});
  server.publish(publication);
  auto client = tcp::connect(server.port());
  auto input = session::import();
  const auto pending = wire::submit(request(7, input_address, 8));
  input.insert(input.end(), pending.begin(), pending.end());
  tcp::send(client, input);
  tcp::Received received;
  pump(server, bus, client, received, origin);
  require(received.bytes.size() == import_bytes && !received.eof,
          "A coalesced import and complete MIDI IN request retain their exact boundaries");
  pump(server, bus, client, received, origin + Hours{2});
  require(received.bytes.size() == import_bytes && received.events.empty() &&
              server.connections() == 1,
          "A fully framed pending MIDI IN remains open far beyond the frame assembly timeout");
  constexpr std::array<std::uint8_t, 3> payload{0x19, 0x37, 0xa5};
  prime(bus, UsbBuffer::endpoint2, payload);
  pump(server, bus, client, received, origin + Hours{2});
  auto expected = wire::words({3, 7, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0});
  expected.insert(expected.end(), payload.begin(), payload.end());
  require(std::ranges::equal(std::span(received.bytes).subspan(import_bytes), expected),
          "The waiting read returns exact actual MCU bytes once the endpoint ACKs");
  const auto after_idle = origin + Hours{3};
  pump(server, bus, client, received, after_idle);
  require(!received.eof && received.events.empty() &&
              received.bytes.size() == import_bytes + expected.size(),
          "An imported peer with no pending URB also remains open without an inactivity deadline");
  const auto first = wire::unlink(8, 7);
  tcp::send(client, std::span(first).first(16));
  pump(server, bus, client, received, after_idle + Seconds{1});
  auto tail = tcp::Bytes(first.begin() + 16, first.end());
  const auto second = wire::unlink(9, 7);
  tail.push_back(second.front());
  tcp::send(client, tail);
  pump(server, bus, client, received, after_idle + Seconds{10});
  const auto reply_bytes = received.bytes.size();
  constexpr unsigned unlink_bytes = 48;
  require(reply_bytes == import_bytes + expected.size() + unlink_bytes &&
              std::ranges::equal(std::span(received.bytes).last(unlink_bytes),
                                 wire::words({4, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0})),
          "Completing an old frame and beginning the next in one read yields exact UNLINK bytes");
  pump(server, bus, client, received, after_idle + budget + Seconds{1});
  require(!received.eof && received.events.empty(),
          "A genuinely new partial frame receives its own budget across a coalesced boundary");
  tcp::send(client, std::span(second).subspan(1, 1));
  pump(server, bus, client, received, after_idle + budget + Seconds{9});
  pump(server, bus, client, received, after_idle + budget + Seconds{10});
  require_expired(received, true);
  require(received.bytes.size() == reply_bytes && !publication->available(),
          "A later byte trickle cannot extend the new deadline or release its imported lease");
}

void test_partial_payload_preserves_native_setup() {
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
  UsbipTcpServer server({.frame_timeout = Milliseconds{10}});
  server.publish(publication);
  auto client = tcp::connect(server.port());
  auto input = session::import();
  auto control = request(1, control_input, device::device.size());
  control.setup = device::device_setup;
  const auto submitted = wire::submit(control);
  input.insert(input.end(), submitted.begin(), submitted.end());
  tcp::send(client, input);
  tcp::Received received;
  pump(server, bus, client, received, origin, current);
  require(cpu.pending_interrupt().has_value() && received.bytes.size() == import_bytes,
          "A complete control request actually owns SETUP/IRQ while awaiting native execution");
  auto output = request(2, output_address, 3);
  output.output = {1, 2, 3};
  const auto partial = wire::submit(output);
  tcp::send(client, std::span(partial).first(partial.size() - 1));
  pump(server, bus, client, received, origin + Milliseconds{1}, current);
  pump(server, bus, client, received, origin + Milliseconds{10}, current);
  require(!received.eof, "A partial OUT payload is accepted within its explicit host budget");
  pump(server, bus, client, received, origin + Milliseconds{11}, current);
  require_expired(received, true);
  require(cpu.pending_interrupt().has_value() && received.bytes.size() == import_bytes &&
              !publication->available(),
          "An expired later frame neither rolls back accepted SETUP nor invents a URB reply");
  guest::interrupt(cpu, {.vector = guest::setup_vector}, guest::setup_instructions);
  constexpr unsigned data_valid = 0x4000;
  require((word(bus, endpoint_base) & data_valid) != 0 && !cpu.pending_interrupt(),
          "After transport retirement actual HC12 ISR still produces guest-owned USB RAM");
}

void test_policy_and_clock_bounds() {
  for (const auto duration : {Milliseconds{0},
                              -Milliseconds{1},
                              std::chrono::milliseconds{maximum_timeout} + Milliseconds{1}}) {
    reject([duration] { const UsbipTcpServer server({.frame_timeout = duration}); },
           "frame timeout must be positive");
  }
  McuBus bus(device::descriptor_flash);
  UsbipTcpServer server({.frame_timeout = maximum_timeout});
  auto client = tcp::connect(server.port());
  static_cast<void>(server.poll(bus, context, origin));
  reject([&] { static_cast<void>(server.poll(bus, context, origin - Milliseconds{1})); },
         "host clock moved backwards");
  tcp::Received received;
  tcp::read(client, received);
  require(received.eof && server.connections() == 0,
          "A backwards host clock retires the listener instead of extending existing deadlines");
  reject([&] { static_cast<void>(server.poll(bus, context, origin)); }, "Faulted/moved");
  UsbipTcpServer overflowing({});
  reject([&] { static_cast<void>(overflowing.poll(bus, context, Time::max())); },
         "cannot represent a frame deadline");
}
}

int main() {
  try {
    test_initial_deadline_and_isolation();
    test_idle_midi_and_coalesced_frame_boundary();
    test_partial_payload_preserves_native_setup();
    test_policy_and_clock_bounds();
    std::cout << "Verified real TCP frame deadlines with an injected monotonic clock, idle MIDI "
                 "NAKs, coalesced boundaries and retained native SETUP/IRQ. No kernel USB or "
                 "original enumeration claim.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
