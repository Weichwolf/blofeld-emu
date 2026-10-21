// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_usb.hpp"
#include "blofeld/usb_transfer.hpp"
#include "blofeld/usbip_runtime.hpp"
#include "usb_device_fixture.hpp"
#include "usb_device_native_fixture.hpp"
#include "usb_host_fixture.hpp"
#include "usbip_fixture.hpp"
#include "usbip_runtime_fixture.hpp"
#include "usbip_session_fixture.hpp"
#include "usbip_tcp_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <sys/poll.h>

namespace {
using namespace blofeld;
using namespace blofeld::test::usb_host;
using blofeld::test::usbip_runtime::Fixture;
namespace native = blofeld::test::usb_device_native;
namespace device = blofeld::test::usb_device;
namespace session = blofeld::test::usbip_session;
namespace tcp = blofeld::test::usbip_tcp;
namespace wire = blofeld::test::usbip_fixture;
constexpr unsigned maximum_polls = 1000;
constexpr unsigned import_bytes = 320;
constexpr unsigned transfer_header_bytes = 48;
constexpr unsigned suspend_change = 0x1000;
constexpr unsigned suspend_detected = 0x10;
constexpr unsigned input_ready = 0x5012;
constexpr unsigned management_tick = 20;
constexpr unsigned device_descriptor_bytes = 18;
constexpr unsigned runtime_device_id = 0x10001;

void until(Fixture &f, const tcp::Socket &socket, tcp::Received &received, const auto &complete) {
  for (unsigned i = 0; i < maximum_polls; ++i) {
    f.poll(f.boundary.ticks + 1);
    tcp::read(socket, received);
    if (complete()) { return; }
    pollfd ready{.fd = socket.get(), .events = POLLIN, .revents = 0};
    static_cast<void>(::poll(&ready, 1, 1));
  }
  throw std::runtime_error("Own suspended USB runtime exceeded its bounded socket wait");
}

void suspend(Fixture &f) {
  const auto before = f.bus.usb_host_evidence();
  f.bus.observe_usb_decoder({.suspended = true, .enumerated = true});
  const auto live = word(f.bus, status);
  require(f.bus.usb_host_suspended() &&
              (live & (suspend_change | suspend_detected)) == (suspend_change | suspend_detected) &&
              f.bus.usb_host_evidence() == before,
          "Decoded suspend sets actual USD/USSC without invalidating the retained link identity");
}

void resume(Fixture &f) {
  const auto before = f.bus.usb_host_evidence();
  f.bus.observe_usb_decoder({.enumerated = true});
  const auto live = word(f.bus, status);
  require(!f.bus.usb_host_suspended() && (live & suspend_detected) == 0 &&
              (live & suspend_change) != 0 && f.bus.usb_host_evidence() == before,
          "Decoded resume clears USD and retains the actual change flag and link identity");
}

UsbTransferRequest descriptor_request(const unsigned sequence) {
  auto request = blofeld::test::usb_host::request(sequence, control_input, device_descriptor_bytes);
  request.setup = device::device_setup;
  return request;
}

void test_suspended_probe_and_retained_metadata() {
  Fixture f;
  const auto initial_configuration = f.bus.usb_host_configuration();
  suspend(f);
  f.poll(10);
  require(f.runtime.started() && !f.runtime.published() && (word(f.bus, status) & 1U) == 0 &&
              f.bus.usb_host_configuration() == initial_configuration,
          "Admission during known suspend waits without SETUP, readiness or configuration repair");
  auto absent = tcp::connect(f.runtime.port());
  tcp::send(absent, session::list);
  tcp::Received empty;
  until(f, absent, empty, [&] { return empty.eof; });
  constexpr std::array<std::uint8_t, 12> no_device{1, 0x11, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0};
  require(std::ranges::equal(empty.bytes, no_device) && (word(f.bus, status) & 1U) == 0,
          "Real management I/O continues while the still-uncaptured USB probe remains suspended");
  resume(f);
  f.finish_capture(f.boundary.ticks + 1);
  suspend(f);
  auto listed = tcp::connect(f.runtime.port());
  tcp::send(listed, session::list);
  tcp::Received metadata;
  until(f, listed, metadata, [&] { return metadata.eof; });
  constexpr unsigned configured_list_bytes = 332;
  constexpr unsigned configuration_offset = 321;
  constexpr unsigned interfaces_offset = 323;
  require(metadata.bytes.size() == configured_list_bytes && metadata.bytes[11] == 1 &&
              metadata.bytes[configuration_offset] == 1 && metadata.bytes[interfaces_offset] == 2 &&
              f.runtime.published() && f.bus.usb_host_configuration() == initial_configuration,
          "Captured descriptors and actual configuration remain exportable through later suspend");
  reject([&] { static_cast<void>(f.bus.request_usb_in(UsbBuffer::control_in)); }, "during suspend");
}

void test_control_setup_data_and_status_pause() {
  Fixture f;
  f.capture();
  f.poll(management_tick);
  suspend(f);
  auto client = tcp::connect(f.runtime.port());
  auto input = session::import("1-1");
  auto request = wire::submit(descriptor_request(7));
  wire::set(request, wire::Slot::device, runtime_device_id);
  input.insert(input.end(), request.begin(), request.end());
  tcp::send(client, input);
  tcp::Received received;
  until(f, client, received, [&] { return received.bytes.size() >= import_bytes; });
  f.poll(f.boundary.ticks + 1);
  require(received.bytes.size() == import_bytes && (word(f.bus, status) & 1U) == 0,
          "Import and transfer coalescing never inject SETUP while the actual device is suspended");
  resume(f);
  f.poll(f.boundary.ticks + 1);
  native::interrupt(f.cpu, {.vector = native::setup_vector}, native::setup_instructions);
  require(word(f.bus, endpoint_base) == input_ready,
          "The actual own native ISR prepares the expected DATA1 device descriptor");
  suspend(f);
  f.poll(f.boundary.ticks + 1);
  require(word(f.bus, endpoint_base) == input_ready,
          "Suspended packet polling leaves real DVALID, TXSIZ and the pending DATA PID intact");
  resume(f);
  f.poll(f.boundary.ticks + 1);
  native::interrupt(f.cpu, {.vector = native::input_vector}, 3);
  suspend(f);
  const auto before_status = word(f.bus, endpoint_base + 2);
  f.poll(f.boundary.ticks + 1);
  tcp::read(client, received);
  require(received.bytes.size() == import_bytes && word(f.bus, endpoint_base + 2) == before_status,
          "A suspended status stage neither changes OUT registers nor returns early success");
  resume(f);
  f.poll(f.boundary.ticks + 1);
  native::interrupt(f.cpu, {.vector = native::output_vector}, 3);
  const auto completed_bytes = import_bytes + transfer_header_bytes + device::device.size();
  until(f, client, received, [&] { return received.bytes.size() >= completed_bytes; });
  auto expected = wire::words({3, 7, 0, 0, 0, 0, device_descriptor_bytes, 0, 0, 0, 0, 0});
  auto descriptor = device::device;
  descriptor[7] = native::full_speed_maximum;
  expected.insert(expected.end(), descriptor.begin(), descriptor.end());
  require(received.bytes.size() == completed_bytes &&
              std::ranges::equal(std::span(received.bytes).subspan(import_bytes), expected) &&
              f.runtime.published() && !f.runtime.faulted(),
          "Only actual resumed IN/OUT ACKs return exact bytes through the same imported session");
}

void test_unlink_while_suspended_keeps_guest_effects() {
  Fixture f;
  f.capture();
  f.poll(management_tick);
  auto client = tcp::connect(f.runtime.port());
  auto input = session::import("1-1");
  auto request = wire::submit(descriptor_request(7));
  wire::set(request, wire::Slot::device, runtime_device_id);
  input.insert(input.end(), request.begin(), request.end());
  tcp::send(client, input);
  tcp::Received received;
  until(f, client, received, [&] { return (word(f.bus, status) & 1U) != 0; });
  suspend(f);
  native::interrupt(f.cpu, {.vector = native::setup_vector}, native::setup_instructions);
  auto unlink = wire::unlink(8, 7);
  wire::set(unlink, wire::Slot::device, runtime_device_id);
  tcp::send(client, unlink);
  until(f, client, received, [&] {
    return received.bytes.size() >= import_bytes + transfer_header_bytes;
  });
  // Linux USB/IP RET_UNLINK reports -ECONNRESET for a genuinely cancelled request.
  constexpr std::uint32_t cancelled = 0xffffff98;
  const auto expected = wire::words({4, 8, 0, 0, 0, cancelled, 0, 0, 0, 0, 0, 0});
  require(
      std::ranges::equal(std::span(received.bytes).subspan(import_bytes), expected) &&
          word(f.bus, endpoint_base) == input_ready,
      "Real suspended TCP UNLINK retires only the host request, preserving accepted guest data");
  resume(f);
  f.poll(f.boundary.ticks + 1);
  tcp::read(client, received);
  require(received.bytes.size() == import_bytes + transfer_header_bytes &&
              word(f.bus, endpoint_base) == input_ready && !f.runtime.faulted(),
          "Resume cannot generate a late cancelled reply or consume its remaining guest packet");
}
}

int main() {
  try {
    test_suspended_probe_and_retained_metadata();
    test_control_setup_data_and_status_pause();
    test_unlink_while_suspended_keeps_guest_effects();
    std::cout << "Decoded suspend/resume retains native USB state and TCP management/cancellation; "
                 "packet phases resume only from actual device ACKs. No wire timing proof.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
