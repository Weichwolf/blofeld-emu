// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_usb.hpp"
#include "blofeld/usb_transfer.hpp"
#include "blofeld/usbip_session.hpp"
#include "usb_device_fixture.hpp"
#include "usb_host_fixture.hpp"
#include "usbip_fixture.hpp"
#include "usbip_session_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>

namespace {
using namespace blofeld;
using namespace blofeld::test::usb_host;
namespace device = blofeld::test::usb_device;
namespace wire = blofeld::test::usbip_fixture;
namespace fixture = blofeld::test::usbip_session;

void test_management_and_native_data_ownership() {
  McuBus bus(device::descriptor_flash);
  device::prepare_device(bus);
  const auto exported = fixture::publication(bus);
  UsbipSession absent;
  absent.receive(fixture::list);
  constexpr std::array<std::uint8_t, 12> no_devices{1, 0x11, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0};
  require(std::ranges::equal(fixture::drain(absent), no_devices) && absent.closed(),
          "An unpublished device never appears in DEVLIST");
  UsbipSession session;
  auto bytes = fixture::import();
  const auto input = wire::submit(request(7, input_address, 8));
  bytes.insert(bytes.end(), input.begin(), input.end());
  constexpr unsigned import_reply = 320;
  session.receive(bytes, exported);
  session.poll(bus, context);
  require(
      session.imported() && session.pending() == 1 && session.queued_bytes() == import_reply,
      "Coalesced import and transfer preserve the unconsumed suffix and management-first output");
  require(fixture::drain(session).size() == import_reply,
          "Import metadata is distinct from transfer completion");
  session.poll(bus, context);
  require(session.output().empty(), "Actual NAK produces no successful transfer reply");
  constexpr std::array<std::uint8_t, 3> payload{0x19, 0x37, 0xa5};
  prime(bus, UsbBuffer::endpoint2, payload);
  session.poll(bus, context);
  auto expected = wire::words({3, 7, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0});
  expected.insert(expected.end(), payload.begin(), payload.end());
  fixture::Bytes actual;
  while (!session.output().empty()) {
    actual.push_back(session.output().front());
    session.consume_output(1);
  }
  require(actual == expected && session.pending() == 0 &&
              (word(bus, input_control) & acknowledged) == acknowledged,
          "Arbitrarily partial writes retain exact actual IN bytes and real ACK ownership");
  session.end_input();
  require(session.closed() && !exported->available(),
          "Imported EOF retires the lease without resetting/re-exporting the guest");
}

void test_cancelled_prefix_and_completed_race() {
  McuBus bus(device::descriptor_flash);
  device::prepare_device(bus);
  const auto exported = fixture::publication(bus);
  UsbipSession session;
  fixture::attach(session, exported);
  auto output = request(1, output_address, 10);
  output.output = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  session.receive(wire::submit(output));
  session.poll(bus, context);
  require(received(bus, UsbBuffer::endpoint3, 8) ==
              fixture::Bytes(output.output.begin(), output.output.begin() + 8),
          "The acknowledged prefix reaches actual OUT storage before cancellation");
  session.receive(wire::unlink(2, 1));
  constexpr unsigned linux_cancelled = 0xffffff98;
  require(session.pending() == 0 &&
              fixture::drain(session) ==
                  wire::words({4, 2, 0, 0, 0, linux_cancelled, 0, 0, 0, 0, 0, 0}),
          "Successful unlink returns only RET_UNLINK and suppresses late RET_SUBMIT");
  require(received(bus, UsbBuffer::endpoint3, 8).front() == 0,
          "Cancellation never rolls back actual device writes");
  session.poll(bus, context);
  require(session.output().empty(), "No stale success appears after cancelled request removal");
  release(bus, UsbBuffer::endpoint3);
  output.sequence = 3;
  output.length = 1;
  output.output = {9};
  session.receive(wire::submit(output));
  session.poll(bus, context);
  session.receive(wire::unlink(4, 3));
  const auto completed = fixture::drain(session);
  const auto first = wire::words({3, 3, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0});
  const auto second = wire::words({4, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  auto expected = first;
  expected.insert(expected.end(), second.begin(), second.end());
  require(completed == expected,
          "Already completed submissions precede a zero-status unlink result");
}

void test_half_close_and_fault_poisoning() {
  McuBus bus(device::descriptor_flash);
  device::prepare_device(bus);
  const auto exported = fixture::publication(bus);
  UsbipSession session;
  fixture::attach(session, exported);
  session.receive(wire::submit(request(1, input_address, 1)));
  session.end_input();
  session.poll(bus, context);
  require(!session.closed() && session.pending() == 1 && session.output().empty(),
          "A valid half-close keeps accepted requests pending on actual NAK");
  constexpr std::array<std::uint8_t, 1> payload{0x37};
  prime(bus, UsbBuffer::endpoint2, payload);
  session.poll(bus, context);
  require(!session.closed(), "Accepted data remains owned until every reply byte is consumed");
  static_cast<void>(fixture::drain(session));
  require(session.closed(), "A half-close ends only after actual completion and output drain");
  UsbipSession truncated;
  const auto frame = fixture::import();
  truncated.receive(std::span(frame).first(7), exported);
  reject([&] { truncated.end_input(); }, "EOF");
  require(truncated.faulted() && truncated.output().empty(),
          "Truncated management cannot fabricate success");
}

void test_stale_pid_and_remote_reset_reject() {
  McuBus bus(device::descriptor_flash);
  device::prepare_device(bus);
  const auto exported = fixture::publication(bus);
  constexpr std::array<std::uint8_t, 1> accepted{7};
  require(bus.receive_usb_out({.buffer = UsbBuffer::endpoint3, .data = accepted}) ==
              UsbResponse::ack,
          "An actual earlier host packet advances the device PID before the new session");
  release(bus, UsbBuffer::endpoint3);
  UsbipSession session;
  fixture::attach(session, exported);
  auto output = request(1, output_address, 1);
  output.output = {9};
  session.receive(wire::submit(output));
  reject([&] { session.poll(bus, context); }, "OUT DATA PID disagrees");
  require(
      session.faulted() && session.output().empty() &&
          received(bus, UsbBuffer::endpoint3, 1).front() == accepted.front(),
      "A stale host PID cannot count a duplicate ACK as new data or overwrite the accepted prefix");
  McuBus reset_bus(device::descriptor_flash);
  device::prepare_device(reset_bus);
  UsbipSession reset;
  fixture::attach(reset, fixture::publication(reset_bus));
  auto proxy = request(1, 0, 0);
  constexpr std::uint8_t hub_port_request = 0x23;
  proxy.setup = {hub_port_request, 3, 4, 0, 1, 0, 0, 0};
  reject([&] { reset.receive(wire::submit(proxy)); }, "remote device reset");
  require(reset.faulted() && reset.output().empty() && (word(reset_bus, status) & 1U) == 0,
          "The USB/IP hub reset proxy is neither a fake reset success nor a firmware class SETUP");
}

void test_maximum_frame_coalescing() {
  McuBus bus(device::descriptor_flash);
  device::prepare_device(bus);
  const auto exported = fixture::publication(bus);
  UsbipSession session;
  fixture::attach(session, exported);
  auto output = request(1, output_address, maximum_usb_transfer);
  constexpr std::uint8_t payload = 0x55;
  output.output.assign(maximum_usb_transfer, payload);
  const auto frame = wire::submit(output);
  std::size_t offset = 0;
  while (offset < frame.size() - 1) {
    const auto count = std::min(usbip_input_chunk, frame.size() - 1 - offset);
    session.receive(std::span(frame).subspan(offset, count));
    offset += count;
  }
  auto suffix = wire::unlink(2, 1);
  suffix.insert(suffix.begin(), frame.back());
  session.receive(suffix);
  constexpr unsigned linux_cancelled = 0xffffff98;
  require(session.pending() == 0 &&
              fixture::drain(session) ==
                  wire::words({4, 2, 0, 0, 0, linux_cancelled, 0, 0, 0, 0, 0, 0}),
          "The final byte of a maximum-sized OUT frame coalesces with the next complete command "
          "without overflow");
}

void test_backpressure_and_live_sequences() {
  McuBus bus(device::descriptor_flash);
  device::prepare_device(bus);
  const auto exported = fixture::publication(bus);
  UsbipSession session;
  fixture::attach(session, exported);
  std::uint32_t sequence = 1;
  constexpr unsigned absent_target = 0xfffffffe;
  while (session.wants_input() && sequence <= maximum_usbip_replies) {
    session.receive(wire::unlink(sequence, absent_target));
    ++sequence;
  }
  require(
      !session.wants_input() && !session.faulted() && sequence <= maximum_usbip_replies &&
          session.queued_bytes() <= maximum_usbip_output,
      "A non-reading peer reaches bounded output backpressure without unbounded frame allocation");
  const auto count = session.output().size();
  session.consume_output(count);
  require(session.wants_input(), "Consuming an actual reply restores bounded input capacity");
  reject([&] { session.receive(wire::unlink(2, absent_target)); }, "Duplicate live");
  require(session.faulted() && !exported->available(),
          "Unsent live sequences cannot be reused or restore the guest lease");
}
}

int main() {
  try {
    test_management_and_native_data_ownership();
    test_cancelled_prefix_and_completed_race();
    test_half_close_and_fault_poisoning();
    test_maximum_frame_coalescing();
    test_stale_pid_and_remote_reset_reject();
    test_backpressure_and_live_sequences();
    std::cout << "Verified bounded USB/IP sessions, actual MCU ACK/NAK ownership, partial output, "
                 "cancellation, EOF, coalesced maximum frames and backpressure.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
