// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_usb.hpp"
#include "usb/usb_transfer.hpp"
#include "usb/usbip.hpp"
#include "usb/usbip_export.hpp"
#include "usb/usbip_session.hpp"
#include "usb_device_fixture.hpp"
#include "usb_endpoint_reset_fixture.hpp"
#include "usb_host_fixture.hpp"
#include "usbip_fixture.hpp"
#include "usbip_session_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <span>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::usb_host;
namespace device = blofeld::test::usb_device;
namespace wire = blofeld::test::usbip_fixture;
namespace fixture = blofeld::test::usbip_session;
using Bytes = std::vector<std::uint8_t>;
constexpr std::array<std::uint8_t, 4> payload{0x19, 0x73, 0xa5, 0x42};

std::shared_ptr<UsbipExport> publish(McuBus &bus) {
  device::prepare_device(bus);
  return fixture::publication(bus);
}

struct Fixture {
  McuBus bus{device::descriptor_flash};
  std::shared_ptr<UsbipExport> publication = publish(bus);
  UsbipSession session;
};

void all_import_transfer_splits() {
  auto coalesced = fixture::import();
  const auto input = wire::submit(request(7, input_address, payload.size()));
  coalesced.insert(coalesced.end(), input.begin(), input.end());
  constexpr unsigned import_reply_bytes = 320;
  for (std::size_t split = 0; split <= coalesced.size(); ++split) {
    Fixture state;
    auto &session = state.session;
    session.receive(std::span(coalesced).first(split), state.publication);
    session.receive(std::span(coalesced).subspan(split), state.publication);
    require(session.imported() && session.pending() == 1 &&
                fixture::drain(session).size() == import_reply_bytes,
            "Every fixed import/IN split preserves exactly one import and pending transfer");
    session.poll(state.bus, context);
    require(session.output().empty(), "Actual NAK supplies no fabricated transfer reply");
    prime(state.bus, UsbBuffer::endpoint2, payload);
    session.poll(state.bus, context);
    auto expected = wire::words({3, 7, 0, 0, 0, 0, payload.size(), 0, 0, 0, 0, 0});
    expected.insert(expected.end(), payload.begin(), payload.end());
    require(fixture::drain(session) == expected && session.pending() == 0,
            "Every split returns exact MCU-owned bytes and independently specified wire fields");
    session.end_input();
    require(session.closed() && !state.publication->available(),
            "EOF never automatically re-advertises an imported guest");
  }
}

void all_truncated_eof_boundaries() {
  const auto management = fixture::import();
  for (std::size_t cut = 0; cut < management.size(); ++cut) {
    Fixture state;
    state.session.receive(std::span(management).first(cut), state.publication);
    reject([&] { state.session.end_input(); }, "EOF");
    require(state.session.faulted() && state.session.output().empty() &&
                state.publication->available(),
            "Every incomplete import fails without output or an acquired lease");
  }
  const auto input = wire::submit(request(1, input_address, 1));
  for (std::size_t cut = 1; cut < input.size(); ++cut) {
    Fixture state;
    fixture::attach(state.session, state.publication);
    state.session.receive(std::span(input).first(cut));
    reject([&] { state.session.end_input(); }, "EOF");
    require(state.session.faulted() && state.session.output().empty() &&
                !state.publication->available(),
            "Every truncated transfer header fails while retaining its imported claim");
  }
  auto output = request(2, output_address, payload.size());
  output.output.assign(payload.begin(), payload.end());
  const auto bytes = wire::submit(output);
  for (std::size_t cut = usbip_transfer_header; cut < bytes.size(); ++cut) {
    Fixture state;
    fixture::attach(state.session, state.publication);
    state.session.receive(std::span(bytes).first(cut));
    require(state.session.pending() == 0, "Partial OUT data never enters the real transfer queue");
    reject([&] { state.session.end_input(); }, "EOF");
    require(state.session.faulted() && state.session.output().empty(),
            "Every missing OUT payload suffix faults without a transfer reply");
  }
}

void malformed_suffix_and_partial_reply_identity() {
  Fixture malformed;
  auto bytes = fixture::import();
  constexpr unsigned unknown_command = 99;
  const auto invalid = wire::words({unknown_command, 1, wire::device, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  bytes.insert(bytes.end(), invalid.begin(), invalid.end());
  reject([&] { malformed.session.receive(bytes, malformed.publication); }, "Unknown USB/IP");
  require(malformed.session.faulted() && malformed.session.output().empty() &&
              !malformed.publication->available(),
          "An invalid coalesced suffix cannot leak its already queued import-success reply");
  Fixture partial;
  fixture::attach(partial.session, partial.publication);
  partial.session.receive(wire::submit(request(7, input_address, payload.size())));
  prime(partial.bus, UsbBuffer::endpoint2, payload);
  partial.session.poll(partial.bus, context);
  partial.session.consume_output(1);
  const auto acknowledged_control = word(partial.bus, input_control);
  reject([&] { partial.session.receive(wire::unlink(7, 8)); }, "Duplicate live");
  require(partial.session.faulted() && partial.session.output().empty() &&
              !partial.publication->available() &&
              word(partial.bus, input_control) == acknowledged_control,
          "A partly sent reply retains its identity; rejection never rolls back an actual ACK");
}

void full_queue_cancellation_and_pending_disconnect() {
  Fixture full;
  fixture::attach(full.session, full.publication);
  for (unsigned sequence = 0; sequence < maximum_usb_pending; ++sequence) {
    full.session.receive(wire::submit(request(sequence, input_address, 1)));
  }
  full.session.receive(wire::unlink(static_cast<unsigned>(maximum_usb_pending), 0));
  constexpr unsigned linux_cancelled = 0xffffff98;
  require(full.session.pending() == maximum_usb_pending - 1 &&
              fixture::drain(full.session) ==
                  wire::words({4, maximum_usb_pending, 0, 0, 0, linux_cancelled, 0, 0, 0, 0, 0, 0}),
          "A full host transfer queue still accepts UNLINK and suppresses its target completion");
  full.session.abort();
  require(full.session.closed() && full.session.pending() == 0 && full.session.output().empty() &&
              !full.publication->available(),
          "Abort drops host scheduling without successful completion or automatic lease release");
  Fixture pending;
  fixture::attach(pending.session, pending.publication);
  auto control = request(1, control_input, device::device.size());
  control.setup = device::device_setup;
  pending.session.receive(wire::submit(control));
  pending.session.poll(pending.bus, context);
  const auto flags = word(pending.bus, status);
  pending.session.end_input();
  require(!pending.session.closed() && !pending.session.wants_input() &&
              pending.session.pending() == 1 && pending.session.output().empty(),
          "Half-close retains the accepted SETUP until real completion or explicit abort");
  pending.session.abort();
  require(word(pending.bus, status) == flags && !pending.publication->available() &&
              pending.session.pending() == 0 && pending.session.output().empty(),
          "Aborting the pending SETUP creates no guest reset, status success or IRQ rollback");
}

void controller_failure_and_exclusive_import() {
  Fixture state;
  fixture::attach(state.session, state.publication);
  UsbipSession second;
  second.receive(fixture::import(), state.publication);
  constexpr std::array<std::uint8_t, 8> busy{1, 0x11, 0, 3, 0, 0, 0, 1};
  require(std::ranges::equal(fixture::drain(second), busy) && second.closed() &&
              state.session.imported(),
          "A second connection cannot steal the imported controller");
  state.bus.write(test::endpoint_reset::control, test::endpoint_reset::mce_high);
  constexpr unsigned reserved_nisof = 0x804000d1;
  test::endpoint_reset::configure(state.bus, {.bank = 2, .value = reserved_nisof});
  state.bus.write(test::endpoint_reset::control, test::endpoint_reset::men_high);
  state.session.receive(wire::submit(request(7, input_address, 1)));
  try {
    state.session.poll(state.bus, context);
  } catch (const Fault &fault) {
    const std::string_view message = fault.what();
    require(
        message.contains("seq=7") && message.contains("endpoint=0x81") &&
            message.contains("phase=data") && message.contains("mcu-pc=0x8123") &&
            message.contains("ccr=0x42") && message.contains("NISOF") && state.session.faulted() &&
            state.session.output().empty() && !state.publication->available(),
        "A real controller fault preserves endpoint/phase/PC/CCR and poisons its claimed session");
    reject([&] { state.session.poll(state.bus, context); }, "Faulted USB/IP session");
    return;
  }
  throw Fault("Reserved MCU USB controller fields produced a successful session result");
}
}

int main() {
  try {
    all_import_transfer_splits();
    all_truncated_eof_boundaries();
    malformed_suffix_and_partial_reply_identity();
    full_queue_cancellation_and_pending_disconnect();
    controller_failure_and_exclusive_import();
    std::cout
        << "Verified every fixed import/IN split and truncated EOF boundary, real MCU packets, "
           "partial-reply identities, full-queue UNLINK and strict controller failures.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
