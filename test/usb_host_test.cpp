// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_usb.hpp"
#include "blofeld/usb_host.hpp"
#include "blofeld/usb_transfer.hpp"
#include "usb_endpoint_reset_fixture.hpp"
#include "usb_host_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::usb_host;
constexpr std::array<std::uint8_t, 32768> own_flash{};
constexpr std::array<std::uint8_t, 3> payload{0x19, 0x37, 0xa5};

void test_bulk_output_nak_and_zero_packet() {
  McuBus bus(own_flash);
  prepare(bus);
  UsbHostQueue queue;
  auto r = request(1, output_address, 10);
  r.output = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  queue.submit(r);
  require(queue.poll(bus, context).empty() && queue.pending() == 1 &&
              word(bus, output_control) == (output_valid | 8U),
          "First OUT packet is ACKed but does not complete a longer URB");
  require(received(bus, UsbBuffer::endpoint3, 8) ==
              std::vector<std::uint8_t>({0, 1, 2, 3, 4, 5, 6, 7}),
          "Eight independent own bytes actually reach the local endpoint");
  require(queue.poll(bus, context).empty() && word(bus, output_control) == (output_valid | 8U),
          "An occupied device buffer NAKs without duplicate delivery or host completion");
  release(bus, UsbBuffer::endpoint3);
  const auto done = queue.poll(bus, context);
  require(done.size() == 1 && done[0].sequence == 1 && done[0].transferred == 10 &&
              done[0].status == UsbTransferStatus::success &&
              received(bus, UsbBuffer::endpoint3, 2) == std::vector<std::uint8_t>({8, 9}),
          "Second DATA1 packet completes exactly ten accepted OUT bytes");
  release(bus, UsbBuffer::endpoint3);
  r = request(2, output_address, 8);
  r.output = {8, 7, 6, 5, 4, 3, 2, 1};
  r.zero_packet = true;
  queue.submit(r);
  require(queue.poll(bus, context).empty(), "ZERO_PACKET keeps a full-packet OUT transfer pending");
  require(queue.poll(bus, context).empty(), "Its extra zero packet also respects device NAK");
  release(bus, UsbBuffer::endpoint3);
  const auto zero = queue.poll(bus, context);
  require(zero.size() == 1 && zero[0].transferred == 8 && word(bus, output_control) == output_valid,
          "An actual zero-length packet, not another payload byte, completes ZERO_PACKET");
}

void test_input_short_fifo_and_stall() {
  McuBus bus(own_flash);
  prepare(bus);
  UsbHostQueue queue;
  queue.submit(request(1, input_address, 16));
  queue.submit(request(2, input_address, 2));
  require(queue.poll(bus, context).empty() && queue.pending() == 2,
          "No firmware bytes means pending IN requests, not zero-filled success");
  prime(bus, UsbBuffer::endpoint2, payload);
  const auto first = queue.poll(bus, context);
  require(first.size() == 1 &&
              first[0].input == std::vector<std::uint8_t>(payload.begin(), payload.end()) &&
              first[0].transferred == 3 && first[0].status == UsbTransferStatus::success &&
              queue.pending() == 1,
          "Short IN completes its first URB only; younger same-endpoint requests stay queued");
  require(word(bus, input_control) == (acknowledged | 3U),
          "Host acceptance alone supplies the real IN ACK and TFRC");
  constexpr std::array<std::uint8_t, 2> second{0x42, 0x24};
  prime(bus, UsbBuffer::endpoint2, second);
  const auto next = queue.poll(bus, context);
  require(next.size() == 1 && next[0].sequence == 2 &&
              next[0].input == std::vector<std::uint8_t>(second.begin(), second.end()),
          "The next request receives its own DATA1 bytes in FIFO order");
  auto r = request(3, input_address, 16);
  r.short_not_ok = true;
  queue.submit(r);
  prime(bus, UsbBuffer::endpoint2, payload);
  const auto short_error = queue.poll(bus, context);
  require(short_error.size() == 1 && short_error[0].status == UsbTransferStatus::short_packet &&
              short_error[0].transferred == 3,
          "SHORT_NOT_OK reports an error while retaining the real received prefix");
  queue.submit(request(4, input_address, 4));
  constexpr unsigned stall_interrupt = 0x90;
  bus.write(input_control, stall_interrupt);
  const auto stall = queue.poll(bus, context);
  require(stall.size() == 1 && stall[0].status == UsbTransferStatus::stalled &&
              stall[0].transferred == 0,
          "A native endpoint STALL is returned as an actual failed transfer");
}

void test_control_both_directions() {
  McuBus bus(own_flash);
  prepare(bus);
  UsbHostQueue queue;
  auto r = request(1, control_input, 11);
  r.setup = {0x80, 6, 0, 1, 0, 0, 11, 0};
  queue.submit(r);
  require(queue.poll(bus, context).empty() && (word(bus, status) & 1U) != 0,
          "Control setup reaches the real peripheral before the host can read any data");
  constexpr std::array<std::uint8_t, 8> first{8, 7, 6, 5, 4, 3, 2, 1};
  prime(bus, UsbBuffer::control_in, first);
  require(queue.poll(bus, context).empty(),
          "A full EP0 packet is insufficient for an eleven-byte control read");
  prime(bus, UsbBuffer::control_in, payload);
  require(queue.poll(bus, context).empty(),
          "Complete data still requires a separate actual status stage");
  const auto read = queue.poll(bus, context);
  const std::vector<std::uint8_t> expected_input{8, 7, 6, 5, 4, 3, 2, 1, 0x19, 0x37, 0xa5};
  require(read.size() == 1 && read[0].input == expected_input &&
              word(bus, endpoint_base + 2) == output_valid,
          "Control read completes only after a genuine DATA1 zero-length OUT status packet");
  release(bus, UsbBuffer::control_out);
  r = request(2, 0, 3);
  constexpr unsigned class_host_to_interface = 0x21;
  r.setup = {class_host_to_interface, 9, 0, 0, 0, 0, 3, 0};
  r.output.assign(payload.begin(), payload.end());
  queue.submit(r);
  require(queue.poll(bus, context).empty() && queue.poll(bus, context).empty(),
          "Control write setup and OUT data are separate packets");
  require(received(bus, UsbBuffer::control_out, 3) ==
              std::vector<std::uint8_t>(payload.begin(), payload.end()),
          "Control OUT retains its exact data bytes");
  require(queue.poll(bus, context).empty(), "Missing device status bytes NAK a control write");
  prime(bus, UsbBuffer::control_in, {});
  const auto written = queue.poll(bus, context);
  require(written.size() == 1 && written[0].transferred == 3 && written[0].input.empty(),
          "Actual DATA1 IN status completes the control write");
}

void test_cancel_snapshot_and_unknowns() {
  McuBus bus(own_flash);
  prepare(bus);
  UsbHostQueue queue;
  auto r = request(1, output_address, 10);
  r.output = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  queue.submit(r);
  require(queue.poll(bus, context).empty(), "A partial OUT prefix precedes cancellation");
  const auto before = word(bus, output_control);
  require(queue.cancel(1) && !queue.cancel(1) && queue.poll(bus, context).empty() &&
              word(bus, output_control) == before,
          "Cancellation emits no late completion and cannot undo already accepted device bytes");
  release(bus, UsbBuffer::endpoint3);
  r = request(2, output_address, 3);
  r.output.assign(payload.begin(), payload.end());
  queue.submit(r);
  auto copy = queue;
  auto peer = bus.clone();
  const auto a = queue.poll(bus, context);
  const auto b = copy.poll(*peer, context);
  require(a == b && a.size() == 1 &&
              received(*peer, UsbBuffer::endpoint3, 3) ==
                  std::vector<std::uint8_t>(payload.begin(), payload.end()),
          "Joint queue/controller replay retains PID, prefix and independent endpoint bytes");
  constexpr unsigned reserved_description = 0x804000d1;
  test::endpoint_reset::configure(bus, {.bank = 2, .value = reserved_description});
  queue.submit(request(3, input_address, 1));
  reject([&] { static_cast<void>(queue.poll(bus, context)); }, "NISOF");
  require(queue.faulted() && queue.pending() == 1,
          "Unknown endpoint behavior remains a fatal pending request, not successful data");
  reject([&] { static_cast<void>(queue.poll(bus, context)); }, "new reset/session");
}

void test_invalid_admission_and_context() {
  McuBus bus(own_flash);
  prepare(bus);
  UsbHostQueue queue;
  auto r = request(1, output_address, 4);
  reject([&] { queue.submit(r); }, "payload");
  require(queue.pending() == 0, "An incomplete OUT request cannot alter the queue");
  r = request(1, input_address, 4);
  queue.submit(r);
  reject([&] { queue.submit(r); }, "Duplicate");
  queue.reset();
  for (unsigned i = 0; i < maximum_usb_pending; ++i) { queue.submit(request(i, input_address, 1)); }
  reject([&] { queue.submit(request(maximum_usb_pending, input_address, 1)); }, "excessive");
  require(queue.pending() == maximum_usb_pending,
          "Bounded pending admission cannot partially enqueue the extra request");
  queue.reset();
  r = request(2, control_input, 1);
  r.setup = {0x80, 8, 0, 0, 0, 0, 1, 0};
  queue.submit(r);
  try {
    static_cast<void>(queue.poll(bus, context));
  } catch (const Fault &fault) {
    const std::string_view text = fault.what();
    require(
        text.contains("seq=2") && text.contains("endpoint=0x80") && text.contains("phase=setup") &&
            text.contains("mcu-pc=0x8123") && text.contains("ccr=0x42") &&
            text.contains("Unimplemented USB automatic standard request"),
        "Unsupported automatic requests preserve endpoint, phase and actual MCU execution context");
    require(queue.faulted(), "The failed hardware path poisons further polling");
    return;
  }
  throw Fault("Unsupported standard request reported host success");
}

void test_no_data_control_and_reset_unknownness() {
  McuBus bus(own_flash);
  prepare(bus);
  UsbHostQueue queue;
  auto r = request(1, 0, 0);
  r.setup = {0x40, 1, 0, 0, 0, 0, 0, 0};
  queue.submit(r);
  require(queue.poll(bus, context).empty() && queue.poll(bus, context).empty(),
          "No-data control requests still wait for the actual IN status handshake");
  prime(bus, UsbBuffer::control_in, {});
  const auto done = queue.poll(bus, context);
  require(done.size() == 1 && done[0].transferred == 0,
          "A real zero-length DATA1 completes no-data control");
  r = request(2, 0, 3);
  r.setup = {0, 7, 0, 0, 0, 0, 3, 0};
  r.output.assign(payload.begin(), payload.end());
  queue.submit(r);
  require(queue.poll(bus, context).empty(),
          "Disabled SET_DESCRIPTOR still accepts its SETUP token");
  const auto stalled = queue.poll(bus, context);
  require(stalled.size() == 1 && stalled[0].status == UsbTransferStatus::stalled &&
              stalled[0].transferred == 0,
          "The actual peripheral rejects the disabled optional request's later data stage");
  test::endpoint_reset::reset_command(bus);
  test::endpoint_reset::enable(bus);
  r = request(3, output_address, 3);
  r.output.assign(payload.begin(), payload.end());
  queue.submit(r);
  reject([&] { static_cast<void>(queue.poll(bus, context)); },
         "Unknown USB OUT DATA PID after endpoint reset");
  require(queue.faulted(), "Endpoint reset cannot be repaired by invented host PID state");
}
}

int main() {
  try {
    test_bulk_output_nak_and_zero_packet();
    test_input_short_fifo_and_stall();
    test_control_both_directions();
    test_cancel_snapshot_and_unknowns();
    test_invalid_admission_and_context();
    test_no_data_control_and_reset_unknownness();
    std::cout << "Verified MCU-backed host control/bulk packet ownership, NAK/ACK/status, FIFO, "
                 "cancellation and replay; no host attachment claim.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
