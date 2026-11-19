// SPDX-License-Identifier: GPL-3.0-only
#include "io/usb.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {
using blofeld::vi::Usb;
constexpr std::uint16_t control_clock = 0x4000, control_enabled = 0xc000;
constexpr std::uint16_t status = 0x302, interrupts = 0x304, timestamp = 0x30a;
constexpr std::uint16_t configuration_status = 0x30c, select = 0x30e, data = 0x310;
constexpr std::uint16_t buffer = 0x380, other_buffer = 0x3c0;
constexpr std::uint16_t setup = 0x338;
constexpr std::uint16_t reset_status = 0xe040, interrupt_mask = 0x7807, all_bits = 0xffff;
constexpr std::uint16_t counts = 0x0800, alternates = 0x0900, count_mask = 0x0773;
constexpr std::uint16_t lower = 0x5678, upper = 0x1234;
constexpr std::uint8_t default_in = 0x80, default_out = 0;
constexpr std::uint16_t ep0_in = 0x314, ep0_out = 0x316;
constexpr std::uint16_t stall_enabled = 0x9000, transfer_irq = 0x1000;
constexpr std::uint16_t transfer_ack = 0x3000, primed = 0x5000, received = 0x7000;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void write(Usb &usb, const Usb::Write access) {
  require(usb.write(access, control_clock), "Write a defined USB register contract");
}

std::uint16_t read(const Usb &usb, const Usb::Read access) {
  std::uint16_t value = 0;
  require(usb.read(access, value), "Read a known USB register or initialized buffer byte");
  return value;
}

void reset_and_status() {
  Usb usb;
  require(read(usb, {.offset = status, .bytes = 2}) == reset_status &&
              read(usb, {.offset = timestamp, .bytes = 2}) == 0 &&
              read(usb, {.offset = configuration_status, .bytes = 2}) == 0 && !usb.pending(),
          "Reset has no transaction, enumeration, SOF or configuration request");
  write(usb, {.offset = status, .value = all_bits, .bytes = 2});
  require(read(usb, {.offset = status, .bytes = 2}) == reset_status,
          "W1C cannot fabricate flags or change read-only active endpoint/speed fields");
  write(usb, {.offset = interrupts, .value = all_bits, .bytes = 2});
  require(read(usb, {.offset = interrupts, .bytes = 2}) == interrupt_mask &&
              usb.interrupts_enabled(),
          "The interrupt mask retains only implemented enables");
  constexpr std::uint16_t status_vector = 0xffd4;
  require(usb.pending() == status_vector, "RESUMEIE with USD=0 requests the documented level IRQ");
  write(usb, {.offset = interrupts, .value = 0, .bytes = 2});
  write(usb, {.offset = configuration_status, .value = all_bits, .bytes = 2});
  require(!usb.pending() && read(usb, {.offset = configuration_status, .bytes = 2}) == 0,
          "DONEECRU without a host request cannot invent configuration success");
  std::uint16_t value = 0;
  require(!usb.read({.offset = setup}, value) && !usb.write({.offset = setup}, control_clock) &&
              !usb.read({.offset = status - 2}, value),
          "Unknown setup contents and another owner's UMCR are not swallowed");
}

void decoded_link_status() {
  Usb usb;
  constexpr std::uint16_t status_vector = 0xffd4;
  constexpr std::uint16_t frame_vector = 0xffbe;
  constexpr std::uint16_t link_irqs = 0x1804;
  constexpr std::uint16_t reset_detected = 0xe048;
  constexpr std::uint16_t reset_changed = 0x0800;
  constexpr std::uint16_t enumerated = 0xe0c0;
  constexpr std::uint16_t suspended_changed = 0xf0d0;
  constexpr std::uint16_t resumed_with_sof = 0xf0c4;
  write(usb, {.offset = interrupts, .value = link_irqs, .bytes = 2});
  require(!usb.observe_link(Usb::Link::reset, control_clock) &&
              !usb.receive_frame(1, control_enabled) && !usb.pending(),
          "An inactive link cannot latch a USB reset or a frame");
  require(usb.observe_link(Usb::Link::reset, control_enabled) &&
              read(usb, {.offset = status, .bytes = 2}) == (reset_detected | reset_changed) &&
              usb.pending() == status_vector && !usb.receive_frame(1, control_enabled),
          "A decoded reset sets URD and URSC and excludes packet reception");
  write(usb, {.offset = status, .value = reset_changed >> 8U});
  require(usb.observe_link(Usb::Link::reset, control_enabled) && !usb.pending() &&
              read(usb, {.offset = status, .bytes = 2}) == reset_detected,
          "Repeating the same asserted reset after W1C raises no new interrupt");
  require(usb.observe_link(Usb::Link::full_speed, control_enabled) &&
              read(usb, {.offset = status, .bytes = 2}) == (enumerated | reset_changed) &&
              usb.pending() == status_vector,
          "Reset release updates ENUMD and latches the opposite URSC edge");
  constexpr std::uint16_t last_frame = 0x07ff;
  require(usb.receive_frame(last_frame, control_enabled) &&
              !usb.receive_frame(last_frame + 1, control_enabled) &&
              read(usb, {.offset = timestamp, .bytes = 2}) == last_frame &&
              usb.pending() == status_vector,
          "UTSR keeps the last 11-bit frame and status has priority over SOF");
  write(usb, {.offset = status, .value = reset_changed >> 8U});
  require(usb.pending() == frame_vector, "Acknowledging the reset leaves the lower-priority SOF");
  write(usb, {.offset = status + 1, .value = 4});
  require(!usb.pending() && read(usb, {.offset = timestamp, .bytes = 2}) == last_frame &&
              usb.observe_link(Usb::Link::suspended, control_enabled) &&
              read(usb, {.offset = status, .bytes = 2}) == suspended_changed &&
              !usb.receive_frame(0, control_enabled),
          "SOF acknowledgement preserves UTSR and suspend closes the packet path");
  require(usb.observe_link(Usb::Link::full_speed, control_enabled) &&
              usb.receive_frame(0, control_enabled) &&
              read(usb, {.offset = timestamp, .bytes = 2}) == 0 &&
              read(usb, {.offset = status, .bytes = 2}) == resumed_with_sof,
          "Resume and frame rollover are separate received observations");
}

void firmware_setup() {
  Usb usb;
  constexpr Usb::Setup first{0x80, 6, 0, 1, 0, 0, 18, 0};
  constexpr Usb::Setup second{0xc1, 0x52, 0xab, 0x23, 0x17, 0x56, 9, 0};
  constexpr Usb::Setup automatic{0, 9, 1, 0, 0, 0, 0, 0};
  constexpr Usb::Setup optional{0, 7, 0, 1, 0, 0, 18, 0};
  constexpr std::uint16_t setup_vector = 0xffd2;
  constexpr std::uint16_t descriptor_support = 0x10;
  constexpr std::uint16_t pending_setup = 0xc1;
  constexpr std::uint16_t overrun = 0xc3;
  constexpr std::uint16_t input_get_descriptor = 0x8006;
  constexpr std::uint16_t request_length = 0x1200;
  constexpr std::uint16_t new_value = 0xab23;
  constexpr std::uint16_t new_index = 0x1756;
  require(!usb.receive_setup(first, control_enabled) &&
              usb.observe_link(Usb::Link::full_speed, control_enabled),
          "SETUP reception requires a live enumerated link");
  write(usb, {.offset = interrupts, .value = 3, .bytes = 2});
  write(usb, {.offset = ep0_in, .value = stall_enabled, .bytes = 2});
  write(usb, {.offset = ep0_out, .value = stall_enabled, .bytes = 2});
  require(usb.receive_setup(first, control_enabled) && usb.pending() == setup_vector &&
              read(usb, {.offset = status + 1}) == pending_setup &&
              read(usb, {.offset = ep0_in, .bytes = 2}) == transfer_irq &&
              read(usb, {.offset = ep0_out, .bytes = 2}) == transfer_irq,
          "One descriptor request clears both default stalls and raises the actual SETUP level");
  write(usb, {.offset = status, .value = 1, .bytes = 2});
  require(!usb.pending() && read(usb, {.offset = setup, .bytes = 2}) == input_get_descriptor &&
              read(usb, {.offset = setup + 2, .bytes = 2}) == 0x0001 &&
              read(usb, {.offset = setup + 6, .bytes = 2}) == request_length &&
              !usb.write({.offset = setup + 7, .value = 1}, control_enabled),
          "The original W1C-before-read sequence keeps the eight received bytes read-only");
  require(usb.receive_setup(first, control_enabled) && usb.receive_setup(second, control_enabled) &&
              read(usb, {.offset = status + 1}) == overrun && usb.pending() == setup_vector,
          "A second unacknowledged request replaces the buffer and latches SETOVR");
  require(!usb.receive_setup(automatic, control_enabled) &&
              !usb.receive_setup(optional, control_enabled) &&
              read(usb, {.offset = setup + 2, .bytes = 2}) == new_value &&
              read(usb, {.offset = setup + 4, .bytes = 2}) == new_index,
          "Automatic and disabled optional requests leave a pending firmware transfer intact");
  write(usb, {.offset = status + 1, .value = 2});
  require(usb.pending() == setup_vector && read(usb, {.offset = status + 1}) == pending_setup &&
              usb.receive_setup(optional, control_enabled | descriptor_support) &&
              read(usb, {.offset = setup, .bytes = 2}) == 7,
          "SETOVR and SETUP have separate W1C flags and optional forwarding honors UMCR");
}

void control_transfers() {
  Usb usb;
  constexpr std::uint16_t in_vector = 0xffce;
  constexpr std::uint16_t out_vector = 0xffcc;
  constexpr Usb::Setup descriptor{0x80, 6, 0, 1, 0, 0, 2, 0};
  constexpr std::array<std::uint8_t, 2> reply{0x11, 0x22};
  constexpr std::array<std::uint8_t, 2> host_data{0xe1, 0xd2};
  constexpr std::uint16_t both_buffer_windows = 0x0110;
  constexpr std::uint16_t overlong_packet = 65;
  constexpr std::uint16_t reply_word = 0x1122;
  constexpr std::uint16_t host_word = 0xe1d2;
  constexpr std::uint16_t reply_length = reply.size();
  constexpr std::uint16_t host_length = host_data.size();
  require(usb.observe_link(Usb::Link::full_speed, control_enabled) &&
              usb.receive_setup(descriptor, control_enabled) &&
              usb.input(default_in, control_enabled).transfer == Usb::Transfer::unavailable,
          "Endpoint zero needs the firmware's committed control maximum packet size");
  write(usb, {.offset = select, .value = both_buffer_windows, .bytes = 2});
  write(usb, {.offset = data, .value = 0x0200, .bytes = 2});
  write(usb, {.offset = data + 2, .value = 0, .bytes = 2});
  require(usb.commit_mapping() &&
              usb.input(default_in, control_enabled).transfer == Usb::Transfer::nak,
          "IN without DVALID returns NAK while the firmware prepares the reply");
  write(usb, {.offset = ep0_in, .value = primed | overlong_packet, .bytes = 2});
  require(usb.input(default_in, control_enabled).transfer == Usb::Transfer::unavailable &&
              !usb.acknowledge_input(default_in, control_enabled),
          "A declared length beyond the configured maximum cannot retire an IN transfer");
  write(usb, {.offset = ep0_in, .value = primed | reply_length, .bytes = 2});
  require(usb.input(default_in, control_enabled).transfer == Usb::Transfer::unavailable &&
              !usb.acknowledge_input(default_in, control_enabled) && !usb.pending(),
          "Unknown bytes cannot leak through a primed IN or latch TFRC");
  write(usb, {.offset = buffer, .value = reply_word, .bytes = 2});
  const auto payload = usb.input(default_in, control_enabled);
  require(payload.transfer == Usb::Transfer::ready && std::ranges::equal(payload.bytes, reply) &&
              payload.bytes.data() == usb.regions(0)[0].bytes.data() && !usb.pending() &&
              read(usb, {.offset = ep0_in, .bytes = 2}) == (primed | reply_length),
          "Reading the offer borrows the exact RAM and leaves it pending until host ACK");
  require(usb.acknowledge_input(default_in, control_enabled) && usb.pending() == in_vector &&
              read(usb, {.offset = ep0_in, .bytes = 2}) == (transfer_ack | reply_length) &&
              !usb.acknowledge_input(default_in, control_enabled),
          "Only one acknowledged IN clears DVALID, sets TFRC and requests the IN interrupt");
  write(usb, {.offset = ep0_in, .value = transfer_ack >> 8U});

  write(usb, {.offset = ep0_out, .value = stall_enabled, .bytes = 2});
  require(usb.output(default_out, host_data, control_enabled) == Usb::Transfer::stall &&
              !usb.pending(),
          "A stalled OUT cannot write packet RAM or interrupt firmware");
  write(usb, {.offset = ep0_out, .value = transfer_irq, .bytes = 2});
  require(usb.output(default_out, host_data, control_enabled) == Usb::Transfer::ready &&
              usb.pending() == out_vector &&
              read(usb, {.offset = ep0_out, .bytes = 2}) == (received | host_length) &&
              read(usb, {.offset = other_buffer, .bytes = 2}) == host_word &&
              usb.output(default_out, {}, control_enabled) == Usb::Transfer::nak,
          "OUT completion publishes the exact byte count and prevents overwrite until consumed");
  write(usb, {.offset = ep0_out, .value = transfer_ack, .bytes = 2});
  require(usb.output(default_out, {}, control_enabled) == Usb::Transfer::ready &&
              read(usb, {.offset = ep0_out, .bytes = 2}) == received,
          "A real zero-length OUT uses the documented transfer and size fields");
  write(usb, {.offset = ep0_in, .value = primed, .bytes = 2});
  require(usb.receive_setup(descriptor, control_enabled) &&
              usb.input(default_in, control_enabled).transfer == Usb::Transfer::nak &&
              read(usb, {.offset = ep0_out, .bytes = 2}) == transfer_ack,
          "A new SETUP cancels priming for the old control transfer and preserves its TFRC flag");
}

void automatic_configuration() {
  Usb usb;
  constexpr Usb::Setup request{0, 9, 1, 0, 0, 0, 0, 0};
  constexpr Usb::Setup bad_value{0, 9, 2, 0, 0, 0, 0, 0};
  constexpr Usb::Setup bad_length{0, 9, 1, 0, 0, 0, 1, 0};
  constexpr std::uint16_t enabled_irqs = 0x2003;
  constexpr std::uint16_t config_vector = 0xffd0;
  constexpr std::uint16_t requested = 0xa100;
  constexpr std::uint16_t confirmed = 0x8100;
  constexpr std::uint16_t done_byte = 0xb1;
  write(usb, {.offset = interrupts, .value = enabled_irqs, .bytes = 2});
  write(usb, {.offset = select, .value = counts, .bytes = 2});
  write(usb, {.offset = data, .value = 0, .bytes = 2});
  write(usb, {.offset = data + 2, .value = 1, .bytes = 2});
  require(usb.observe_link(Usb::Link::full_speed, control_enabled) &&
              !usb.receive_setup(bad_value, control_enabled) &&
              !usb.receive_setup(bad_length, control_enabled) && !usb.pending(),
          "Automatic SET_CONFIGURATION validates the documented limit and no-data request");
  require(usb.receive_setup(request, control_enabled) && usb.pending() == config_vector &&
              read(usb, {.offset = configuration_status, .bytes = 2}) == requested &&
              usb.input(default_in, control_enabled).transfer == Usb::Transfer::nak &&
              !usb.acknowledge_input(default_in, control_enabled),
          "Accepted configuration notifies firmware and NAKs status IN until DONEECRU");
  std::uint16_t value = 0;
  require(!usb.read({.offset = setup}, value) && !usb.receive_setup(request, control_enabled),
          "Automatic requests keep USTB unknown and reject overlap before completion");
  write(usb, {.offset = ep0_in, .value = primed, .bytes = 2});
  require(usb.input(default_in, control_enabled).transfer == Usb::Transfer::nak,
          "DVALID cannot bypass the configuration-IRQ acknowledgement");
  write(usb, {.offset = configuration_status, .value = done_byte});
  const auto completion = usb.input(default_in, control_enabled);
  require(!usb.pending() && read(usb, {.offset = configuration_status, .bytes = 2}) == confirmed &&
              completion.transfer == Usb::Transfer::ready && completion.bytes.empty() &&
              usb.acknowledge_input(default_in, control_enabled) &&
              !usb.acknowledge_input(default_in, control_enabled) &&
              read(usb, {.offset = ep0_in, .bytes = 2}) == primed,
          "The controller completes its own status stage after the firmware ACK without consuming "
          "local RAM");
  require(usb.observe_link(Usb::Link::reset, control_enabled) &&
              read(usb, {.offset = configuration_status, .bytes = 2}) == 0,
          "An actual USB reset returns the controller to its unconfigured state");
}

void local_bulk_endpoints() {
  Usb usb;
  constexpr Usb::Setup configure{0, 9, 1, 0, 0, 0, 0, 0};
  constexpr std::uint16_t in_select = 0x0232;
  constexpr std::uint16_t out_select = 0x0332;
  constexpr std::uint16_t in_csr = 0x318;
  constexpr std::uint16_t out_csr = 0x31a;
  constexpr std::uint16_t in_vector = 0xffca;
  constexpr std::uint16_t out_vector = 0xffc8;
  constexpr std::uint8_t in = 0x82;
  constexpr std::uint8_t out = 3;
  constexpr std::uint16_t in_high = 0x8100;
  constexpr std::uint16_t in_low = 0x00d2;
  constexpr std::uint16_t out_high = 0x0080;
  constexpr std::uint16_t out_low = 0x00c3;
  constexpr std::uint16_t done_byte = 0xb1;
  constexpr std::array<std::uint8_t, 4> input{0x09, 0x90, 0x3c, 0x64};
  constexpr std::array<std::uint8_t, 4> output{0x08, 0x80, 0x3c, 0};
  constexpr std::uint16_t size = input.size();
  constexpr std::array<std::uint8_t, 17> overlong{};
  write(usb, {.offset = select, .value = counts, .bytes = 2});
  write(usb, {.offset = data, .value = 0, .bytes = 2});
  write(usb, {.offset = data + 2, .value = 1, .bytes = 2});
  write(usb, {.offset = select, .value = in_select, .bytes = 2});
  write(usb, {.offset = data, .value = in_high, .bytes = 2});
  write(usb, {.offset = data + 2, .value = in_low, .bytes = 2});
  write(usb, {.offset = select, .value = out_select, .bytes = 2});
  write(usb, {.offset = data, .value = out_high, .bytes = 2});
  write(usb, {.offset = data + 2, .value = out_low, .bytes = 2});
  require(usb.commit_mapping() && usb.observe_link(Usb::Link::full_speed, control_enabled) &&
              usb.input(in, control_enabled).transfer == Usb::Transfer::unavailable &&
              usb.receive_setup(configure, control_enabled),
          "A configured register bank cannot receive bulk traffic before host configuration");
  write(usb, {.offset = configuration_status, .value = done_byte});
  require(usb.output(out, output, control_enabled) == Usb::Transfer::unavailable &&
              usb.acknowledge_input(default_in, control_enabled),
          "The host must finish configuration before using the new bulk endpoints");

  auto windows = usb.regions(0);
  write(usb, {.offset = in_csr, .value = primed | size, .bytes = 2});
  write(usb, {.offset = out_csr, .value = transfer_irq, .bytes = 2});
  require(usb.input(out, control_enabled).transfer == Usb::Transfer::unavailable &&
              usb.output(in, output, control_enabled) == Usb::Transfer::unavailable &&
              usb.output(out, overlong, control_enabled) == Usb::Transfer::unavailable &&
              usb.input(in, control_enabled).transfer == Usb::Transfer::unavailable &&
              !usb.pending(),
          "Direction, maximum length and initialized bytes gate bulk transfers");
  std::ranges::copy(input, windows[0].bytes.begin());
  std::ranges::fill(windows[0].known.first(input.size()), std::uint8_t{1});
  const auto payload = usb.input(in, control_enabled);
  require(payload.transfer == Usb::Transfer::ready && std::ranges::equal(payload.bytes, input) &&
              payload.bytes.data() == windows[0].bytes.data() && !usb.pending() &&
              read(usb, {.offset = data, .bytes = 2}) == out_high,
          "Logical bulk IN borrows its own physical RAM and leaves it pending until ACK");
  require(
      usb.output(out, output, control_enabled) == Usb::Transfer::ready &&
          std::ranges::equal(windows[1].bytes.first(output.size()), output) &&
          windows[1].known[output.size()] == 0 &&
          read(usb, {.offset = out_csr, .bytes = 2}) == (received | size) &&
          usb.output(out, {}, control_enabled) == Usb::Transfer::nak && usb.pending() == out_vector,
      "Bulk OUT grants exactly the received bytes and holds its buffer until firmware consumes it");
  require(usb.acknowledge_input(in, control_enabled) && usb.pending() == in_vector &&
              usb.input(in, control_enabled).transfer == Usb::Transfer::nak,
          "Acknowledged bulk IN clears DVALID and takes priority over the pending OUT completion");
  write(usb, {.offset = in_csr, .value = transfer_ack, .bytes = 2});
  require(usb.pending() == out_vector, "An IN W1C preserves the independent OUT completion");
  write(usb, {.offset = out_csr, .value = transfer_ack, .bytes = 2});
  require(!usb.pending(), "Firmware acknowledges each completion independently");

  write(usb, {.offset = in_csr, .value = stall_enabled, .bytes = 2});
  require(usb.input(in, control_enabled).transfer == Usb::Transfer::stall &&
              read(usb, {.offset = in_csr, .bytes = 2}) == transfer_irq &&
              usb.input(in, control_enabled).transfer == Usb::Transfer::stall && !usb.pending(),
          "A bulk STALL command self-clears in the CSR and retains the endpoint halt");
  require(usb.observe_link(Usb::Link::reset, control_enabled) &&
              usb.observe_link(Usb::Link::full_speed, control_enabled) &&
              usb.receive_setup(configure, control_enabled),
          "Bus reset ends the old session and retains the firmware-programmed routing fields");
  write(usb, {.offset = configuration_status, .value = done_byte});
  require(usb.acknowledge_input(default_in, control_enabled) &&
              usb.input(in, control_enabled).transfer == Usb::Transfer::nak,
          "Reconfiguration no longer observes the previous bulk halt");
  write(usb, {.offset = data, .value = in_high, .bytes = 2});
  write(usb, {.offset = data + 2, .value = in_low, .bytes = 2});
  require(usb.input(in, control_enabled).transfer == Usb::Transfer::unavailable,
          "Duplicate active logical endpoint addresses are ambiguous");
}

void configuration_commits() {
  Usb usb;
  write(usb, {.offset = select, .value = 0x0100, .bytes = 2});
  write(usb, {.offset = data, .value = upper, .bytes = 2});
  require(read(usb, {.offset = data, .bytes = 2}) == 0 &&
              read(usb, {.offset = data + 2, .bytes = 2}) == 0,
          "The first word stages data without changing the selected 32-bit register");
  write(usb, {.offset = data + 2, .value = lower, .bytes = 2});
  require(read(usb, {.offset = data, .bytes = 2}) == upper &&
              read(usb, {.offset = data + 2, .bytes = 2}) == lower &&
              read(usb, {.offset = data}) == (upper >> 8U),
          "The second word commits both halves with big-endian readback");
  require(!usb.write({.offset = data, .value = 0}, control_clock) &&
              !usb.write({.offset = data + 2, .value = 1, .bytes = 2}, control_clock),
          "Byte writes and a missing staged high word cannot pretend a valid commit");

  write(usb, {.offset = select, .value = counts, .bytes = 2});
  write(usb, {.offset = data, .value = all_bits, .bytes = 2});
  write(usb, {.offset = data + 2, .value = all_bits, .bytes = 2});
  require(read(usb, {.offset = data, .bytes = 2}) == 0 &&
              read(usb, {.offset = data + 2, .bytes = 2}) == count_mask,
          "UNCIR has the documented reserved-bit mask");
  require(usb.write({.offset = data, .value = 0, .bytes = 2}, control_enabled) &&
              usb.write({.offset = data + 2, .value = 1, .bytes = 2}, control_enabled) &&
              read(usb, {.offset = data + 2, .bytes = 2}) == count_mask,
          "UNCIR writes while MEN=1 are ignored rather than changing active limits");
  write(usb, {.offset = select, .value = alternates, .bytes = 2});
  write(usb, {.offset = data, .value = upper, .bytes = 2});
  write(usb, {.offset = data + 2, .value = lower, .bytes = 2});
  require(read(usb, {.offset = data, .bytes = 2}) == upper &&
              read(usb, {.offset = data + 2, .bytes = 2}) == lower,
          "UNASR retains all eight alternate-setting nibbles");

  constexpr std::uint16_t endpoint = 0x0200;
  constexpr std::uint16_t noniso_sof = 0xc100;
  constexpr std::uint16_t bulk_in = 0x0052;
  write(usb, {.offset = select, .value = endpoint, .bytes = 2});
  require(usb.write({.offset = data, .value = noniso_sof, .bytes = 2}, control_enabled) &&
              usb.write({.offset = data + 2, .value = bulk_in, .bytes = 2}, control_enabled) &&
              read(usb, {.offset = data, .bytes = 2}) == noniso_sof &&
              read(usb, {.offset = data + 2, .bytes = 2}) == bulk_in,
          "Endpoint configuration stays writable with MEN=1 and preserves non-isochronous NISOF");
}

void direct_buffer_windows() {
  Usb usb;
  auto windows = usb.regions(0);
  require(windows[0].first == buffer && windows[1].first == other_buffer &&
              windows[0].bytes.data() == windows[1].bytes.data() && windows[0].known[0] == 0,
          "Reset windows alias the same borrowed RAM, with unknown initial contents");
  constexpr std::uint8_t first = 0x5a;
  constexpr std::uint8_t second = 0xa5;
  windows[0].bytes[0] = first;
  windows[0].known[0] = 1;
  require(read(usb, {.offset = other_buffer}) == first,
          "Direct memory and register fallback observe the same backing storage");
  constexpr std::uint16_t initial_selection = 0x23;
  constexpr std::uint16_t reversed_selection = 0x32;
  write(usb, {.offset = select + 1, .value = initial_selection});
  require(usb.mapping_pending() && usb.regions(0)[0].bytes.data() == windows[0].bytes.data() &&
              usb.commit_mapping() && !usb.mapping_pending() && !usb.commit_mapping(),
          "Bank changes require exactly one retired-instruction rebind");
  write(usb, {.offset = buffer, .value = first});
  write(usb, {.offset = other_buffer, .value = second});
  write(usb, {.offset = select + 1, .value = reversed_selection});
  require(usb.commit_mapping(), "Commit the reversed buffer mapping");
  windows = usb.regions(0);
  require(windows[0].bytes[0] == second && windows[1].bytes[0] == first &&
              read(usb, {.offset = buffer}) == second &&
              read(usb, {.offset = other_buffer}) == first,
          "Window changes select RAM instead of copying or clearing payloads");
  std::uint16_t value = 0;
  require(!usb.read({.offset = buffer + 1}, value), "Unwritten buffer bytes remain unknown");
  constexpr std::uint16_t reserved_windows = 0x77;
  write(usb, {.offset = select + 1, .value = reserved_windows});
  require(usb.commit_mapping() && usb.regions(0)[0].bytes.empty() &&
              !usb.read({.offset = buffer}, value) && !usb.write({.offset = buffer}, control_clock),
          "Reserved buffer selectors withdraw grants and fail actual accesses");
  write(usb, {.offset = select + 1, .value = initial_selection});
  require(usb.commit_mapping(), "Restore the valid bank selection");
  constexpr std::uint16_t relocated_first = 0x780;
  constexpr std::uint16_t relocated_second = 0x7c0;
  const auto relocated = usb.regions(0x400);
  require(relocated[0].first == relocated_first && relocated[1].first == relocated_second &&
              relocated[0].bytes[0] == first,
          "INITRG relocation moves the views while preserving the selected buffers");
}

void endpoint_controls() {
  Usb usb;
  constexpr std::uint16_t local = 0x314;
  constexpr std::uint16_t auxiliary_out = 0x31c;
  constexpr std::uint16_t out_status = 0x31e;
  constexpr std::uint16_t in_status = 0x322;
  constexpr std::uint16_t local_mask = 0xd07f;
  constexpr std::uint16_t auxiliary_mask = 0xdfff;
  constexpr std::uint16_t out_mask = 0x500f;
  constexpr std::uint16_t in_mask = 0x100f;
  write(usb, {.offset = local, .value = all_bits, .bytes = 2});
  require(read(usb, {.offset = local, .bytes = 2}) == local_mask && !usb.pending() &&
              usb.interrupts_enabled(),
          "Priming DVALID and TCIE cannot manufacture a completed transfer or IRQ");
  write(usb, {.offset = auxiliary_out, .value = all_bits, .bytes = 2});
  require(read(usb, {.offset = auxiliary_out, .bytes = 2}) == auxiliary_mask,
          "The IQUE control word has its own size and continue-on-error fields");
  write(usb, {.offset = out_status, .value = all_bits, .bytes = 2});
  write(usb, {.offset = in_status, .value = all_bits, .bytes = 2});
  require(read(usb, {.offset = out_status, .bytes = 2}) == out_mask &&
              read(usb, {.offset = in_status, .bytes = 2}) == in_mask && !usb.pending(),
          "Error/short-packet flags are W1C; unavailable IQUE buffers are not invented");
}
}

int main() try {
  reset_and_status();
  decoded_link_status();
  firmware_setup();
  control_transfers();
  automatic_configuration();
  local_bulk_endpoints();
  configuration_commits();
  direct_buffer_windows();
  endpoint_controls();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
