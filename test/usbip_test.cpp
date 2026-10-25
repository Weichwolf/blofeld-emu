// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "usb/usb_transfer.hpp"
#include "usb/usbip.hpp"
#include "usbip_fixture.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::usbip_fixture;
constexpr std::array<std::uint8_t, 3> payload{0x19, 0x37, 0xa5};

void require(const bool value, const std::string_view reason) {
  if (!value) { throw Fault(std::string(reason)); }
}

void reject(const auto &operation) {
  try {
    operation();
  } catch (const Fault &) { return; }
  throw Fault("Invalid USB/IP input reported success");
}

UsbTransferRequest own_output() {
  UsbTransferRequest request;
  constexpr unsigned sequence = 0x10203040;
  request.sequence = sequence;
  request.address = 2;
  request.length = 3;
  request.output.assign(payload.begin(), payload.end());
  return request;
}

void verify_output(const UsbipCommand &command) {
  const auto *r = std::get_if<UsbTransferRequest>(&command);
  constexpr unsigned sequence = 0x10203040;
  require(
      r != nullptr && r->sequence == sequence && r->address == 2 && r->length == 3 &&
          r->output == std::vector<std::uint8_t>(payload.begin(), payload.end()),
      "Network-order USB/IP words retain independent sequence/address/length and exact OUT bytes");
}

void test_fragmentation_and_eof() {
  const auto frame = submit(own_output());
  for (std::size_t split = 0; split < frame.size(); ++split) {
    UsbipDecoder decoder(device);
    decoder.feed(std::span(frame).first(split));
    require(!decoder.next(), "A partial OUT frame cannot be delivered");
    decoder.feed(std::span(frame).subspan(split));
    const auto command = decoder.next();
    require(command.has_value(), "Every fixed TCP split reconstructs one complete request");
    if (command) { verify_output(*command); }
    require(!decoder.next(), "No duplicate request follows a complete frame");
    decoder.finish();
  }
  for (std::size_t cut = 1; cut < frame.size(); ++cut) {
    UsbipDecoder decoder(device);
    decoder.feed(std::span(frame).first(cut));
    require(!decoder.next(), "Truncated header/payload remains pending until EOF");
    reject([&] { decoder.finish(); });
    require(decoder.faulted(), "Truncated EOF fails the connection");
    reject([&] { decoder.feed(frame); });
    reject([&] { static_cast<void>(decoder.next()); });
  }
}

void test_coalescing_setup_and_unlink() {
  auto data = submit(own_output());
  UsbTransferRequest r;
  constexpr unsigned control_length = 258;
  r.sequence = 7;
  r.address = 0x80;
  r.length = control_length;
  r.setup = {0x80, 6, 0, 1, 0, 0, 2, 1};
  auto control = submit(r);
  constexpr auto unused_schedule = std::numeric_limits<std::uint32_t>::max();
  set(control, Slot::frame, unused_schedule);
  set(control, Slot::packets, unused_schedule);
  data.insert(data.end(), control.begin(), control.end());
  const auto cancel = unlink(8, 7);
  data.insert(data.end(), cancel.begin(), cancel.end());
  UsbipDecoder decoder(device);
  decoder.feed(data);
  const auto first = decoder.next();
  require(first.has_value(), "Coalesced OUT decodes first");
  if (first) { verify_output(*first); }
  const auto next = decoder.next();
  const auto *input = next ? std::get_if<UsbTransferRequest>(&*next) : nullptr;
  require(input != nullptr && input->address == 0x80 && input->length == control_length &&
              input->setup == r.setup && input->output.empty(),
          "IN carries no payload; its eight setup bytes retain USB little-endian fields unchanged");
  const auto third = decoder.next();
  const auto *remove = third ? std::get_if<UsbUnlinkRequest>(&*third) : nullptr;
  require(remove != nullptr && remove->sequence == 8 && remove->target == 7 && !decoder.next(),
          "UNLINK retains both sequence identities after coalesced submissions");
  decoder.finish();
}

void test_invalid_fields_and_bounds() {
  const auto correct = submit(own_output());
  const std::vector<std::pair<Slot, unsigned>> controls{{Slot::command, 3},
                                                        {Slot::device, device + 1},
                                                        {Slot::direction, 2},
                                                        {Slot::endpoint, 16},
                                                        {Slot::flags, 2},
                                                        {Slot::length, 0xffffffff},
                                                        {Slot::frame, 1},
                                                        {Slot::packets, 1},
                                                        {Slot::setup0, 1}};
  for (const auto &[slot, value] : controls) {
    auto bad = correct;
    set(bad, slot, value);
    UsbipDecoder decoder(device);
    decoder.feed(bad);
    reject([&] { static_cast<void>(decoder.next()); });
    require(decoder.faulted(), "A malformed frame poisons its connection");
  }
  auto bad_unlink = unlink(9, 7);
  set(bad_unlink, Slot::length, 1);
  UsbipDecoder decoder(device);
  decoder.feed(bad_unlink);
  reject([&] { static_cast<void>(decoder.next()); });
  UsbipDecoder oversized(device);
  const std::vector<std::uint8_t> huge(maximum_usb_transfer + usbip_transfer_header + 1, 0);
  reject([&] { oversized.feed(huge); });
  require(oversized.faulted(), "Frame-buffer allocation is bounded before input append");
  reject([] { const UsbipDecoder invalid(1); });
}

void test_independent_replies() {
  UsbTransferCompletion completion{.sequence = 7,
                                   .status = UsbTransferStatus::success,
                                   .transferred = 3,
                                   .input =
                                       std::vector<std::uint8_t>(payload.begin(), payload.end()),
                                   .input_direction = true};
  auto expected = words({3, 7, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0});
  expected.insert(expected.end(), payload.begin(), payload.end());
  require(encode_usbip_completion(completion) == expected,
          "RET_SUBMIT zeros server-only fields and appends exact IN bytes");
  completion.status = UsbTransferStatus::short_packet;
  constexpr unsigned remote_io = 0xffffff87;
  set(expected, Slot::flags, remote_io);
  require(encode_usbip_completion(completion) == expected,
          "SHORT_NOT_OK encodes Linux -EREMOTEIO with the accepted prefix");
  completion.status = UsbTransferStatus::stalled;
  completion.transferred = 0;
  completion.input.clear();
  constexpr unsigned pipe_error = 0xffffffe0;
  require(encode_usbip_completion(completion) ==
              words({3, 7, 0, 0, 0, pipe_error, 0, 0, 0, 0, 0, 0}),
          "STALL encodes Linux -EPIPE");
  constexpr unsigned cancelled = 0xffffff98;
  require(
      encode_usbip_unlink(8, true) == words({4, 8, 0, 0, 0, cancelled, 0, 0, 0, 0, 0, 0}) &&
          encode_usbip_unlink(9, false) == words({4, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}),
      "UNLINK distinguishes active cancellation (-ECONNRESET) from an already completed target");
  completion.transferred = 1;
  reject([&] { static_cast<void>(encode_usbip_completion(completion)); });
}
}

int main() {
  try {
    test_fragmentation_and_eof();
    test_coalescing_setup_and_unlink();
    test_invalid_fields_and_bounds();
    test_independent_replies();
    std::cout << "Verified independent USB/IP wire framing, every fixed fragmentation/EOF "
                 "boundary, bounds and exact replies; no import/host-enumeration claim.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
