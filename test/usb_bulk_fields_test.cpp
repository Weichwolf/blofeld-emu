// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/mcu_usb.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
constexpr unsigned selector = 0x30e;
constexpr unsigned data = 0x310;
constexpr unsigned buffers = 0x30f;
constexpr unsigned local = 0x380;
constexpr unsigned in_control = 0x318;
constexpr unsigned out_control = 0x31a;
constexpr unsigned configuration_status = 0x30c;
constexpr unsigned enabled = 0xc000;
constexpr std::uint8_t acknowledge_transfer = 0x30; // TFRC W1C and TCIE, DVALID clear.
constexpr unsigned word_mask = 0xffff;
constexpr unsigned byte_bits = 8;
constexpr std::array<std::uint8_t, 4> first{9, 0x90, 0x45, 100};
constexpr std::array<std::uint8_t, 4> second{8, 0x80, 0x45, 0};

void require(const bool condition, const std::string_view reason) {
  if (!condition) { throw Fault(std::string(reason)); }
}

struct Description {
  unsigned bank;
  std::uint32_t value;
};

void describe(McuUsb &usb, const Description description) {
  usb.write({.address = selector, .value = static_cast<std::uint8_t>(description.bank)});
  usb.write_memory({.address = data, .size = 2, .value = description.value >> 16U});
  usb.write_memory({.address = data + 2, .size = 2, .value = description.value & word_mask});
}

unsigned read_word(const McuUsb &usb, const unsigned address) {
  return (static_cast<unsigned>(usb.read(static_cast<std::uint16_t>(address))) << byte_bits) |
         usb.read(static_cast<std::uint16_t>(address + 1));
}

struct Profile {
  unsigned nisof;
  unsigned maximum;
  unsigned type = 2;
};

McuUsb prepare(const Profile profile) {
  McuUsb usb;
  constexpr unsigned default_description = 0x02000000;
  constexpr unsigned original_interface_counts = 0x11;
  constexpr unsigned input_fields = 0x891;
  constexpr unsigned output_fields = 0x881;
  const auto size = profile.maximum << 19U;
  const auto field = profile.nisof << 30U;
  const auto type = profile.type << 5U;
  describe(usb, {.bank = 1, .value = default_description});
  describe(usb, {.bank = 2, .value = field | size | type | input_fields});
  describe(usb, {.bank = 3, .value = field | size | type | output_fields});
  describe(usb, {.bank = 8, .value = original_interface_counts});
  usb.observe_decoder({});
  usb.set_control(enabled);
  usb.observe_decoder({.enumerated = true});
  usb.observe_configuration({.configuration = 1});
  usb.write({.address = configuration_status, .value = 0x10});
  return usb;
}

void check_payload(McuUsb &usb, const std::array<std::uint8_t, 4> &expected) {
  for (unsigned i = 0; i < expected.size(); ++i) {
    require(usb.read(static_cast<std::uint16_t>(local + i)) == expected[i],
            "Every received byte must be the actual accepted packet byte");
  }
}

void output_case(McuUsb &usb) {
  usb.write({.address = buffers, .value = 3});
  const UsbOutPacket packet{.buffer = UsbBuffer::endpoint3, .data = first};
  require(usb.receive_out(packet) == UsbResponse::ack, "DATA0 must deliver the first bulk packet");
  check_payload(usb, first);
  const auto before = read_word(usb, out_control);
  require(usb.receive_out(packet) == UsbResponse::ack && read_word(usb, out_control) == before,
          "A duplicate DATA0 is acknowledged without a second delivery or state advance");
  require(usb.receive_out({.buffer = UsbBuffer::endpoint3, .data = second, .data1 = true}) ==
              UsbResponse::nak,
          "Occupied endpoint must NAK new DATA1 rather than overwrite its actual payload");
  check_payload(usb, first);
  usb.write({.address = out_control, .value = acknowledge_transfer});
  require(usb.receive_out({.buffer = UsbBuffer::endpoint3, .data = second, .data1 = true}) ==
              UsbResponse::ack,
          "Actual firmware buffer release permits exactly the next DATA1 packet");
  check_payload(usb, second);
}

void input_case(McuUsb &usb) {
  usb.write({.address = buffers, .value = 2});
  for (unsigned i = 0; i < first.size(); ++i) {
    usb.write({.address = static_cast<std::uint16_t>(local + i), .value = first[i]});
  }
  constexpr unsigned actual_ready_descriptor = 0x5004;
  usb.write({.address = in_control,
             .value = static_cast<std::uint8_t>(actual_ready_descriptor >> byte_bits)});
  constexpr unsigned byte_mask = 255;
  usb.write({.address = in_control + 1,
             .value = static_cast<std::uint8_t>(actual_ready_descriptor & byte_mask)});
  const auto first_packet = usb.request_in(UsbBuffer::endpoint2);
  require(first_packet.response == UsbResponse::data && first_packet.size == first.size() &&
              !first_packet.data1 &&
              std::ranges::equal(std::span(first_packet.data).first(first.size()), first),
          "Bulk IN uses actual programmed bytes and independent initial DATA0");
  usb.finish_in(false);
  const auto retry = usb.request_in(UsbBuffer::endpoint2);
  require(retry.response == UsbResponse::data && retry.size == first_packet.size &&
              retry.data1 == first_packet.data1 && retry.data == first_packet.data,
          "Lost ACK retains every transmitted byte and the same PID");
  usb.finish_in(true);
  constexpr unsigned acknowledged = 0x3004;
  require(read_word(usb, in_control) == acknowledged,
          "ACK clears DVALID and sets TFRC without changing actual TXSIZ or TCIE");
  usb.write({.address = in_control, .value = acknowledge_transfer});
  for (unsigned i = 0; i < second.size(); ++i) {
    usb.write({.address = static_cast<std::uint16_t>(local + i), .value = second[i]});
  }
  usb.write({.address = in_control,
             .value = static_cast<std::uint8_t>(actual_ready_descriptor >> byte_bits)});
  usb.write({.address = in_control + 1,
             .value = static_cast<std::uint8_t>(actual_ready_descriptor & byte_mask)});
  const auto next = usb.request_in(UsbBuffer::endpoint2);
  require(next.response == UsbResponse::data && next.size == second.size() && next.data1 &&
              std::ranges::equal(std::span(next.data).first(second.size()), second),
          "Only the acknowledged DATA0 advances actual bulk IN to DATA1 and the new payload");
  usb.finish_in(true);
}

void reject_interrupt(const unsigned nisof) {
  auto usb = prepare({.nisof = nisof, .maximum = 64, .type = 3});
  bool rejected = false;
  try {
    static_cast<void>(usb.request_in(UsbBuffer::endpoint2));
  } catch (const Fault &failure) {
    rejected = std::string_view(failure.what()).contains("Reserved USB local endpoint NISOF");
  }
  require(rejected, "Bulk hypothesis must not admit nonzero NISOF on interrupt endpoints");
}

void check() {
  for (unsigned nisof = 0; nisof < 4; ++nisof) {
    for (const unsigned maximum : {8U, 16U, 32U, 64U}) {
      auto usb = prepare({.nisof = nisof, .maximum = maximum});
      output_case(usb);
      input_case(usb);
      usb.write({.address = selector, .value = 2});
      const auto high = read_word(usb, data);
      require((high >> 14U) == nisof, "All raw NISOF bits must survive actual transfers unchanged");
    }
  }
  for (unsigned nisof = 1; nisof < 4; ++nisof) { reject_interrupt(nisof); }
}
}

int main() {
  try {
    check();
    std::cout << "32 bulk direction/size/NISOF cases and three strict interrupt guards passed\n";
    return 0;
  } catch (const std::exception &failure) {
    std::cerr << failure.what() << '\n';
    return 1;
  }
}
