// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/mcu_clock.hpp"
#include "blofeld/mcu_usb.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <string>

namespace {
// Independent references: S12USB20D6E2FV1 register/interrupt tables,
// 9S12UF32DGV1 table 5-1, USB 2.0 sections 8.6.2..8.6.4. All payloads are own data.
constexpr unsigned status = 0x302;
constexpr unsigned mask = 0x304;
constexpr unsigned timestamp = 0x30a;
constexpr unsigned configuration_status = 0x30c;
constexpr unsigned selector = 0x30e;
constexpr unsigned buffers = 0x30f;
constexpr unsigned data = 0x310;
constexpr unsigned endpoint2 = 0x318;
constexpr unsigned endpoint3 = 0x31a;
constexpr unsigned setup_buffer = 0x338;
constexpr unsigned local_buffer = 0x380;
constexpr unsigned second_buffer = 0x3c0;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned word_mask = 0xffff;
constexpr unsigned enabled_control = 0xc000;
constexpr unsigned setup_mask = 3;
constexpr unsigned setup_flag = 1;
constexpr unsigned overrun_flag = 2;
constexpr unsigned sof_flag = 4;
constexpr unsigned endpoint_ie = 0x10;
constexpr unsigned endpoint_dv_ie = 0x50;
constexpr unsigned endpoint_ack = 0x30;
constexpr unsigned endpoint_rx_ack = 0x70;
constexpr std::array<std::uint8_t, 8> descriptor_request{0x80, 6, 0, 1, 0, 0, 18, 0};
constexpr std::array<std::uint8_t, 8> class_request{0x21, 9, 0, 0, 0, 0, 2, 0};

void require(const bool condition, const std::string &message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void reject(const auto &operation, const std::string &message) {
  try {
    operation();
  } catch (const blofeld::Fault &) { return; }
  throw blofeld::Fault(message);
}

unsigned word(const blofeld::McuUsb &usb, const unsigned address) {
  const auto high = usb.read(static_cast<std::uint16_t>(address));
  return (static_cast<unsigned>(high) << 8U) | usb.read(static_cast<std::uint16_t>(address + 1));
}

void write_word(blofeld::McuUsb &usb, const unsigned address, const unsigned value) {
  usb.write({.address = static_cast<std::uint16_t>(address),
             .value = static_cast<std::uint8_t>(value >> 8U)});
  usb.write({.address = static_cast<std::uint16_t>(address + 1),
             .value = static_cast<std::uint8_t>(value & byte_mask)});
}

struct Configuration {
  unsigned select = 0;
  std::uint32_t value = 0;
};

void configure(blofeld::McuUsb &usb, const Configuration config) {
  usb.write({.address = selector, .value = static_cast<std::uint8_t>(config.select)});
  usb.write_memory({.address = data, .size = 2, .value = config.value >> 16U});
  usb.write_memory({.address = data + 2, .size = 2, .value = config.value & word_mask});
}

void start(blofeld::McuUsb &usb) {
  constexpr unsigned default_endpoint = 0x02000000;
  constexpr unsigned input_endpoint = 0x020000d1;
  constexpr unsigned output_endpoint = 0x020000c2;
  constexpr unsigned configurations = 0x11;
  constexpr unsigned alternate_settings = 0x1111;
  configure(usb, {.select = 1, .value = default_endpoint});
  configure(usb, {.select = 2, .value = input_endpoint});
  configure(usb, {.select = 3, .value = output_endpoint});
  configure(usb, {.select = 8, .value = configurations});
  configure(usb, {.select = 9, .value = alternate_settings});
  usb.observe_decoder({});
  usb.set_control(enabled_control);
  usb.observe_decoder({.enumerated = true});
}

void activate_bulk(blofeld::McuUsb &usb) {
  usb.observe_configuration({.configuration = 1, .interface = std::nullopt});
  usb.write({.address = configuration_status, .value = endpoint_ie});
}

void test_reset_flags_status_and_resume_level() {
  blofeld::McuUsb usb;
  constexpr unsigned reset_status = 0xe040;
  constexpr unsigned mask_bits = 0x7807;
  require(word(usb, status) == reset_status && word(usb, mask) == 0,
          "UMSR1 resets to literal E040, with no active transaction and full-speed default");
  write_word(usb, status, word_mask);
  require(word(usb, status) == reset_status, "Read-only status fields survive W1C writes");
  write_word(usb, mask, word_mask);
  require(word(usb, mask) == mask_bits && usb.interrupt_sources() == 1,
          "UIMR masks reserved bits; RESUMEIE is a USD=0 level source, not an invented edge latch");
  write_word(usb, mask, 0);
  start(usb);
  constexpr unsigned enumerated_status = 0xe440;
  require(word(usb, status) == enumerated_status,
          "Explicit full-speed enumeration observation updates only live fields");
  constexpr unsigned status_change_enables = 0x1800;
  write_word(usb, mask, status_change_enables);
  usb.observe_decoder({.suspended = true, .enumerated = true});
  constexpr unsigned suspended_status = 0xf450;
  require(word(usb, status) == suspended_status && usb.interrupt_sources() == 1,
          "Suspend edge latches USSC and sets USD");
  usb.write({.address = status, .value = endpoint_ie});
  require(usb.interrupt_sources() == 0, "USSC W1C acknowledges the status-change interrupt");
  constexpr unsigned resume_enable = 0x4000;
  write_word(usb, mask, resume_enable);
  require(usb.interrupt_sources() == 0, "USD=1 blocks the resume level");
  usb.observe_decoder({.enumerated = true});
  usb.write({.address = status, .value = endpoint_ie});
  require(usb.interrupt_sources() == 1,
          "Clearing USSC cannot acknowledge the separate USD=0 level");
  write_word(usb, mask, status_change_enables);
  usb.observe_decoder({.reset = true});
  constexpr unsigned reset_detected = 0xe848;
  require(word(usb, status) == reset_detected && usb.interrupt_sources() == 1,
          "Reset assertion sets URD/URSC and ends the observed enumeration state");
  usb.observe_decoder({});
  usb.write({.address = status, .value = 8});
  require(word(usb, status) == reset_status && usb.interrupt_sources() == 0,
          "Reset deassertion latches a change; W1C clears only its sticky flag");
  usb.reset();
  require(word(usb, status) == reset_status && word(usb, mask) == 0,
          "External reset restores control/mask/status defaults");
  usb.set_control(enabled_control);
  reject([&usb] { static_cast<void>(word(usb, status)); },
         "MEN cannot fabricate unknown decoder inputs");
}

void test_sof_and_configuration_sources() {
  blofeld::McuUsb usb;
  start(usb);
  write_word(usb, mask, sof_flag);
  constexpr unsigned first_frame = 0x345;
  usb.observe_sof(first_frame, false);
  require(word(usb, timestamp) == 0 && usb.interrupt_sources() == 0,
          "Bad-CRC SOF does not update time or flags");
  usb.observe_sof(first_frame);
  require(word(usb, timestamp) == first_frame && usb.interrupt_sources() == 8,
          "Valid SOF updates the literal eleven-bit frame number and independent SOF source");
  usb.write({.address = status + 1, .value = sof_flag});
  write_word(usb, timestamp, word_mask);
  require(usb.interrupt_sources() == 0 && word(usb, timestamp) == first_frame,
          "SOF W1C clears its flag; timestamp writes cannot synthesize host frames");
  constexpr unsigned last_frame = 2047;
  usb.observe_sof(last_frame);
  usb.observe_sof(0);
  require(word(usb, timestamp) == 0, "Frame-number wrap follows explicit host observations");
  reject([&usb] { usb.observe_sof(last_frame + 1); },
         "Out-of-range frame numbers fail without truncation");
  constexpr unsigned configuration_enable = 0x2000;
  write_word(usb, mask, configuration_enable);
  usb.observe_configuration({.configuration = 1, .interface = std::nullopt});
  constexpr unsigned pending = 0xa100;
  require(word(usb, configuration_status) == pending && usb.interrupt_sources() == 4,
          "Decoded configuration request sets CFGVALID/CFG/SETECR and its own interrupt source");
  usb.write({.address = configuration_status, .value = 0});
  require(usb.interrupt_sources() == 4, "Writing DONEECRU=0 cannot acknowledge SETECR");
  usb.write({.address = configuration_status, .value = endpoint_ie});
  constexpr unsigned done = 0x8100;
  require(word(usb, configuration_status) == done && usb.interrupt_sources() == 0,
          "DONEECRU acknowledges SETECR and self-clears without changing requested fields");
  usb.observe_configuration({.configuration = 1, .interface = 0, .alternate = 0});
  constexpr unsigned interface_request = 0xe100;
  require(word(usb, configuration_status) == interface_request && usb.interrupt_sources() == 4,
          "Interface/alternate request validity is distinct from configuration validity");
  reject([&usb] { usb.observe_configuration({.configuration = 1, .interface = std::nullopt}); },
         "Overlapping unresolved configuration requests fail");
  usb.write({.address = configuration_status, .value = endpoint_ie});
  reject([&usb] { usb.observe_configuration({.configuration = 2, .interface = std::nullopt}); },
         "The configured limit bounds decoded configuration input");
  reject(
      [&usb] { usb.observe_configuration({.configuration = 1, .interface = 0, .alternate = 1}); },
      "Alternate settings are bounded by the separately programmed UNASR fields");
}

void test_setup_buffers_overrun_and_explicit_limits() {
  blofeld::McuUsb usb;
  start(usb);
  write_word(usb, mask, setup_mask);
  reject([&usb] { static_cast<void>(usb.read(setup_buffer)); },
         "USTB is not valid before an application SETUP");
  require(usb.receive_setup(descriptor_request) == blofeld::UsbResponse::ack &&
              usb.interrupt_sources() == 2,
          "GET_DESCRIPTOR reaches the application setup buffer and setup interrupt source");
  for (unsigned index = 0; index < descriptor_request.size(); ++index) {
    require(usb.read(static_cast<std::uint16_t>(setup_buffer + index)) == descriptor_request[index],
            "Setup buffer retains the eight literal request bytes in wire order");
  }
  require(usb.receive_setup(class_request, false) == blofeld::UsbResponse::none &&
              (usb.read(status + 1) & setup_mask) == setup_flag,
          "A bad-CRC setup does not overwrite the old request or create overrun");
  require(usb.receive_setup(class_request) == blofeld::UsbResponse::ack &&
              (usb.read(status + 1) & setup_mask) == setup_mask &&
              usb.read(setup_buffer) == class_request[0],
          "A second application SETUP overwrites the request and latches SETOVR");
  usb.write({.address = status + 1, .value = setup_flag});
  require(usb.interrupt_sources() == 2, "SETOVR remains pending after SETUP is acknowledged");
  reject([&usb] { static_cast<void>(usb.read(setup_buffer)); },
         "Cleared SETUP invalidates buffer reads despite retained bytes");
  usb.write({.address = status + 1, .value = overrun_flag});
  require(usb.interrupt_sources() == 0, "SETOVR W1C clears its independent sticky event");
  constexpr std::array<std::uint8_t, 8> automatic_address{0, 5, 7, 0, 0, 0, 0, 0};
  reject([&usb, &automatic_address] { static_cast<void>(usb.receive_setup(automatic_address)); },
         "Automatic standard requests remain explicit failures until their protocol engine exists");
  constexpr std::array<std::uint8_t, 8> optional_descriptor{0, 7, 0, 1, 0, 0, 2, 0};
  require(usb.receive_setup(optional_descriptor) == blofeld::UsbResponse::ack &&
              usb.interrupt_sources() == 0,
          "SDSUP=0 accepts the SETUP stage but does not pass SET_DESCRIPTOR to firmware");
  require(usb.request_in(blofeld::UsbBuffer::control_in).response == blofeld::UsbResponse::stall,
          "Unsupported optional standard request stalls the data/status phase");
  usb.receive_setup(descriptor_request);
  require(usb.request_in(blofeld::UsbBuffer::control_in).response == blofeld::UsbResponse::nak,
          "A new SETUP ends the old stall and NAKs until firmware supplies real data");
}

void test_buffer_aliases_and_out_retries() {
  blofeld::McuUsb usb;
  start(usb);
  activate_bulk(usb);
  reject([&usb] { static_cast<void>(usb.read(local_buffer)); },
         "Cold USB buffer bytes are unknown");
  constexpr unsigned first_byte = 0xa5;
  usb.write({.address = local_buffer, .value = first_byte});
  require(usb.read(second_buffer) == first_byte,
          "Both reset windows alias the same default IN buffer");
  constexpr unsigned invalid_windows = 0x44;
  usb.write({.address = buffers, .value = invalid_windows});
  reject([&usb] { static_cast<void>(usb.read(local_buffer)); },
         "Reserved buffer selections cannot alias real RAM");
  usb.write({.address = buffers, .value = 3});
  usb.write({.address = endpoint3, .value = endpoint_ie});
  constexpr std::array<std::uint8_t, 4> midi{9, 0x90, 60, 127};
  const blofeld::UsbOutPacket packet{.buffer = blofeld::UsbBuffer::endpoint3, .data = midi};
  require(usb.receive_out(packet) == blofeld::UsbResponse::ack,
          "DATA0 delivers the first bulk OUT packet");
  constexpr unsigned received = 0x7004;
  constexpr unsigned endpoint3_source = 0x80;
  require(word(usb, endpoint3) == received && usb.interrupt_sources() == endpoint3_source,
          "Accepted OUT sets DVALID/TFRC, preserves TCIE and reports its real byte length");
  for (unsigned byte = 0; byte < midi.size(); ++byte) {
    require(usb.read(static_cast<std::uint16_t>(local_buffer + byte)) == midi[byte],
            "OUT data reaches the selected buffer");
  }
  usb.write({.address = endpoint3, .value = endpoint_rx_ack});
  require(usb.interrupt_sources() == 0,
          "TFRC W1C acknowledges completion without releasing the occupied buffer");
  constexpr std::array<std::uint8_t, 2> changed{0x55, 0xaa};
  require(usb.receive_out({.buffer = blofeld::UsbBuffer::endpoint3, .data = changed}) ==
                  blofeld::UsbResponse::ack &&
              usb.read(local_buffer) == midi[0] && usb.interrupt_sources() == 0,
          "A duplicate DATA0 after a lost ACK is re-ACKed without redelivery or a new completion");
  require(
      usb.receive_out({.buffer = blofeld::UsbBuffer::endpoint3, .data = changed, .data1 = true}) ==
          blofeld::UsbResponse::nak,
      "A new DATA1 packet waits while firmware owns the OUT buffer");
  usb.write({.address = endpoint3, .value = endpoint_ie});
  require(
      usb.receive_out({.buffer = blofeld::UsbBuffer::endpoint3, .data = changed, .data1 = true}) ==
              blofeld::UsbResponse::ack &&
          usb.read(local_buffer) == changed[0],
      "Clearing DVALID releases the buffer for the next toggle");
  require(usb.receive_out(
              {.buffer = blofeld::UsbBuffer::endpoint3, .data = midi, .crc_valid = false}) ==
              blofeld::UsbResponse::none,
          "Bad-CRC OUT produces no handshake and no buffer update");
  const std::array<std::uint8_t, 65> oversized{};
  reject(
      [&usb, &oversized] {
        static_cast<void>(
            usb.receive_out({.buffer = blofeld::UsbBuffer::endpoint3, .data = oversized}));
      },
      "Oversized decoded packets fail before writing beyond local RAM");
}

void test_in_ack_loss_and_ownership() {
  blofeld::McuUsb usb;
  start(usb);
  activate_bulk(usb);
  usb.write({.address = buffers, .value = 2});
  constexpr std::array<std::uint8_t, 4> payload{1, 0x23, 0x45, 0x67};
  for (unsigned byte = 0; byte < payload.size(); ++byte) {
    usb.write({.address = static_cast<std::uint16_t>(local_buffer + byte), .value = payload[byte]});
  }
  usb.write({.address = endpoint2 + 1, .value = 4});
  usb.write({.address = endpoint2, .value = endpoint_dv_ie});
  const auto first = usb.request_in(blofeld::UsbBuffer::endpoint2);
  require(first.response == blofeld::UsbResponse::data && first.size == payload.size() &&
              !first.data1 &&
              std::ranges::equal(std::span{first.data}.first(payload.size()), payload),
          "Firmware-backed IN returns the literal payload and initial DATA0");
  require(usb.interrupt_sources() == 0, "Sending IN data is not completion before the host ACK");
  const auto retried = usb.request_in(blofeld::UsbBuffer::endpoint2);
  require(retried.data == first.data && retried.size == first.size && retried.data1 == first.data1,
          "IN retry preserves the outstanding payload and PID");
  reject([&usb] { usb.write({.address = local_buffer, .value = 0}); },
         "CPU writes cannot silently mutate an in-flight packet");
  reject([&usb] { static_cast<void>(usb.read(status)); },
         "Unverified ACTEP timing is not fabricated during IN");
  usb.finish_in(false);
  reject([&usb] { usb.write({.address = local_buffer, .value = 0}); },
         "Missing ACK preserves packet ownership even after the handshake window ends");
  const auto after_loss = usb.request_in(blofeld::UsbBuffer::endpoint2);
  require(!after_loss.data1 && after_loss.data == first.data,
          "A missing ACK does not consume or toggle the packet");
  usb.finish_in(true);
  constexpr unsigned completed = 0x3004;
  constexpr unsigned endpoint2_source = 0x40;
  require(word(usb, endpoint2) == completed && usb.interrupt_sources() == endpoint2_source,
          "Only host ACK clears DVALID, sets TFRC and exposes the endpoint interrupt");
  usb.write({.address = endpoint2, .value = endpoint_ack});
  require(usb.interrupt_sources() == 0 &&
              usb.request_in(blofeld::UsbBuffer::endpoint2).response == blofeld::UsbResponse::nak,
          "Acknowledged firmware completion leaves the IN endpoint empty and NAKing");
  usb.write({.address = endpoint2, .value = endpoint_dv_ie});
  require(usb.request_in(blofeld::UsbBuffer::endpoint2).data1,
          "The next accepted IN packet uses DATA1");
  usb.finish_in(true);
  reject([&usb] { usb.finish_in(true); },
         "An extra host ACK cannot acknowledge an unrelated transfer");
  usb.write({.address = endpoint2, .value = endpoint_dv_ie});
  require(!usb.request_in(blofeld::UsbBuffer::endpoint2).data1,
          "DATA0 follows a completed DATA1 packet");
  usb.finish_in(true);
  usb.observe_configuration({.configuration = 1, .interface = std::nullopt});
  usb.write({.address = configuration_status, .value = endpoint_ie});
  usb.write({.address = endpoint2, .value = endpoint_dv_ie});
  require(!usb.request_in(blofeld::UsbBuffer::endpoint2).data1,
          "A configuration event reinitializes bulk data toggles to DATA0");
  usb.finish_in(true);
}

void test_packet_boundaries_and_reset_during_in() {
  blofeld::McuUsb usb;
  start(usb);
  constexpr std::array<std::uint8_t, 8> request64{0x80, 6, 0, 1, 0, 0, 64, 0};
  usb.receive_setup(request64);
  constexpr unsigned endpoint0 = 0x314;
  usb.write({.address = endpoint0 + 1, .value = 64});
  usb.write({.address = endpoint0, .value = endpoint_dv_ie});
  reject([&usb] { static_cast<void>(usb.request_in(blofeld::UsbBuffer::control_in)); },
         "DVALID cannot turn uninitialized USB RAM into a valid outgoing packet");
  for (unsigned byte = 0; byte < 64; ++byte) {
    usb.write({.address = static_cast<std::uint16_t>(local_buffer + byte),
               .value = static_cast<std::uint8_t>(byte)});
  }
  const auto packet = usb.request_in(blofeld::UsbBuffer::control_in);
  require(packet.size == 64 && packet.data1,
          "A full local-buffer packet retains its size and control DATA1");
  for (unsigned byte = 0; byte < 64; ++byte) {
    require(packet.data[byte] == byte, "Every packet boundary byte is preserved");
  }
  usb.observe_sof(2);
  reject([&usb] { usb.finish_in(true); },
         "A later SOF cannot leave an obsolete IN handshake window open");
  const auto retry = usb.request_in(blofeld::UsbBuffer::control_in);
  require(retry.data == packet.data && retry.data1,
          "Token interleaving preserves the unacknowledged packet snapshot");
  usb.reset();
  reject([&usb] { usb.finish_in(true); }, "Reset discards outstanding handshake ownership");
  require(usb.interrupt_sources() == 0 && word(usb, endpoint0) == 0,
          "Reset does not fabricate a completion interrupt");
  reject([&usb] { static_cast<void>(usb.read(local_buffer)); },
         "Reset invalidates buffer knowledge without inventing cleared RAM");
}

void test_control_enable_gates() {
  blofeld::McuClock clock;
  constexpr unsigned control = 0x300;
  constexpr unsigned low_control = 0x301;
  constexpr unsigned clock_on = 0x40;
  constexpr unsigned men_clock = 0xc0;
  clock.write({.address = control, .value = clock_on});
  clock.write({.address = low_control, .value = 8});
  clock.write({.address = control, .value = men_clock});
  clock.write({.address = low_control, .value = byte_mask});
  constexpr unsigned descriptor_only_changed = 0x18;
  require(clock.read(low_control) == descriptor_only_changed,
          "MEN protects DISCON/EXSPD/RWUC/SPWR/SCSUP but not SDSUP");
  clock.write({.address = control, .value = 0});
  require(clock.read(control) == clock_on,
          "MCE cannot change in the same write that clears an already set MEN");
  clock.write({.address = control, .value = 0});
  require(clock.read(control) == 0, "After MEN clears, another write can disable the module clock");
}
}

int main() {
  try {
    test_reset_flags_status_and_resume_level();
    test_sof_and_configuration_sources();
    test_setup_buffers_overrun_and_explicit_limits();
    test_buffer_aliases_and_out_retries();
    test_in_ack_loss_and_ownership();
    test_packet_boundaries_and_reset_during_in();
    test_control_enable_gates();
    std::cout << "USB status, interrupts, setup, local buffers and retry checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
