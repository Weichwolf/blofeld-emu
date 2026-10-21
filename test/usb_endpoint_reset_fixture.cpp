// SPDX-License-Identifier: GPL-3.0-only
#include "usb_endpoint_reset_fixture.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_usb.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace blofeld::test::endpoint_reset {
void require(const bool condition, const std::string_view message) {
  if (!condition) { throw Fault(std::string(message)); }
}

unsigned word(McuBus &bus, const unsigned address) {
  return static_cast<unsigned>(
      bus.read_memory({.address = static_cast<std::uint16_t>(address), .size = 2}));
}

void put(McuBus &bus, const MemoryWrite write) {
  bus.write_memory(write);
}

void configure(McuBus &bus, const Description description) {
  bus.write(selector, static_cast<std::uint8_t>(description.bank));
  put(bus, {.address = config_data, .size = 2, .value = description.value >> 16U});
  put(bus, {.address = config_data + 2, .size = 2, .value = description.value & word_mask});
}

void enable(McuBus &bus) {
  bus.write(control, men_high);
}

void activate_bulk(McuBus &bus) {
  bus.observe_usb_configuration({.configuration = 1});
  bus.write(configuration_status, 0x10);
}

void prepare(McuBus &bus) {
  bus.set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
  bus.write(control, mce_high);
  bus.write(control + 1, 8);
  bus.observe_phy({.edges = phy_edges});
  for (const auto description : {Description{.bank = 1, .value = default_endpoint},
                                 Description{.bank = 2, .value = bulk_in},
                                 Description{.bank = 3, .value = bulk_out},
                                 Description{.bank = 8, .value = 0x11}}) {
    configure(bus, description);
  }
  bus.observe_usb_decoder({});
  enable(bus);
  bus.observe_usb_decoder({.reset = true});
  bus.observe_usb_decoder({.enumerated = true});
  bus.observe_usb_decoder({.suspended = true, .enumerated = true});
  bus.observe_usb_decoder({.enumerated = true});
  activate_bulk(bus);
  require(bus.receive_usb_setup(request) == UsbResponse::ack &&
              bus.receive_usb_setup(request) == UsbResponse::ack,
          "Two own SETUPs establish DATA1, SETUP and SETOVR before reset");
}

void populate(McuBus &bus) {
  for (unsigned buffer = 0; buffer < 4; ++buffer) {
    bus.write(buffer_selector, static_cast<std::uint8_t>(buffer));
    for (unsigned i = 0; i < 64; ++i) {
      bus.write(static_cast<std::uint16_t>(local + i),
                static_cast<std::uint8_t>((buffer * 64) + i));
    }
  }
  require(bus.receive_usb_out({.buffer = UsbBuffer::control_out, .data = payload, .data1 = true}) ==
                  UsbResponse::ack &&
              bus.receive_usb_out({.buffer = UsbBuffer::endpoint3, .data = payload}) ==
                  UsbResponse::ack,
          "Real OUT packets populate both OUT controls and completion flags");
  constexpr unsigned stalled_out = 0xd0;
  bus.write(endpoint + 2, stalled_out);
  bus.write(endpoint + 6, stalled_out);
  put(bus, {.address = endpoint, .size = 2, .value = software_ready});
  put(bus, {.address = endpoint + 4, .size = 2, .value = software_ready});
  constexpr unsigned out_ique = 0x31c;
  constexpr unsigned in_ique = 0x320;
  constexpr unsigned ique_fields = 0xd357;
  put(bus, {.address = out_ique, .size = 2, .value = ique_fields});
  put(bus, {.address = in_ique, .size = 2, .value = ique_fields});
  put(bus, {.address = mask, .size = 2, .value = pending_mask});
  bus.observe_usb_sof(last_frame);
  constexpr unsigned before_status = 0xfc47;
  require(word(bus, status) == before_status,
          "All four reset flags and preserved SOF are genuinely pending");
  const auto first = bus.request_usb_in(UsbBuffer::control_in);
  const auto second = bus.request_usb_in(UsbBuffer::endpoint2);
  require(first.response == UsbResponse::data && first.data1 &&
              second.response == UsbResponse::data && !second.data1,
          "Both IN endpoints have real immutable unacknowledged transmissions before reset");
  bus.write(selector, 1);
  constexpr unsigned high_packet_size = 0x0200;
  put(bus, {.address = config_data, .size = 2, .value = high_packet_size});
  bus.write(buffer_selector, selected_buffers);
  constexpr unsigned rti_control = 0x3b;
  constexpr unsigned oscillator_prefix = 123;
  bus.write(rti_control, 0x40);
  bus.advance_oscillator(oscillator_prefix);
}

void reset_command(McuBus &bus) {
  bus.write(control, mce_high);
  bus.write(control, eprst_high);
}

void complete_staging(McuBus &bus) {
  put(bus, {.address = config_data + 2, .size = 2, .value = 0});
}

std::vector<unsigned> inspect(McuBus &bus) {
  auto copy = bus.clone();
  std::vector<unsigned> result;
  constexpr std::array<unsigned, 6> registers{
      control, status, mask, timestamp, configuration_status, selector};
  constexpr unsigned observation_count = 6 + 3 + 6 + 1 + 14 + 256;
  result.reserve(observation_count);
  for (const auto at : registers) { result.push_back(word(*copy, at)); }
  for (const auto at : {0x35U, 0x37U, 0x39U}) {
    result.push_back(copy->read(static_cast<std::uint16_t>(at)));
  }
  for (const auto at : controls) { result.push_back(word(*copy, at)); }
  result.push_back(copy->oscillator_cycles_until_rti());
  complete_staging(*copy);
  for (const auto bank : {1U, 2U, 3U, 4U, 5U, 8U, 9U}) {
    copy->write(selector, static_cast<std::uint8_t>(bank));
    result.push_back(word(*copy, config_data));
    result.push_back(word(*copy, config_data + 2));
  }
  for (unsigned buffer = 0; buffer < 4; ++buffer) {
    copy->write(buffer_selector, static_cast<std::uint8_t>(buffer));
    for (unsigned i = 0; i < 64; ++i) {
      result.push_back(copy->read(static_cast<std::uint16_t>(local + i)));
    }
  }
  return result;
}

void verify_reset(McuBus &bus) {
  constexpr unsigned quiet_control = 0x4008;
  constexpr unsigned before_rti = 8069;
  require(
      word(bus, control) == quiet_control && word(bus, status) == reset_status &&
          word(bus, mask) == pending_mask && word(bus, timestamp) == last_frame &&
          word(bus, configuration_status) == ready_configuration &&
          word(bus, selector) == selected_registers &&
          bus.oscillator_cycles_until_rti() == before_rti,
      "EPRST clears only its endpoint/UMSR flags, preserving SOF/configuration/select/CRG phase");
  for (const auto at : controls) {
    require(word(bus, at) == 0, "Every modeled local/IQUE A control and size resets to zero");
  }
  auto copy = bus.clone();
  enable(*copy);
  constexpr unsigned unmasked_ccr = 0x40;
  // 9S12UF32DGV1 table 5-1: SOF has its own vector, separate from status change.
  constexpr unsigned sof_vector = 0xffbe;
  const auto pending = copy->interrupt_request(unmasked_ccr);
  require(pending && pending->vector == sof_vector,
          "Preserved SOF still raises the real USB SOF interrupt after EPRST");
  copy->write(status + 1, 4);
  require(!copy->interrupt_request(unmasked_ccr),
          "After acknowledging SOF, reset flags and canceled endpoints raise no interrupt");
  complete_staging(*copy);
  for (const auto description : {Description{.bank = 1, .value = default_endpoint},
                                 Description{.bank = 2, .value = bulk_in},
                                 Description{.bank = 3, .value = bulk_out},
                                 Description{.bank = 8, .value = 0x11}}) {
    copy->write(selector, static_cast<std::uint8_t>(description.bank));
    const auto value = (word(*copy, config_data) << 16U) | word(*copy, config_data + 2);
    require(value == description.value,
            "Static endpoint descriptions and interface limits survive EPRST");
  }
  for (unsigned buffer = 0; buffer < 4; ++buffer) {
    copy->write(buffer_selector, static_cast<std::uint8_t>(buffer));
    for (unsigned i = 0; i < 64; ++i) {
      const auto expected =
          (buffer % 2 == 1 && i < payload.size()) ? payload[i] : (buffer * 64) + i;
      require(copy->read(static_cast<std::uint16_t>(local + i)) == expected,
              "EPRST does not zero or forget known local-buffer bytes");
    }
  }
}

McuFlashImage own_image() {
  std::array<std::uint8_t, McuFlashImage::byte_count> flash{};
  constexpr unsigned nop = 0xa7;
  flash.fill(nop);
  // Own HC12 LDD #$4208 / STD $0300, not manufacturer program bytes.
  constexpr std::array<std::uint8_t, 6> code{0xcc, 0x42, 8, 0x7c, 3, 0};
  std::ranges::copy(code, flash.begin());
  return McuFlashImage::from_known(flash);
}
}
