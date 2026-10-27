// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "common/bus.hpp"
#include "mcu/mcu_usb.hpp"

#include <array>
#include <cstdint>

namespace usb_halt_fixture {
using blofeld::Fault;
using blofeld::McuUsb;
using blofeld::UsbBuffer;
using blofeld::UsbResponse;
constexpr unsigned selector = 0x30e, config_data = 0x310, buffer_select = 0x30f;
constexpr unsigned ep2 = 0x318, ep3 = 0x31a, fifo = 0x380, config_status = 0x30c;
constexpr unsigned word_mask = 0xffff, stall = 0x8000, completed = 0x2000;
constexpr unsigned valid = 0x4000, irq_enable = 0x1000, module_enable = 0xc000;
constexpr std::uint8_t device_address = 42;
constexpr std::uint8_t input_address = 0x81;
constexpr std::uint8_t output_address = 2;
constexpr std::array<std::uint8_t, 4> payload{0x31, 0x52, 0x73, 0x14};

inline void require(const bool condition, const char *message) {
  if (!condition) { throw Fault(message); }
}

struct Description {
  unsigned bank = 0;
  std::uint32_t value = 0;
};

inline void describe(McuUsb &usb, const Description description) {
  usb.write({.address = selector, .value = static_cast<std::uint8_t>(description.bank)});
  usb.write_memory({.address = config_data, .size = 2, .value = description.value >> 16U});
  usb.write_memory({.address = config_data + 2, .size = 2, .value = description.value & word_mask});
}

inline unsigned word(const McuUsb &usb, const std::uint16_t address) {
  return (static_cast<unsigned>(usb.read(address)) << 8U) |
         usb.read(static_cast<std::uint16_t>(address + 1));
}

struct Profile {
  bool interrupt = false;
  bool separate_interfaces = false;
};

inline McuUsb configured(const Profile profile = {}) {
  McuUsb usb;
  constexpr unsigned control = 0x02000000;
  constexpr unsigned input = 0x020000d1;
  constexpr unsigned input_interrupt = 0x020000f1;
  constexpr unsigned output = 0x020000c2;
  constexpr unsigned output_interrupt = 0x020000e2;
  constexpr unsigned counts = 0x11;
  constexpr unsigned alternates = 0x1111;
  constexpr unsigned two_interfaces = 0x21;
  constexpr unsigned second_interface = 0x800;
  for (const auto description :
       {Description{.bank = 1, .value = control},
        {.bank = 2, .value = profile.interrupt ? input_interrupt : input},
        {.bank = 3,
         .value = (profile.interrupt ? output_interrupt : output) |
                  (profile.separate_interfaces ? second_interface : 0U)},
        {.bank = 8, .value = profile.separate_interfaces ? two_interfaces : counts},
        {.bank = 9, .value = alternates}}) {
    describe(usb, description);
  }
  usb.observe_decoder({});
  usb.set_control(module_enable);
  usb.observe_decoder({.enumerated = true});
  constexpr std::array<std::uint8_t, 8> address{0, 5, device_address, 0, 0, 0, 0, 0};
  require(usb.receive_setup(address) == UsbResponse::ack, "Actual address SETUP");
  const auto packet = usb.request_in(UsbBuffer::control_in);
  require(packet.response == UsbResponse::data && packet.size == 0 && packet.data1,
          "Address status is DATA1/ZLP");
  usb.finish_in(true);
  constexpr std::array<std::uint8_t, 8> selection{0, 9, 1, 0, 0, 0, 0, 0};
  require(usb.receive_setup(selection) == UsbResponse::ack, "Actual configuration SETUP");
  usb.write({.address = config_status, .value = 0x10});
  const auto status = usb.request_in(UsbBuffer::control_in);
  require(status.response == UsbResponse::data && status.size == 0 && status.data1,
          "Actual configuration DATA1/ZLP status");
  usb.finish_in(true);
  return usb;
}

struct Case {
  bool input = false;
  bool occupied = false;
};

inline UsbResponse transaction(McuUsb &usb, const Case test) {
  if (test.input) { return usb.request_in(UsbBuffer::endpoint2).response; }
  return usb.receive_out({.buffer = UsbBuffer::endpoint3, .data = payload, .data1 = test.occupied});
}

inline McuUsb armed_case(const Case test, const Profile profile = {}) {
  auto usb = configured(profile);
  if (test.input && test.occupied) {
    usb.write({.address = buffer_select, .value = 2});
    for (unsigned index = 0; index < payload.size(); ++index) {
      usb.write({.address = static_cast<std::uint16_t>(fifo + index), .value = payload[index]});
    }
    usb.write({.address = ep2 + 1, .value = payload.size()});
  } else if (test.occupied) {
    require(usb.receive_out({.buffer = UsbBuffer::endpoint3, .data = payload}) == UsbResponse::ack,
            "An actual preceding DATA0 packet occupies the OUT buffer");
  }
  const auto address = static_cast<std::uint16_t>(test.input ? ep2 : ep3);
  const auto control = stall | completed | irq_enable | (test.occupied ? valid : 0U);
  usb.write({.address = address, .value = static_cast<std::uint8_t>(control >> 8U)});
  return usb;
}
}
