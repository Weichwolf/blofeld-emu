// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_usb.hpp"
#include "usb/usb_host.hpp"
#include "usb/usb_transfer.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <vector>

namespace {
using namespace blofeld;
constexpr unsigned entry = 0x8000;
constexpr unsigned reset_vector = 0xfffe;
constexpr unsigned control = 0x300;
constexpr unsigned status_low = 0x303;
constexpr unsigned mask_low = 0x305;
constexpr unsigned selector = 0x30e;
constexpr unsigned config_data = 0x310;
constexpr unsigned config_status = 0x30c;
constexpr unsigned ep2 = 0x318;
constexpr unsigned ep3 = 0x31a;
constexpr unsigned word_mask = 0xffff;
constexpr unsigned negative = 8;
constexpr unsigned preserved_flags = 0xf1;
constexpr unsigned reset_masks = 0xd0;
constexpr unsigned changed_flags = 0x0e;
constexpr unsigned unmasked_ccr = 0x40;
constexpr unsigned configuration_done = 0x10;
constexpr unsigned phy_edges = 15;
constexpr unsigned device_address = 42;
constexpr std::uint8_t input_address = 0x81;
constexpr std::uint8_t output_address = 2;
constexpr std::uint8_t endpoint_status_input = 0x82;

void require(const bool condition, const char *message) {
  if (!condition) { throw Fault(message); }
}

std::array<std::uint8_t, 32768> image(const bool halted) {
  std::array<std::uint8_t, 32768> bytes{};
  constexpr std::uint8_t halt_irq_high = 0x90;
  constexpr std::uint8_t irq_high = 0x10;
  // Own LDS #$2000; LDD #$9000/$1000; STD $0318; STD $031A.
  // S12CPUV2 LDD/STD opcode and NZV semantics are the independent expectations.
  const std::array<std::uint8_t, 12> code{
      0xcf, 0x20, 0, 0xcc, halted ? halt_irq_high : irq_high, 0, 0x7c, 3, 0x18, 0x7c, 3, 0x1a};
  std::ranges::copy(code, bytes.begin());
  bytes.at(reset_vector - entry) = static_cast<std::uint8_t>(entry >> 8U);
  bytes.at(reset_vector - entry + 1) = 0;
  return bytes;
}

struct Description {
  unsigned bank;
  unsigned value;
};

void describe(McuBus &bus, const Description description) {
  bus.write(selector, static_cast<std::uint8_t>(description.bank));
  bus.write_memory({.address = config_data, .size = 2, .value = description.value >> 16U});
  bus.write_memory({.address = config_data + 2, .size = 2, .value = description.value & word_mask});
}

void status(McuBus &bus) {
  const auto packet = bus.request_usb_in(UsbBuffer::control_in);
  require(packet.response == UsbResponse::data && packet.size == 0 && packet.data1,
          "Actual address/configuration DATA1 status");
  bus.finish_usb_in(true);
}

void prepare(McuBus &bus) {
  constexpr unsigned module_clock = 0x40;
  constexpr unsigned module_enable = 0xc0;
  constexpr unsigned control_description = 0x02000000;
  constexpr unsigned input_description = 0x020000d1;
  constexpr unsigned output_description = 0x020000c2;
  constexpr unsigned counts = 0x11;
  constexpr unsigned alternates = 0x1111;
  bus.write(control, module_clock);
  bus.write(control + 1, 8);
  bus.observe_phy({.edges = phy_edges});
  describe(bus, {.bank = 1, .value = control_description});
  describe(bus, {.bank = 2, .value = input_description});
  describe(bus, {.bank = 3, .value = output_description});
  describe(bus, {.bank = 8, .value = counts});
  describe(bus, {.bank = 9, .value = alternates});
  bus.observe_usb_decoder({});
  bus.write(control, module_enable);
  bus.observe_usb_decoder({.reset = true});
  bus.observe_usb_decoder({.enumerated = true});
  constexpr std::array<std::uint8_t, 8> address{0, 5, device_address, 0, 0, 0, 0, 0};
  constexpr std::array<std::uint8_t, 8> configuration{0, 9, 1, 0, 0, 0, 0, 0};
  require(bus.receive_usb_setup(address) == UsbResponse::ack, "Actual SET_ADDRESS SETUP");
  status(bus);
  require(bus.receive_usb_setup(configuration) == UsbResponse::ack,
          "Actual SET_CONFIGURATION SETUP");
  bus.write(config_status, configuration_done);
  status(bus);
  bus.write(mask_low, 1);
}

void native_case(const bool input, const bool halted, const bool consumed) {
  const auto bytes = image(halted);
  McuBus bus(bytes);
  bus.set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
  Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.reset();
  const auto reset = cpu.state();
  require(reset.pc == entry && (reset.ccr & reset_masks) == reset_masks,
          "Actual reset vector fetch and independently specified S/X/I masks");
  prepare(bus);
  for (unsigned instruction = 0; instruction < 4; ++instruction) {
    const auto result = cpu.step();
    require(result.instruction_executed && !result.interrupt_vector,
            "Actual HC12 LDS/LDD/STD instructions retire without an invented IRQ");
  }
  const auto state = cpu.state();
  constexpr unsigned after_stores = entry + 12;
  constexpr unsigned stack = 0x2000;
  constexpr unsigned irq_only = 0x1000;
  constexpr unsigned halt_irq = 0x9000;
  require(state.pc == after_stores && state.sp == stack &&
              state.d == (halted ? halt_irq : irq_only) && state.x == reset.x &&
              state.y == reset.y &&
              (state.ccr & preserved_flags) == (reset.ccr & preserved_flags) &&
              (state.ccr & changed_flags) == (halted ? negative : 0U),
          "Independent native register, addressing and NZV/preserved-flag expectations");
  if (consumed) {
    constexpr std::array<std::uint8_t, 1> payload{0x37};
    const auto response =
        input ? bus.request_usb_in(UsbBuffer::endpoint2).response
              : bus.receive_usb_out({.buffer = UsbBuffer::endpoint3, .data = payload});
    require(response == UsbResponse::stall, "Actual transaction consumes the guest STALL command");
  }
  const auto endpoint = input ? input_address : output_address;
  const UsbTransferRequest request{.sequence = 7,
                                   .address = 0x80,
                                   .length = 2,
                                   .setup = {endpoint_status_input, 0, 0, 0, endpoint, 0, 2, 0},
                                   .output = {},
                                   .short_not_ok = false,
                                   .zero_packet = false};
  UsbHostQueue queue;
  queue.submit(request);
  std::vector<UsbTransferCompletion> completed;
  for (unsigned poll = 0; poll < 4 && completed.empty(); ++poll) {
    completed = queue.poll(bus, {.pc = state.pc, .ccr = state.ccr});
  }
  require(completed.size() == 1 && completed[0].status == UsbTransferStatus::success &&
              completed[0].transferred == 2 &&
              completed[0].input ==
                  std::vector<std::uint8_t>{static_cast<std::uint8_t>(halted), 0} &&
              queue.pending() == 0,
          "Host queue completes exactly one real GET_STATUS data/ACK/status transfer");
  const auto after = cpu.state();
  require(after.pc == state.pc && after.d == state.d && after.x == state.x && after.y == state.y &&
              after.sp == state.sp && after.ccr == state.ccr && (bus.read(status_low) & 3U) == 0 &&
              !bus.interrupt_request(unmasked_ccr),
          "Automatic command does not run a firmware SETUP ISR or change native CPU state");
  const auto expected_control = (halted && !consumed) ? halt_irq : irq_only;
  require(bus.read_memory({.address = static_cast<std::uint16_t>(input ? ep2 : ep3), .size = 2}) ==
              expected_control,
          "Automatic status retains exactly the guest-written data endpoint control");
}
}

int main() {
  unsigned passed = 0;
  unsigned refused = 0;
  for (const bool input : {false, true}) {
    for (const bool halted : {false, true}) {
      for (unsigned phase = 0; phase < (halted ? 2U : 1U); ++phase) {
        try {
          native_case(input, halted, phase != 0);
          ++passed;
        } catch (const std::exception &error) {
          ++refused;
          std::cout << "NATIVE_REFUSED input=" << input << " halted=" << halted
                    << " consumed=" << phase << " reason=" << error.what() << '\n';
        }
      }
    }
  }
  std::cout << "native-endpoint-status passed=" << passed << " refused=" << refused << '\n';
  return refused == 0 ? 0 : 1;
}
