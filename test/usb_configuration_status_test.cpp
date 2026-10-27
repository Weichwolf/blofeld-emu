// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/mcu_usb.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
constexpr unsigned module_control = 0xc000, selector = 0x30e, config_data = 0x310;
constexpr unsigned configuration_status = 0x30c, endpoint = 0x314, irq_mask = 0x304;
constexpr unsigned word_mask = 0xffff, setup_buffer = 0x338, byte_mask = 0xff;
constexpr unsigned status_low = 0x303, configuration_setup_enable = 0x2003;
constexpr unsigned done = 0x10, valid = 0x4000, transfer_complete = 0x2000, interrupt = 0x1000;
constexpr unsigned configuration_irq = 4, control_irq = 0x10;
constexpr std::array<std::uint8_t, 8> select_config{0, 9, 1, 0, 0, 0, 0, 0};
constexpr std::array<std::uint8_t, 8> select_interface{1, 11, 0, 0, 0, 0, 0, 0};
constexpr std::array<std::uint8_t, 8> clear_config{0, 9, 0, 0, 0, 0, 0, 0};
constexpr std::array<std::uint8_t, 8> vendor{0x40, 0x1f, 0, 0, 0, 0, 0, 0};

void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

void rejects(const auto &operation) {
  try {
    operation();
  } catch (const Fault &) { return; }
  throw Fault("Expected visible USB configuration-status rejection");
}

unsigned word(const McuUsb &usb, const unsigned address) {
  return (static_cast<unsigned>(usb.read(static_cast<std::uint16_t>(address))) << 8U) |
         usb.read(static_cast<std::uint16_t>(address + 1));
}

void write_word(McuUsb &usb, const std::uint16_t address, const unsigned value) {
  usb.write({.address = address, .value = static_cast<std::uint8_t>(value >> 8U)});
  usb.write({.address = static_cast<std::uint16_t>(address + 1),
             .value = static_cast<std::uint8_t>(value & byte_mask)});
}

struct Configuration {
  unsigned index = 0;
  std::uint32_t value = 0;
};

void config(McuUsb &usb, const Configuration request) {
  usb.write({.address = selector, .value = static_cast<std::uint8_t>(request.index)});
  usb.write_memory({.address = config_data, .size = 2, .value = request.value >> 16U});
  usb.write_memory({.address = config_data + 2, .size = 2, .value = request.value & word_mask});
}

McuUsb start() {
  McuUsb usb;
  constexpr unsigned default_endpoint = 0x02000000;
  constexpr unsigned configuration_counts = 0x11;
  constexpr unsigned alternate_counts = 0x11;
  config(usb, {.index = 1, .value = default_endpoint});
  config(usb, {.index = 8, .value = configuration_counts});
  config(usb, {.index = 9, .value = alternate_counts});
  usb.observe_decoder({});
  usb.set_control(module_control);
  usb.observe_decoder({.enumerated = true});
  write_word(usb, irq_mask, configuration_setup_enable);
  return usb;
}

void packet(McuUsb &usb) {
  const auto result = usb.request_in(UsbBuffer::control_in);
  require(result.response == UsbResponse::data && result.size == 0 && result.data1,
          "USB2.0 no-data completion is exactly one zero-length DATA1 packet");
}

void acknowledge(McuUsb &usb) {
  usb.write({.address = configuration_status, .value = done});
}

void configure_first(McuUsb &usb) {
  require(usb.receive_setup(select_config) == UsbResponse::ack, "Initial SETUP is received");
  acknowledge(usb);
  packet(usb);
  usb.finish_in(true);
  usb.write({.address = endpoint, .value = 0x20});
}

void matrix(const unsigned request,
            const bool enabled,
            const bool data_valid,
            const unsigned size) {
  auto usb = start();
  if (request == 1) { configure_first(usb); }
  constexpr std::array requests{select_config, select_interface, clear_config};
  const auto &setup = requests.at(request);
  require(usb.receive_setup(setup) == UsbResponse::ack &&
              usb.interrupt_sources() == configuration_irq,
          "A real decoded request raises only SETECR before firmware acknowledgement");
  const auto before = (enabled ? interrupt : 0U) | (data_valid ? valid : 0U) | size;
  write_word(usb, endpoint, before);
  require(usb.request_in(UsbBuffer::control_in).response == UsbResponse::nak &&
              word(usb, endpoint) == before,
          "SETECR keeps status NAK even with pending arbitrary local-buffer data");
  rejects([&] { usb.finish_in(true); });
  usb.write({.address = configuration_status, .value = 0});
  require(usb.interrupt_sources() == configuration_irq, "DONE=0 cannot release the hardware gate");
  acknowledge(usb);
  const auto state = usb;
  for (unsigned replay = 0; replay < 2; ++replay) {
    usb = state;
    for (unsigned retry = 0; retry < 3; ++retry) {
      packet(usb);
      require(word(usb, endpoint) == before && usb.interrupt_sources() == 0,
              "Status production neither consumes local data nor publishes a completed transfer");
      const auto in_flight = usb;
      packet(usb);
      usb = in_flight;
      usb.finish_in(false);
      require(word(usb, endpoint) == before && usb.interrupt_sources() == 0,
              "Missing host ACK preserves all endpoint controls and the status phase");
    }
    packet(usb);
    const auto pending = usb;
    usb.finish_in(true);
    const auto completed = (before & ~valid) | transfer_complete;
    require(word(usb, endpoint) == completed &&
                usb.interrupt_sources() == (enabled ? control_irq : 0U) &&
                usb.host_endpoint(0x80).data1 == false,
            "Real ACK alone clears DVALID,sets TFRC,toggles DATA PID and gates the native IRQ");
    require(usb.request_in(UsbBuffer::control_in).response == UsbResponse::nak,
            "The acknowledged automatic status is not transmitted again");
    rejects([&] { usb.finish_in(true); });
    rejects([&] { static_cast<void>(usb.read(setup_buffer)); });
    usb = pending;
    usb.finish_in(true);
    require(word(usb, endpoint) == completed, "An in-flight trusted copy replays ACK effects");
    usb.write(
        {.address = endpoint, .value = static_cast<std::uint8_t>(0x20U | (enabled ? 0x10U : 0U))});
    require(usb.interrupt_sources() == 0 && word(usb, endpoint) == (completed & ~transfer_complete),
            "Firmware W1C clears exactly the completed-transfer IRQ");
  }
}

void cancellation() {
  auto usb = start();
  require(usb.receive_setup(select_config) == UsbResponse::ack, "SETUP reaches the module");
  acknowledge(usb);
  packet(usb);
  require(usb.receive_setup(vendor) == UsbResponse::ack && usb.read(setup_buffer) == vendor[0],
          "New SETUP aborts the old status and supplies actual new setup data");
  rejects([&] { usb.finish_in(true); });
  require(usb.request_in(UsbBuffer::control_in).response == UsbResponse::nak,
          "A replacement request does not inherit the old automatic status response");
  usb.write({.address = status_low, .value = 1});
  rejects([&] { static_cast<void>(usb.read(setup_buffer)); });
  require(usb.receive_setup(select_config) == UsbResponse::ack,
          "Next configuration SETUP is legal");
  acknowledge(usb);
  packet(usb);
  usb.observe_decoder({.reset = true});
  rejects([&] { usb.finish_in(true); });
  usb.observe_decoder({.enumerated = true});
  require(word(usb, configuration_status) == 0 && word(usb, endpoint) == 0 &&
              usb.request_in(UsbBuffer::control_in).response == UsbResponse::nak,
          "Actual USB reset cancels status/configuration without a completed packet");
}

void stall() {
  auto usb = start();
  require(usb.receive_setup(select_config) == UsbResponse::ack, "SETUP reaches configuration");
  acknowledge(usb);
  usb.write({.address = endpoint, .value = 0x80});
  require(usb.request_in(UsbBuffer::control_in).response == UsbResponse::stall,
          "An explicitly stalled control endpoint must not return successful status");
  rejects([&] { usb.finish_in(true); });
  require(usb.receive_setup(select_config) == UsbResponse::ack, "New SETUP clears control STALL");
  acknowledge(usb);
  packet(usb);
  usb.finish_in(true);
}
}

int main() {
  try {
    for (unsigned request = 0; request < 3; ++request) {
      for (const bool enabled : {false, true}) {
        for (const bool valid_data : {false, true}) {
          for (const unsigned size : {0U, 7U, 63U}) { matrix(request, enabled, valid_data, size); }
        }
      }
    }
    cancellation();
    stall();
    std::cout << "38 USB configuration-status scenarios passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
