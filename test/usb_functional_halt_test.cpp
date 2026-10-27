// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/mcu_usb.hpp"
#include "usb_functional_halt_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using namespace usb_halt_fixture;
constexpr unsigned status_low = 0x303;
constexpr unsigned setup_tail = 0x33f;
constexpr std::array<std::uint8_t, 8> vendor{0x40, 0x1f, 0, 0, 0, 0, 0, 0};

enum class Transfer : std::uint8_t { bulk = 2, interrupt = 3 };

McuUsb armed(const Case test, const Transfer transfer) {
  auto usb = armed_case(test, {.interrupt = transfer == Transfer::interrupt});
  require(usb.host_endpoint(input_address).type == static_cast<unsigned>(transfer) &&
              usb.host_endpoint(output_address).type == static_cast<unsigned>(transfer),
          "Both configured physical endpoints have the independently selected transfer type");
  return usb;
}

void reject(const auto &operation, const char *reason) {
  try {
    operation();
  } catch (const Fault &error) {
    require(std::string(error.what()).contains(reason),
            "The expected uncertainty must be explicit");
    return;
  }
  throw Fault("Unsupported state cannot fabricate a successful transfer");
}

std::uint16_t endpoint_address(const Case test) {
  return static_cast<std::uint16_t>(test.input ? ep2 : ep3);
}

void retained_state(McuUsb &usb, const Case test) {
  const auto expected = irq_enable | (test.occupied ? valid | payload.size() : 0U);
  require(word(usb, endpoint_address(test)) == expected && usb.interrupt_sources() == 0,
          "Repeated STALL preserves DVALID/size/TCIE and cannot complete a packet");
  const auto endpoint = usb.host_endpoint(test.input ? input_address : output_address);
  require(endpoint.data1 == (!test.input && test.occupied),
          "STALL does not advance the independently expected initial DATA PID");
  usb.write({.address = buffer_select, .value = static_cast<std::uint8_t>(test.input ? 2 : 3)});
  if (test.occupied) {
    for (unsigned index = 0; index < payload.size(); ++index) {
      require(usb.read(static_cast<std::uint16_t>(fifo + index)) == payload[index],
              "Uncompleted endpoint bytes survive functional halt");
    }
  } else {
    reject([&usb] { static_cast<void>(usb.read(fifo)); }, "Unknown USB endpoint-buffer byte");
  }
}

void persistent(const Case test, const bool copy_before_first, const Transfer transfer) {
  auto initial = armed(test, transfer);
  if (!copy_before_first) {
    require(transaction(initial, test) == UsbResponse::stall, "Initial functional STALL");
  }
  auto usb = initial;
  if (copy_before_first) {
    require(transaction(usb, test) == UsbResponse::stall, "Copied primed STALL command");
  }
  for (unsigned retry = 0; retry < 3; ++retry) {
    require(transaction(usb, test) == UsbResponse::stall,
            "Functional halt needs host intervention");
    retained_state(usb, test);
    const auto high =
        static_cast<std::uint8_t>((irq_enable | completed | (test.occupied ? valid : 0U)) >> 8U);
    usb.write({.address = endpoint_address(test), .value = high});
  }
  require(usb.receive_setup(vendor) == UsbResponse::ack, "An unrelated actual control SETUP");
  require(transaction(usb, test) == UsbResponse::stall, "SETUP does not end a bulk halt");
  usb.write({.address = status_low, .value = 1});
  reject([&usb] { static_cast<void>(usb.read(setup_tail)); },
         "setup-buffer contents are not valid");
  usb.observe_sof(1);
  usb.observe_decoder({.suspended = true, .enumerated = true});
  usb.observe_decoder({.enumerated = true});
  require(transaction(usb, test) == UsbResponse::stall, "SOF and suspend/resume retain halt");
  retained_state(usb, test);
}

void select(McuUsb &usb, const bool interface) {
  const std::array<std::uint8_t, 8> request =
      interface ? std::array<std::uint8_t, 8>{1, 11, 0, 0, 0, 0, 0, 0}
                : std::array<std::uint8_t, 8>{0, 9, 1, 0, 0, 0, 0, 0};
  require(usb.receive_setup(request) == UsbResponse::ack, "Actual selection SETUP");
  usb.write({.address = config_status, .value = 0x10});
  const auto packet = usb.request_in(UsbBuffer::control_in);
  require(packet.response == UsbResponse::data && packet.size == 0 && packet.data1,
          "Completed selection returns only genuine DATA1 status");
  usb.finish_in(true);
}

void resumed(McuUsb &usb, const Case test, const bool buffer_retained) {
  require(usb.host_endpoint(test.input ? input_address : output_address).data1 == false,
          "The selection handshake restores DATA0 independently of halt history");
  if (test.input) {
    const auto packet = usb.request_in(UsbBuffer::endpoint2);
    const auto expected = buffer_retained && test.occupied ? UsbResponse::data : UsbResponse::nak;
    require(packet.response == expected, "IN resumes from actual DVALID, not an invented payload");
    if (expected == UsbResponse::data) {
      require(packet.size == payload.size() && !packet.data1 && packet.data[0] == payload[0],
              "Retained IN payload resumes as DATA0");
      usb.finish_in(true);
    }
  } else {
    const auto expected = buffer_retained && test.occupied ? UsbResponse::nak : UsbResponse::ack;
    require(usb.receive_out({.buffer = UsbBuffer::endpoint3, .data = payload}) == expected,
            "OUT resumes according to actual buffer ownership");
  }
}

void selection_reset(const Case test, const bool interface, const Transfer transfer) {
  auto usb = armed(test, transfer);
  require(transaction(usb, test) == UsbResponse::stall, "Initial functional halt");
  select(usb, interface);
  resumed(usb, test, true);
}

void endpoint_reset(const Case test, const Transfer transfer) {
  auto usb = armed(test, transfer);
  require(transaction(usb, test) == UsbResponse::stall, "Initial halt before endpoint reset");
  usb.set_control(0x8000);
  usb.reset_endpoints();
  usb.set_control(module_enable);
  const auto frozen = usb;
  for (unsigned replay = 0; replay < 2; ++replay) {
    usb = frozen;
    reject([&] { static_cast<void>(transaction(usb, test)); },
           "functional halt after endpoint reset");
    select(usb, false);
    resumed(usb, test, false);
  }
}

void bus_reset(const Case test, const Transfer transfer) {
  auto usb = armed(test, transfer);
  require(transaction(usb, test) == UsbResponse::stall, "Initial halt before bus reset");
  usb.observe_decoder({.reset = true});
  usb.observe_decoder({.enumerated = true});
  constexpr std::array<std::uint8_t, 8> address{0, 5, device_address, 0, 0, 0, 0, 0};
  require(usb.receive_setup(address) == UsbResponse::ack, "Bus reset requires actual addressing");
  require(usb.request_in(UsbBuffer::control_in).response == UsbResponse::data,
          "Actual address status");
  usb.finish_in(true);
  select(usb, false);
  resumed(usb, test, false);
}

void protocol_stall() {
  auto usb = configured();
  constexpr std::array<std::uint8_t, 8> unsupported_optional{0, 7, 0, 0, 0, 0, 0, 0};
  require(usb.receive_setup(unsupported_optional) == UsbResponse::ack, "Actual optional SETUP");
  for (unsigned retry = 0; retry < 3; ++retry) {
    require(usb.request_in(UsbBuffer::control_in).response == UsbResponse::stall,
            "Control protocol STALL lasts through the current request");
  }
  require(usb.receive_setup(vendor) == UsbResponse::ack &&
              usb.request_in(UsbBuffer::control_in).response == UsbResponse::nak,
          "A new control SETUP ends protocol STALL without supplying invented data");
}

void selective_reset(const bool interrupt, const std::uint8_t interface) {
  auto usb = configured({.interrupt = interrupt, .separate_interfaces = true});
  usb.write({.address = ep2, .value = 0x80});
  usb.write({.address = ep3, .value = 0x80});
  require(usb.request_in(UsbBuffer::endpoint2).response == UsbResponse::stall &&
              usb.receive_out({.buffer = UsbBuffer::endpoint3, .data = payload}) ==
                  UsbResponse::stall,
          "Both distinct interface endpoints enter actual functional halt");
  const std::array<std::uint8_t, 8> request{1, 11, 0, 0, interface, 0, 0, 0};
  require(usb.receive_setup(request) == UsbResponse::ack, "Actual same-alternate SET_INTERFACE");
  usb.write({.address = config_status, .value = 0x10});
  const auto status = usb.request_in(UsbBuffer::control_in);
  require(status.response == UsbResponse::data && status.size == 0 && status.data1,
          "Real selection status before probing either affected endpoint");
  usb.finish_in(true);
  const auto input = usb.request_in(UsbBuffer::endpoint2).response;
  const auto output = usb.receive_out({.buffer = UsbBuffer::endpoint3, .data = payload});
  require(input == (interface == 0 ? UsbResponse::nak : UsbResponse::stall) &&
              output == (interface == 1 ? UsbResponse::ack : UsbResponse::stall),
          "SET_INTERFACE clears exactly its own halt; the other interface stays halted");
  require(word(usb, ep2) == 0 &&
              word(usb, ep3) == (interface == 1 ? valid | completed | payload.size() : 0U),
          "Only the newly accepted OUT packet changes data ownership and completion");
}
}

int main() {
  try {
    unsigned cases = 0;
    for (const auto transfer : {Transfer::bulk, Transfer::interrupt}) {
      for (const bool input : {false, true}) {
        for (const bool occupied : {false, true}) {
          const Case test{.input = input, .occupied = occupied};
          for (const bool alternative : {false, true}) {
            persistent(test, alternative, transfer);
            selection_reset(test, alternative, transfer);
            cases += 2;
          }
          endpoint_reset(test, transfer);
          bus_reset(test, transfer);
          cases += 2;
        }
      }
    }
    protocol_stall();
    constexpr std::array<std::uint8_t, 2> interfaces{0, 1};
    for (const bool interrupt : {false, true}) {
      for (const auto interface : interfaces) {
        selective_reset(interrupt, interface);
        ++cases;
      }
    }
    std::cout << "Directed functional-halt cases=" << cases
              << " copies/payload/PID/flags=retained EPRST=explicitly-unqualified "
                 "protocol-STALL=separate\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
