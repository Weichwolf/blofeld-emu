// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/hc12.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/mcu_usb.hpp"
#include "usb_endpoint_reset_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <vector>

namespace {
using namespace blofeld::test::endpoint_reset;

void check_pid_barriers(blofeld::McuBus &bus) {
  require(bus.request_usb_in(blofeld::UsbBuffer::endpoint2).response == blofeld::UsbResponse::nak,
          "A reset empty IN endpoint has a defined NAK without inventing a DATA PID");
  reject([&] { bus.finish_usb_in(true); }, "without an outstanding data packet");
  bus.write(endpoint + 6, 0x10);
  reject(
      [&] {
        static_cast<void>(
            bus.receive_usb_out({.buffer = blofeld::UsbBuffer::endpoint3, .data = payload}));
      },
      "Unknown USB OUT DATA PID after endpoint reset");
  put(bus, {.address = endpoint + 4, .size = 2, .value = software_ready});
  reject([&] { static_cast<void>(bus.request_usb_in(blofeld::UsbBuffer::endpoint2)); },
         "Unknown USB IN DATA PID after endpoint reset");
  reject([&] { static_cast<void>(bus.read(setup_buffer)); }, "setup-buffer contents are not valid");
}

void test_native_reset_and_joint_replays() {
  const auto image = own_image();
  blofeld::McuMachine machine(image, BLOFELD_DEFAULT_SLA);
  prepare(machine.bus());
  populate(machine.bus());
  machine.bus().write(control, mce_high);
  constexpr unsigned initial_d = 0x1234;
  constexpr unsigned initial_x = 0x5678;
  constexpr unsigned initial_y = 0x9abc;
  constexpr unsigned initial_flags = 0xd5;
  machine.cpu().set_state({.pc = own_entry,
                           .d = initial_d,
                           .x = initial_x,
                           .y = initial_y,
                           .sp = own_stack,
                           .ccr = initial_flags});
  const auto loaded = machine.cpu().step();
  require(loaded.instruction_executed && !loaded.interrupt_vector &&
              machine.cpu().state().pc == store_pc,
          "Native own LDD supplies the command operand before the real store");
  const auto frozen = machine.checkpoint();
  std::vector<unsigned> expected;
  for (unsigned attempt = 0; attempt < 3; ++attempt) {
    if (attempt != 0) { machine.restore(frozen); }
    const auto stored = machine.cpu().step();
    constexpr unsigned command_word = 0x4208;
    constexpr unsigned expected_flags = 0xd1;
    const auto state = machine.cpu().state();
    require(stored.instruction_executed && !stored.interrupt_vector && state.pc == after_store &&
                state.d == command_word && state.x == initial_x && state.y == initial_y &&
                state.sp == own_stack && state.ccr == expected_flags,
            "The actual native STD issues EPRST and preserves the independent CPU endpoint");
    verify_reset(machine.bus());
    const auto observed = inspect(machine.bus());
    if (attempt == 0) { expected = observed; }
    require(observed == expected,
            "Two joint owner replays retain all declared register/payload/clock observations");
    const auto now = machine.bus().power_off_flash_image();
    require(now.bytes == image.bytes && now.known == image.known && now.fresh == image.fresh,
            "Native peripheral resets preserve all NVM bytes, knowledge and erase history");
  }
  const auto after_reset = machine.checkpoint();
  for (unsigned branch = 0; branch < 2; ++branch) {
    machine.restore(after_reset);
    complete_staging(machine.bus());
    enable(machine.bus());
    check_pid_barriers(machine.bus());
    activate_bulk(machine.bus());
    const auto packet = machine.bus().request_usb_in(blofeld::UsbBuffer::endpoint2);
    require(packet.response == blofeld::UsbResponse::data && !packet.data1 && packet.size == 4 &&
                packet.data[0] == 128,
            "A genuine decoded configuration acknowledgement reestablishes bulk DATA0");
    machine.bus().finish_usb_in(true);
    require(machine.bus().receive_usb_setup(request) == blofeld::UsbResponse::ack,
            "A new actual SETUP reestablishes the default endpoints' DATA1");
    put(machine.bus(), {.address = endpoint, .size = 2, .value = software_ready});
    const auto control_packet = machine.bus().request_usb_in(blofeld::UsbBuffer::control_in);
    require(control_packet.response == blofeld::UsbResponse::data && control_packet.data1,
            "Control data resumes only with its established SETUP PID");
  }
}

void test_command_gates() {
  blofeld::McuMachine machine(own_image(), BLOFELD_DEFAULT_SLA);
  prepare(machine.bus());
  populate(machine.bus());
  constexpr unsigned in_flight_control = 0x5004;
  machine.bus().write(control, enabled_eprst_high);
  require(word(machine.bus(), endpoint + 4) == in_flight_control &&
              machine.bus().read(control) == men_high,
          "MEN=1 ignores EPRST without clearing an in-flight endpoint");
  machine.bus().write(control, eprst_high);
  require(word(machine.bus(), endpoint + 4) == in_flight_control &&
              machine.bus().read(control) == mce_high,
          "Clearing MEN in the command write does not bypass the prior-MEN gate");
  machine.bus().write(control, 0);
  machine.bus().write(control, 2);
  require(word(machine.bus(), endpoint + 4) == in_flight_control &&
              machine.bus().read(control) == 0,
          "MCE=0 ignores EPRST without a reset side effect");
  reject([&] { machine.bus().write(control, eprst_high); },
         "Unqualified USB EPRST with simultaneous MCE enable");
  require(word(machine.bus(), endpoint + 4) == in_flight_control &&
              machine.bus().read(control) == 0,
          "Unqualified simultaneous clock enable rejects before either owner changes");
  machine.bus().write(control, mce_high);
  for (const auto command : {0x62U, 0x52U, 0x46U}) {
    reject([&] { machine.bus().write(control, static_cast<std::uint8_t>(command)); },
           "Unimplemented USB");
    require(word(machine.bus(), endpoint + 4) == in_flight_control &&
                machine.bus().read(control) == mce_high,
            "Mixed unsupported commands reject before any EPRST or clock mutation");
  }
  machine.bus().write(control, mce_high);
  require(word(machine.bus(), endpoint + 4) == in_flight_control,
          "A zero command bit has no endpoint-reset side effect");
  machine.bus().write(control, eprst_high);
  verify_reset(machine.bus());
}

void test_unknown_payload_stays_unknown() {
  blofeld::McuMachine machine(own_image(), BLOFELD_DEFAULT_SLA);
  prepare(machine.bus());
  reset_command(machine.bus());
  enable(machine.bus());
  require(machine.bus().receive_usb_setup(request) == blofeld::UsbResponse::ack,
          "A new SETUP establishes a PID without supplying endpoint payload bytes");
  put(machine.bus(), {.address = endpoint, .size = 2, .value = software_ready});
  reject([&] { static_cast<void>(machine.bus().request_usb_in(blofeld::UsbBuffer::control_in)); },
         "Unknown USB IN endpoint-buffer data");
}

void test_optional_control_and_bus_reset_recovery() {
  blofeld::McuMachine machine(own_image(), BLOFELD_DEFAULT_SLA);
  prepare(machine.bus());
  constexpr std::array<std::uint8_t, 8> set_descriptor{0, 7, 0, 1, 0, 0, 0, 0};
  require(machine.bus().receive_usb_setup(set_descriptor) == blofeld::UsbResponse::ack &&
              machine.bus().request_usb_in(blofeld::UsbBuffer::control_in).response ==
                  blofeld::UsbResponse::stall,
          "An unsupported optional standard request first establishes its actual STALL response");
  reset_command(machine.bus());
  enable(machine.bus());
  reject([&] { static_cast<void>(machine.bus().request_usb_in(blofeld::UsbBuffer::control_in)); },
         "Unqualified USB optional control response after endpoint reset");
  reject([&] { static_cast<void>(machine.bus().receive_usb_out({.data = {}})); },
         "Unqualified USB optional control response after endpoint reset");
  require(machine.bus().receive_usb_setup(request) == blofeld::UsbResponse::ack &&
              machine.bus().request_usb_in(blofeld::UsbBuffer::control_in).response ==
                  blofeld::UsbResponse::nak,
          "A new supported SETUP clears the old optional-request ambiguity");
  reset_command(machine.bus());
  enable(machine.bus());
  machine.bus().observe_usb_decoder({.reset = true});
  machine.bus().observe_usb_decoder({.enumerated = true});
  activate_bulk(machine.bus());
  require(machine.bus().receive_usb_out({.buffer = blofeld::UsbBuffer::endpoint3,
                                         .data = payload}) == blofeld::UsbResponse::ack,
          "A real decoded bus reset and configuration establish bulk DATA0 again");
}
}

int main() {
  try {
    test_native_reset_and_joint_replays();
    test_command_gates();
    test_unknown_payload_stays_unknown();
    test_optional_control_and_bus_reset_recovery();
    std::cout << "Verified native EPRST, command gates, exact reset domain, retained payloads, "
                 "pending ownership cancellation, conservative DATA PID/control-phase barriers "
                 "and joint replays. No MRST/PHY-reset, IQUE data, wire timing or physical "
                 "hardware claim.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
