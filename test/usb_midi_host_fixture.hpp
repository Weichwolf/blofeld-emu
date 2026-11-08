// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "cpu12/memory.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "support/isa.hpp"
#include "usb/usb_host_link.hpp"
#include "usb/usb_midi_host.hpp"
#include "usb/usb_transfer.hpp"
#include "usb_device_native_fixture.hpp"
#include "usb_endpoint_reset_fixture.hpp"
#include "usb_host_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

namespace blofeld::test::usb_midi_host {
namespace native = usb_device_native;
namespace isa = cpu12::isa;
using cpu12::Byte;
using isa::Opcode;
using usb_host::require;
inline constexpr unsigned config_vector = 0xffd0;
inline constexpr unsigned in_vector = 0xffca;
inline constexpr unsigned out_vector = 0xffc8;
inline constexpr unsigned config_handler = 0x9300;
inline constexpr unsigned in_handler = 0x9340;
inline constexpr unsigned out_handler = 0x9380;
inline constexpr unsigned echo = 0x1210;
inline constexpr unsigned maximum_polls = 100;
inline constexpr unsigned enumeration_timeout = 1000;
inline constexpr unsigned midi_out_instructions = 17;
inline constexpr unsigned local_buffer = 0x380;
inline constexpr unsigned packet_size = 64;
// Own structural interface/endpoints; no claim of complete class-specific USB-MIDI descriptors.
inline constexpr std::array<Byte, 41> configuration{
    9, 2, 41, 0, 2, 1, 0, 0x80, 25, 9, 4,           0, 0, 0, 1, 1,    0, 0,           9, 4, 1,
    0, 2, 1,  3, 0, 0, 7, 5,    1,  2, packet_size, 0, 0, 7, 5, 0x82, 2, packet_size, 0, 0};
inline constexpr std::array<Byte, 4> note{9, 0x90, 60, 100};

constexpr std::array<Byte, 2> address(const unsigned value) {
  constexpr unsigned byte_mask = 0xff;
  return {static_cast<Byte>(value >> 8U), static_cast<Byte>(value & byte_mask)};
}

inline std::array<Byte, 32768> image() {
  auto bytes = native::firmware();
  const auto put = [&bytes](const unsigned location, const auto &data) {
    std::ranges::copy(data, bytes.begin() + (location - native::entry));
  };
  put(native::configuration_source, configuration);
  put(config_vector, address(config_handler));
  put(in_vector, address(in_handler));
  put(out_vector, address(out_handler));
  const auto ack = [](const unsigned target) consteval {
    constexpr Byte complete_irq = 0x30;
    return isa::program(isa::opcode<Opcode::ldaa_immediate>(),
                        std::array<Byte, 1>{complete_irq},
                        isa::opcode<Opcode::staa_extended>(),
                        address(target),
                        isa::rti);
  };
  constexpr unsigned configuration_status = 0x30c;
  constexpr unsigned input_control = 0x318;
  constexpr unsigned output_control = 0x31a;
  constexpr unsigned buffer_select = 0x30f;
  constexpr Byte configuration_done = 0x10;
  constexpr Byte complete_irq = 0x30;
  constexpr unsigned ready_four = 0x7004;
  put(config_handler,
      isa::program(isa::opcode<Opcode::ldaa_immediate>(),
                   std::array<Byte, 1>{configuration_done},
                   isa::opcode<Opcode::staa_extended>(),
                   address(configuration_status),
                   isa::rti));
  put(in_handler, ack(input_control));
  put(out_handler,
      isa::program(isa::opcode<Opcode::ldaa_immediate>(),
                   std::array<Byte, 1>{3},
                   isa::opcode<Opcode::staa_extended>(),
                   address(buffer_select),
                   isa::opcode<Opcode::ldd_extended>(),
                   address(local_buffer),
                   isa::opcode<Opcode::std_extended>(),
                   address(echo),
                   isa::opcode<Opcode::ldd_extended>(),
                   address(local_buffer + 2),
                   isa::opcode<Opcode::std_extended>(),
                   address(echo + 2),
                   isa::opcode<Opcode::ldaa_immediate>(),
                   std::array<Byte, 1>{complete_irq},
                   isa::opcode<Opcode::staa_extended>(),
                   address(output_control),
                   isa::opcode<Opcode::ldaa_immediate>(),
                   std::array<Byte, 1>{2},
                   isa::opcode<Opcode::staa_extended>(),
                   address(buffer_select),
                   isa::opcode<Opcode::ldd_extended>(),
                   address(echo),
                   isa::opcode<Opcode::std_extended>(),
                   address(local_buffer),
                   isa::opcode<Opcode::ldd_extended>(),
                   address(echo + 2),
                   isa::opcode<Opcode::std_extended>(),
                   address(local_buffer + 2),
                   isa::opcode<Opcode::ldd_immediate>(),
                   address(ready_four),
                   isa::opcode<Opcode::std_extended>(),
                   address(input_control),
                   isa::rti));
  return bytes;
}

struct Fixture {
  Fixture() : firmware(image()), bus(firmware), cpu(bus, BLOFELD_DEFAULT_SLA) {
    native::prepare_native(bus);
    bus.write(endpoint_reset::control, endpoint_reset::mce_high);
    constexpr unsigned in_description = 0x08d2;
    constexpr unsigned out_description = 0x08c1;
    constexpr unsigned packet_field = packet_size << 19U;
    endpoint_reset::configure(bus, {.bank = 2, .value = packet_field | in_description});
    endpoint_reset::configure(bus, {.bank = 3, .value = packet_field | out_description});
    bus.write(endpoint_reset::control, endpoint_reset::men_high);
    cpu.set_state(native::initial);
    for (unsigned frame = 0; frame <= recovery_end; ++frame) {
      require(link.poll(bus, frame) == (frame == recovery_end), "Attach the actual own host link");
    }
    for (unsigned index = 0; index < 4; ++index) {
      constexpr Byte acknowledged = 0x30;
      bus.write(static_cast<std::uint16_t>(usb_host::endpoint_base + (index * 2)), acknowledged);
    }
    bus.write(usb_host::mask, 0x20); // SETECRIE; descriptor SETUPIE is already enabled.
    tick = recovery_end;
  }

  void service() {
    unsigned serviced = 0;
    while (const auto request = cpu.pending_interrupt()) {
      require(++serviced <= 16, "Own native handlers must acknowledge every pending USB interrupt");
      unsigned instructions = 3;
      if (request->vector == native::setup_vector) {
        instructions = native::setup_instructions;
      } else if (request->vector == out_vector) {
        instructions = midi_out_instructions;
      } else {
        require(request->vector == config_vector || request->vector == in_vector ||
                    request->vector == native::input_vector ||
                    request->vector == native::output_vector,
                "Only expected own USB handlers run");
      }
      native::interrupt(cpu, *request, instructions);
    }
  }

  std::vector<UsbTransferCompletion> poll(const bool run_firmware = true) {
    ++tick;
    require(link.poll(bus, tick), "Connected host delivers each device-time SOF");
    auto result = host.poll(bus, {.pc = cpu.state().pc, .ccr = cpu.state().ccr}, tick);
    if (run_firmware) { service(); }
    return result;
  }

  void enumerate() {
    for (unsigned index = 0; index < maximum_polls && !host.ready(); ++index) {
      static_cast<void>(poll());
    }
    require(host.ready(), "Actual native descriptor/configuration IRQs complete enumeration");
  }

  static constexpr unsigned recovery_end = 210;
  std::array<Byte, 32768> firmware;
  McuBus bus;
  Hc12 cpu;
  UsbHostLink link{1};
  UsbMidiHost host{1, enumeration_timeout};
  std::uint64_t tick = 0;
};
}
