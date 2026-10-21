// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/usb_device.hpp"
#include "blofeld/usbip_export.hpp"
#include "usb_device_fixture.hpp"
#include "usb_endpoint_reset_fixture.hpp"
#include "usb_host_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace {
using namespace blofeld;
using namespace blofeld::test::usb_device;
constexpr unsigned entry = 0x8000;
constexpr unsigned setup_handler = 0x9000;
constexpr unsigned device_source = 0x9100;
constexpr unsigned configuration_source = 0x9180;
constexpr unsigned input_handler = 0x9200;
constexpr unsigned output_handler = 0x9240;
constexpr unsigned device_copy = 0x9400;
constexpr unsigned configuration_copy = 0x9500;
constexpr unsigned setup_vector = 0xffd2;
constexpr unsigned input_vector = 0xffce;
constexpr unsigned output_vector = 0xffcc;
constexpr unsigned marker = 0x1200;
constexpr unsigned length_marker = marker + 2;
constexpr unsigned type_marker = marker + 3;
constexpr unsigned setup_instructions = 80;
constexpr unsigned full_speed_maximum = 64;
constexpr CpuState initial{
    .pc = entry, .d = 0x1234, .x = 0x5678, .y = 0x9abc, .sp = 0x2000, .ccr = 0x40};
enum class Opcode : std::uint8_t {
  load_d = 0xfc,
  store_d = 0x7c,
  load_a = 0xb6,
  load_b = 0xf6,
  store_a = 0x7a,
  jump = 0x06
};

void instruction(Bytes &bytes, const Opcode opcode, const unsigned operand) {
  bytes.push_back(static_cast<std::uint8_t>(opcode));
  bytes.push_back(static_cast<std::uint8_t>(operand >> 8U));
  bytes.push_back(static_cast<std::uint8_t>(operand & byte_mask));
}

Bytes copy_handler(const unsigned source) {
  Bytes code;
  for (unsigned offset = 0; offset < full_speed_maximum; offset += 2) {
    instruction(code, Opcode::load_d, source + offset);
    instruction(code, Opcode::store_d, local + offset);
  }
  constexpr std::uint8_t load_a_immediate = 0x86;
  constexpr std::uint8_t valid_input = 0x50;
  constexpr std::uint8_t rti = 0x0b;
  code.insert(code.end(), {load_a_immediate, valid_input});
  instruction(code, Opcode::load_b, length_marker);
  instruction(code, Opcode::store_d, endpoint_base);
  code.push_back(rti);
  return code;
}

std::array<std::uint8_t, 32768> firmware() {
  std::array<std::uint8_t, 32768> bytes{};
  const auto vector = [&bytes](const MemoryWrite v) {
    bytes.at(v.address - entry) = static_cast<std::uint8_t>(v.value >> 8U);
    bytes.at(v.address - entry + 1) = static_cast<std::uint8_t>(v.value & byte_mask);
  };
  vector({.address = setup_vector, .value = setup_handler});
  vector({.address = input_vector, .value = input_handler});
  vector({.address = output_vector, .value = output_handler});
  Bytes code;
  constexpr unsigned setup_buffer = 0x338;
  instruction(code, Opcode::load_d, setup_buffer);
  instruction(code, Opcode::store_d, marker);
  instruction(code, Opcode::load_a, setup_buffer + 6);
  instruction(code, Opcode::store_a, length_marker);
  instruction(code, Opcode::load_a, setup_buffer + 3);
  instruction(code, Opcode::store_a, type_marker);
  constexpr std::array<std::uint8_t, 2> acknowledge_setup{0x86, 1};
  code.insert(code.end(), acknowledge_setup.begin(), acknowledge_setup.end());
  instruction(code, Opcode::store_a, status + 1);
  instruction(code, Opcode::load_a, type_marker);
  // CMPA #device_type; BNE skips the absolute device-copy JMP.
  constexpr std::array<std::uint8_t, 4> select_configuration{0x81, 1, 0x26, 3};
  code.insert(code.end(), select_configuration.begin(), select_configuration.end());
  instruction(code, Opcode::jump, device_copy);
  instruction(code, Opcode::jump, configuration_copy);
  std::ranges::copy(code, bytes.begin() + (setup_handler - entry));
  auto own_device = device;
  own_device[7] = full_speed_maximum;
  std::ranges::copy(own_device, bytes.begin() + (device_source - entry));
  std::ranges::copy(configuration, bytes.begin() + (configuration_source - entry));
  std::ranges::copy(copy_handler(device_source), bytes.begin() + (device_copy - entry));
  std::ranges::copy(copy_handler(configuration_source),
                    bytes.begin() + (configuration_copy - entry));
  constexpr std::array<std::uint8_t, 6> ack_input{0x86, 0x30, 0x7a, 3, 0x14, 0x0b};
  constexpr std::array<std::uint8_t, 6> ack_output{0x86, 0x30, 0x7a, 3, 0x16, 0x0b};
  std::ranges::copy(ack_input, bytes.begin() + (input_handler - entry));
  std::ranges::copy(ack_output, bytes.begin() + (output_handler - entry));
  return bytes;
}

void interrupt(Hc12 &cpu, const InterruptRequest expected, const unsigned instructions) {
  const auto entered = cpu.step();
  require(!entered.instruction_executed && entered.interrupt_vector == expected.vector,
          "Actual controller priority selects the independently expected USB IRQ");
  for (unsigned i = 0; i < instructions; ++i) { cpu.step(); }
  const auto s = cpu.state();
  require(
      s.pc == initial.pc && s.d == initial.d && s.x == initial.x && s.y == initial.y &&
          s.sp == initial.sp && s.ccr == initial.ccr,
      "Every own native IRQ restores the supplied independent register state through actual RTI");
}

void test_native_descriptor_export() {
  const auto flash = firmware();
  McuBus bus(flash);
  prepare(bus, full_speed_maximum);
  bus.write(test::endpoint_reset::control, test::endpoint_reset::mce_high);
  constexpr unsigned counts = 0x21;
  test::endpoint_reset::configure(bus, {.bank = 8, .value = counts});
  bus.write(test::endpoint_reset::control, test::endpoint_reset::men_high);
  bus.write(mask + 1, 1);
  Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.set_state(initial);
  UsbDeviceProbe probe;
  constexpr std::array<unsigned, 3> lengths{18, 9, 55};
  for (unsigned stage = 0; stage < lengths.size(); ++stage) {
    const ExecutionContext current{.pc = cpu.state().pc, .ccr = cpu.state().ccr};
    require(!probe.poll(bus, current),
            "Each stage begins with actual SETUP, never host-supplied data");
    interrupt(cpu, {.vector = setup_vector}, setup_instructions);
    constexpr unsigned get_descriptor_word = 0x8006;
    require(word(bus, marker) == get_descriptor_word && bus.read(length_marker) == lengths[stage],
            "Own ISR reads and records the actual request and independently expected length");
    require(!probe.poll(bus, current), "IN payload is still awaiting actual control status");
    interrupt(cpu, {.vector = input_vector}, 3);
    require(probe.poll(bus, current) == (stage == 2),
            "Capture becomes available only after all three status handshakes");
    interrupt(cpu, {.vector = output_vector}, 3);
  }
  auto expected_device = device;
  expected_device[7] = full_speed_maximum;
  require(std::ranges::equal(probe.descriptors().device(), expected_device) &&
              std::ranges::equal(probe.descriptors().configuration(), configuration),
          "All captured bytes originate in actual own guest ROM loads and USB RAM stores");
  UsbipExport exported(probe.descriptors(),
                       {.path = "/blofeld-emu/usb/1-2", .busid = "1-2", .bus = 1, .device = 2},
                       bus,
                       {.pc = cpu.state().pc, .ccr = cpu.state().ccr});
  const auto reply = exported.reply(UsbipImportRequest{.busid = "1-2"});
  constexpr unsigned import_reply_bytes = 320;
  require(reply.imported && !reply.close && reply.bytes.size() == import_reply_bytes &&
              !cpu.pending_interrupt(),
          "Native descriptor capture plus actual MCU configuration yields an import record without "
          "a fake guest responder");
}
}

int main() {
  try {
    test_native_descriptor_export();
    std::cout << "Verified three descriptor requests through own HC12 ISRs, actual status/IRQs and "
                 "USB/IP import metadata. No original enumeration/host attachment claim.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
