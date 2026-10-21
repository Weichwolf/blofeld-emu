// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/usb_host.hpp"
#include "blofeld/usb_transfer.hpp"
#include "blofeld/usbip.hpp"
#include "usb_host_fixture.hpp"
#include "usbip_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <variant>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::usb_host;
constexpr unsigned entry = 0x8000;
constexpr unsigned handler = 0x9000;
constexpr unsigned descriptor_source = 0x9100;
constexpr unsigned input_handler = 0x9200;
constexpr unsigned output_handler = 0x9240;
constexpr unsigned setup_vector = 0xffd2;
constexpr unsigned input_vector = 0xffce;
constexpr unsigned output_vector = 0xffcc;
constexpr unsigned setup_buffer = 0x338;
constexpr unsigned marker = 0x1200;
constexpr unsigned setup_instructions = 25;
constexpr unsigned full_speed_maximum = 64;
constexpr unsigned valid_descriptor = 0x5012;
constexpr unsigned descriptor_length = 18;
// Own USB device descriptor and executable HC12 instructions, never manufacturer data.
constexpr std::array<std::uint8_t, descriptor_length> descriptor{
    descriptor_length, 1, 0, 2, 0, 0, 0, 64, 0x34, 0x12, 0x78, 0x56, 0, 1, 0, 0, 0, 1};
constexpr CpuState initial{
    .pc = entry, .d = 0x1234, .x = 0x5678, .y = 0x9abc, .sp = 0x2000, .ccr = 0x40};

enum class WordOpcode : std::uint8_t {
  load_d = 0xfc,
  store_d = 0x7c,
  store_a = 0x7a,
  immediate_d = 0xcc
};

void instruction(std::vector<std::uint8_t> &code, const WordOpcode opcode, const unsigned operand) {
  code.push_back(static_cast<std::uint8_t>(opcode));
  code.push_back(static_cast<std::uint8_t>(operand >> 8U));
  code.push_back(static_cast<std::uint8_t>(operand & byte_mask));
}

std::array<std::uint8_t, 32768> firmware() {
  std::array<std::uint8_t, 32768> bytes{};
  const auto vector = [&bytes](const MemoryWrite v) {
    bytes.at(v.address - entry) = static_cast<std::uint8_t>(v.value >> 8U);
    bytes.at(v.address - entry + 1) = static_cast<std::uint8_t>(v.value & byte_mask);
  };
  vector({.address = setup_vector, .size = 2, .value = handler});
  vector({.address = input_vector, .size = 2, .value = input_handler});
  vector({.address = output_vector, .size = 2, .value = output_handler});
  constexpr unsigned load_a_immediate = 0x86;
  constexpr unsigned rti = 0x0b;
  std::vector<std::uint8_t> code;
  instruction(code, WordOpcode::load_d, setup_buffer);
  instruction(code, WordOpcode::store_d, marker);
  code.insert(code.end(), {load_a_immediate, 1});
  instruction(code, WordOpcode::store_a, status + 1);
  for (unsigned i = 0; i < descriptor.size(); i += 2) {
    instruction(code, WordOpcode::load_d, descriptor_source + i);
    instruction(code, WordOpcode::store_d, local + i);
  }
  instruction(code, WordOpcode::immediate_d, valid_descriptor);
  instruction(code, WordOpcode::store_d, endpoint_base);
  code.push_back(rti);
  std::ranges::copy(code, bytes.begin() + (handler - entry));
  std::ranges::copy(descriptor, bytes.begin() + (descriptor_source - entry));
  constexpr std::array<std::uint8_t, 6> ack_input{0x86, 0x30, 0x7a, 3, 0x14, 0x0b};
  constexpr std::array<std::uint8_t, 6> ack_output{0x86, 0x30, 0x7a, 3, 0x16, 0x0b};
  std::ranges::copy(ack_input, bytes.begin() + (input_handler - entry));
  std::ranges::copy(ack_output, bytes.begin() + (output_handler - entry));
  return bytes;
}

void interrupt(Hc12 &cpu, const InterruptRequest expected, const unsigned instructions) {
  const auto entered = cpu.step();
  require(!entered.instruction_executed && entered.interrupt_vector == expected.vector,
          "The real priority controller enters the independently expected USB IRQ vector");
  for (unsigned i = 0; i < instructions; ++i) { cpu.step(); }
  const auto s = cpu.state();
  require(s.pc == initial.pc && s.d == initial.d && s.x == initial.x && s.y == initial.y &&
              s.sp == initial.sp && s.ccr == initial.ccr,
          "Executed HC12 instructions restore all supplied CPU registers through "
          "RTI");
}

ExecutionContext current(const Hc12 &cpu) {
  return {.pc = cpu.state().pc, .ccr = cpu.state().ccr};
}

void test_native_setup_through_wire_transfer() {
  const auto flash = firmware();
  McuBus bus(flash);
  prepare(bus, full_speed_maximum);
  bus.write(mask + 1, 1);
  Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.set_state(initial);
  auto host = request(7, control_input, descriptor.size());
  host.setup = {0x80, 6, 0, 1, 0, 0, descriptor_length, 0};
  const auto frame = test::usbip_fixture::submit(host);
  UsbipDecoder decoder(test::usbip_fixture::device);
  decoder.feed(frame);
  const auto command = decoder.next();
  require(command.has_value(), "The own wire request is complete");
  if (!command) { throw Fault("Missing native USB test request"); }
  const auto *submission = std::get_if<UsbTransferRequest>(&*command);
  require(submission != nullptr, "Wire SUBMIT retains its command type");
  if (submission == nullptr) { throw Fault("Wrong native USB command"); }
  UsbHostQueue queue;
  queue.submit(*submission);
  require(queue.poll(bus, current(cpu)).empty(), "SETUP alone never completes a host transfer");
  require(queue.poll(bus, current(cpu)).empty(),
          "No response exists before the actual guest ISR runs");
  interrupt(cpu, {.vector = setup_vector}, setup_instructions);
  constexpr unsigned first_setup_word = 0x8006;
  require(word(bus, marker) == first_setup_word && (word(bus, status) & 1U) == 0,
          "Own native code reads exact setup bytes, records them and acknowledges SETUP itself");
  require(queue.poll(bus, current(cpu)).empty(),
          "Accepted descriptor data still requires control status");
  interrupt(cpu, {.vector = input_vector}, 3);
  const auto completed = queue.poll(bus, current(cpu));
  require(completed.size() == 1 &&
              completed[0].input ==
                  std::vector<std::uint8_t>(descriptor.begin(), descriptor.end()) &&
              completed[0].transferred == descriptor.size() &&
              completed[0].status == UsbTransferStatus::success,
          "Host receives only actual native ISR-produced descriptor bytes after a real status "
          "handshake");
  const auto reply = encode_usbip_completion(completed[0]);
  require(reply.size() == usbip_transfer_header + descriptor.size() &&
              std::ranges::equal(std::span(reply).subspan(usbip_transfer_header), descriptor),
          "USB/IP completion carries those same independently expected own bytes");
  interrupt(cpu, {.vector = output_vector}, 3);
  require(queue.pending() == 0 && !cpu.pending_interrupt(),
          "Separate IN/OUT guest IRQ acknowledgements release all ownership");
  decoder.finish();
}
}

int main() {
  try {
    test_native_setup_through_wire_transfer();
    std::cout << "Verified own USB/IP SUBMIT -> MCU SETUP/HC12 ISR -> descriptor/ACK/status/IRQ -> "
                 "exact RET_SUBMIT. No original Blofeld enumeration claim.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
