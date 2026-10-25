// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_usb.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace {
constexpr unsigned entry = 0x8000;
constexpr unsigned setup_handler = 0x9000;
constexpr unsigned descriptor_address = 0x9100;
constexpr unsigned completion_handler = 0x9200;
constexpr unsigned setup_vector = 0xffd2;
constexpr unsigned input_vector = 0xffce;
constexpr unsigned output_vector = 0xffcc;
constexpr unsigned output_handler = 0x9240;
constexpr unsigned status = 0x302;
constexpr unsigned interrupt_mask = 0x304;
constexpr unsigned configuration_status = 0x30c;
constexpr unsigned selector = 0x30e;
constexpr unsigned configuration_data = 0x310;
constexpr unsigned endpoint_control = 0x314;
constexpr unsigned local_buffer = 0x380;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned ldaa_immediate = 0x86;
enum class WordOpcode : std::uint8_t {
  store_a = 0x7a,
  load_d = 0xfc,
  immediate_d = 0xcc,
  store_d = 0x7c
};
constexpr unsigned rti = 0x0b;
constexpr std::array<std::uint8_t, 8> request{0x80, 6, 0, 1, 0, 0, 18, 0};
// Own descriptor, unrelated to any manufacturer product.
constexpr std::array<std::uint8_t, 18> descriptor{
    18, 1, 0, 2, 0, 0, 0, 64, 0x34, 0x12, 0x78, 0x56, 0, 1, 0, 0, 0, 1};
constexpr blofeld::CpuState initial{
    .pc = entry, .d = 0x1234, .x = 0x5678, .y = 0x9abc, .sp = 0x2000, .ccr = 0x40};

void require(const bool condition, const std::string &message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void word(std::vector<std::uint8_t> &code, const WordOpcode opcode, const unsigned value) {
  code.push_back(static_cast<std::uint8_t>(opcode));
  code.push_back(static_cast<std::uint8_t>(value >> 8U));
  code.push_back(static_cast<std::uint8_t>(value & byte_mask));
}

void vector_word(std::array<std::uint8_t, 32768> &flash, const blofeld::MemoryWrite write) {
  flash.at(write.address - entry) = static_cast<std::uint8_t>(write.value >> 8U);
  flash.at(write.address - entry + 1) = static_cast<std::uint8_t>(write.value & byte_mask);
}

std::array<std::uint8_t, 32768> firmware() {
  std::array<std::uint8_t, 32768> flash{};
  constexpr unsigned nop = 0xa7;
  flash[0] = nop;
  vector_word(flash, {.address = setup_vector, .size = 2, .value = setup_handler});
  vector_word(flash, {.address = input_vector, .size = 2, .value = completion_handler});
  vector_word(flash, {.address = output_vector, .size = 2, .value = output_handler});
  std::vector<std::uint8_t> code{ldaa_immediate, 1};
  word(code, WordOpcode::store_a, status + 1);
  for (unsigned byte = 0; byte < descriptor.size(); byte += 2) {
    word(code, WordOpcode::load_d, descriptor_address + byte);
    word(code, WordOpcode::store_d, local_buffer + byte);
  }
  constexpr unsigned valid_descriptor = 0x5012;
  word(code, WordOpcode::immediate_d, valid_descriptor);
  word(code, WordOpcode::store_d, endpoint_control);
  code.push_back(rti);
  std::ranges::copy(code, flash.begin() + (setup_handler - entry));
  std::ranges::copy(descriptor, flash.begin() + (descriptor_address - entry));
  constexpr std::array<std::uint8_t, 6> ack{0x86, 0x30, 0x7a, 3, 0x14, 0x0b};
  std::ranges::copy(ack, flash.begin() + (completion_handler - entry));
  constexpr std::array<std::uint8_t, 6> ack_out{0x86, 0x30, 0x7a, 3, 0x16, 0x0b};
  std::ranges::copy(ack_out, flash.begin() + (output_handler - entry));
  return flash;
}

void prepare(blofeld::McuBus &bus) {
  bus.set_pin_drive(blofeld::McuPort::e, {.mask = 3, .levels = 3});
  constexpr unsigned control = 0x300;
  constexpr unsigned clock = 0x40;
  constexpr unsigned enabled = 0xc0;
  constexpr unsigned endpoint_maximum_high = 0x200;
  bus.write(control, clock);
  bus.write(control + 1, 8); // Connect full-speed signaling explicitly; reset DISCON is set.
  bus.observe_phy({.edges = 15});
  bus.write(selector, 1);
  bus.write_memory({.address = configuration_data, .size = 2, .value = endpoint_maximum_high});
  bus.write_memory({.address = configuration_data + 2, .size = 2});
  bus.observe_usb_decoder({});
  bus.write(control, enabled);
  bus.observe_usb_decoder({.enumerated = true});
}

void verify_restored(const blofeld::Hc12 &cpu) {
  const auto state = cpu.state();
  require(state.pc == initial.pc && state.sp == initial.sp && state.d == initial.d &&
              state.x == initial.x && state.y == initial.y && state.ccr == initial.ccr,
          "USB ISR restores every independently supplied interrupted register");
}

void test_setup_cpu_response_and_completion_irq() {
  const auto flash = firmware();
  blofeld::McuBus bus(flash);
  prepare(bus);
  bus.write(interrupt_mask + 1, 1);
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.set_state(initial);
  require(bus.receive_usb_setup(request) == blofeld::UsbResponse::ack,
          "Decoded GET_DESCRIPTOR raises a real setup event");
  const auto accepted = cpu.step();
  require(!accepted.instruction_executed && accepted.interrupt_vector == setup_vector,
          "UF32 setup interrupt selects literal FFD2 before the main instruction executes");
  constexpr unsigned handler_instructions = 23;
  for (unsigned index = 0; index < handler_instructions; ++index) { cpu.step(); }
  verify_restored(cpu);
  require(!cpu.pending_interrupt(),
          "Executed ISR instructions acknowledge SETUP without hidden controller clearing");
  const auto response = bus.request_usb_in(blofeld::UsbBuffer::control_in);
  require(response.response == blofeld::UsbResponse::data && response.size == descriptor.size() &&
              response.data1 &&
              std::ranges::equal(std::span{response.data}.first(descriptor.size()), descriptor),
          "Host receives the literal descriptor copied from ROM to USB RAM by real HC12 "
          "instructions, using DATA1");
  require(!cpu.pending_interrupt(), "Endpoint completion stays absent before host ACK");
  bus.finish_usb_in(false);
  const auto retry = bus.request_usb_in(blofeld::UsbBuffer::control_in);
  require(retry.data == response.data && retry.data1,
          "Lost ACK repeats the same descriptor and DATA1");
  bus.finish_usb_in(true);
  const auto completion = cpu.step();
  require(!completion.instruction_executed && completion.interrupt_vector == input_vector,
          "Host ACK raises the distinct EP0 IN interrupt at literal FFCE");
  for (unsigned index = 0; index < 3; ++index) { cpu.step(); }
  verify_restored(cpu);
  require(!cpu.pending_interrupt(), "A second real ISR clears TFRC using W1C");
  constexpr unsigned out_status = 0x316;
  constexpr unsigned out_ie = 0x10;
  bus.write(out_status, out_ie);
  require(
      bus.receive_usb_out({.buffer = blofeld::UsbBuffer::control_out, .data = {}, .data1 = true}) ==
          blofeld::UsbResponse::ack,
      "The control-read status OUT accepts the host's zero-length DATA1");
  constexpr unsigned completed_out = 0x7000;
  require(bus.read_memory({.address = out_status, .size = 2}) == completed_out,
          "Control OUT status completion sets DVALID/TFRC with zero received bytes");
  const auto output = cpu.step();
  require(!output.instruction_executed && output.interrupt_vector == output_vector,
          "The status OUT packet raises the distinct FFCC completion vector");
  for (unsigned index = 0; index < 3; ++index) { cpu.step(); }
  verify_restored(cpu);
  require(!cpu.pending_interrupt(), "The OUT completion ISR clears TFRC and releases DVALID");
}

void test_priority_and_quittances_across_usb_sources() {
  auto flash = firmware();
  constexpr std::array<unsigned, 4> vectors{0xffd4, 0xffd2, 0xffd0, 0xffbe};
  constexpr std::array<unsigned, 4> addresses{0x9300, 0x9340, 0x9380, 0x93c0};
  constexpr std::array<unsigned, 4> registers{status, status + 1, configuration_status, status + 1};
  constexpr std::array<unsigned, 4> values{0x10, 3, 0x10, 4};
  for (unsigned index = 0; index < vectors.size(); ++index) {
    vector_word(flash,
                {.address = static_cast<std::uint16_t>(vectors[index]),
                 .size = 2,
                 .value = addresses[index]});
    std::vector<std::uint8_t> code{ldaa_immediate, static_cast<std::uint8_t>(values[index])};
    word(code, WordOpcode::store_a, registers[index]);
    code.push_back(rti);
    std::ranges::copy(code, flash.begin() + (addresses[index] - entry));
  }
  blofeld::McuBus bus(flash);
  prepare(bus);
  // UNCIR is protected while MEN=1; program its limit before enabling again.
  constexpr unsigned control = 0x300;
  constexpr unsigned disabled = 0x40;
  constexpr unsigned enabled = 0xc0;
  bus.write(control, disabled);
  bus.write(selector, 8);
  bus.write_memory({.address = configuration_data, .size = 2});
  constexpr unsigned configuration_limit = 0x11;
  bus.write_memory({.address = configuration_data + 2, .size = 2, .value = configuration_limit});
  bus.write(control, enabled);
  constexpr unsigned interrupt_enables = 0x3807;
  bus.write_memory({.address = interrupt_mask, .size = 2, .value = interrupt_enables});
  bus.receive_usb_setup(request);
  bus.observe_usb_sof(3);
  bus.observe_usb_configuration({.configuration = 1, .interface = std::nullopt});
  bus.observe_usb_decoder({.suspended = true, .enumerated = true});
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.set_state(initial);
  const auto ordinary = cpu.pending_interrupt();
  require(ordinary && ordinary->vector == vectors[0],
          "Default USB priority follows device vector addresses");
  constexpr unsigned hprio = 0x1f;
  constexpr unsigned promote_sof = 0xbe;
  constexpr unsigned masked_flags = 0xd0;
  bus.begin_instruction({.pc = entry, .ccr = masked_flags});
  bus.write(hprio, promote_sof);
  const auto promoted = cpu.step();
  require(!promoted.instruction_executed && promoted.interrupt_vector == vectors[3],
          "HPRIO promotes SOF above the other three pending USB sources");
  constexpr std::array<unsigned, 3> remaining{0xffd4, 0xffd2, 0xffd0};
  for (const auto expected : remaining) {
    cpu.step();
    cpu.step();
    const auto chained = cpu.step();
    require(chained.instruction_executed && chained.stack_reused &&
                chained.interrupt_vector == expected,
            "Each actual ISR acknowledges only its source; RTI reuses the frame for the next "
            "pending USB vector");
  }
  cpu.step();
  cpu.step();
  const auto returned = cpu.step();
  require(!returned.interrupt_vector && !cpu.pending_interrupt(),
          "All four USB sources deassert only after their real register acknowledgments");
  verify_restored(cpu);
}
}

int main() {
  try {
    test_setup_cpu_response_and_completion_irq();
    test_priority_and_quittances_across_usb_sources();
    std::cout << "USB events through HC12 setup/completion ISRs, priority and RTI checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
