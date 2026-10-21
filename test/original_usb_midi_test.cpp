// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/mcu_usb.hpp"
#include "usb_host_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::usb_host;
constexpr unsigned ring = 0x18f2;
constexpr unsigned reader = 0x1972;
constexpr unsigned writer = 0x1973;
constexpr unsigned ring_mask = 0x7c;
constexpr unsigned event_bytes = 4;
constexpr unsigned endpoint_bytes = 64;
constexpr unsigned events_per_packet = endpoint_bytes / event_bytes;
constexpr unsigned ram_begin = 0x1200;
constexpr unsigned ram_bytes = 5120;
constexpr unsigned vector = 0xffca;
constexpr unsigned handler = 0xbe6b;
constexpr unsigned source_pc = 0x8000;
constexpr unsigned source_stack = 0x2000;
constexpr unsigned stack_low = source_stack - 16; // IRQ9 + JSR2 + local3 + temporary2.
constexpr unsigned instruction_limit = 1000;
constexpr unsigned canary = 0xa5;
constexpr unsigned x_mask = 0x40;
constexpr unsigned high_byte_shift = 8;
constexpr unsigned clear_other_flags = 0xbf;
constexpr unsigned ready_without_completion = 0x5000;
constexpr unsigned idle_control = 0x1000;
constexpr unsigned acknowledged_word = 0x3000;
constexpr unsigned first_note = 48;
constexpr unsigned velocity = 100;
constexpr unsigned note_on = 0x90;
constexpr unsigned midi_buffers = 0x32;
constexpr unsigned qram_mapping = 0x21;
constexpr CpuState initial{
    .pc = source_pc, .d = 0x1234, .x = 0x5678, .y = 0x9abc, .sp = source_stack};
using Bytes = std::vector<std::uint8_t>;
using Writes = std::vector<std::pair<unsigned, unsigned>>;
using Registers = std::array<unsigned, 6>;

struct Case {
  unsigned events = 0;
  unsigned cursor = 0;
  unsigned ccr = x_mask;
};

Registers registers(const CpuState state) {
  return {state.pc, state.d, state.x, state.y, state.sp, state.ccr};
}

Bytes payload(const unsigned events) {
  Bytes bytes;
  for (unsigned i = 0; i < events; ++i) {
    bytes.insert(bytes.end(), {9, note_on, static_cast<std::uint8_t>(first_note + i), velocity});
  }
  return bytes;
}

void prepare(McuMachine &machine, const Case input) {
  auto &bus = machine.bus();
  // Conditional own peripheral/RAM inputs, not the original initializer or boot.
  bus.write(hc12::initee, qram_mapping);
  bus.retire_instruction();
  blofeld::test::usb_host::prepare(bus, endpoint_bytes);
  for (unsigned i = 0; i < ram_bytes; ++i) {
    bus.write(static_cast<std::uint16_t>(ram_begin + i), canary);
  }
  const auto bytes = payload(input.events);
  for (unsigned i = 0; i < bytes.size(); ++i) {
    const auto offset = (input.cursor + i) % (2 * endpoint_bytes);
    bus.write(static_cast<std::uint16_t>(ring + offset), bytes[i]);
  }
  bus.write(reader, static_cast<std::uint8_t>(input.cursor));
  bus.write(writer, static_cast<std::uint8_t>((input.cursor + bytes.size()) & ring_mask));
  bus.write(buffer_select, 2);
  for (unsigned i = 0; i < endpoint_bytes; ++i) {
    bus.write(static_cast<std::uint16_t>(local + i), canary);
  }
  bus.write_memory({.address = input_control, .size = 2, .value = ready_without_completion});
  auto state = initial;
  state.ccr = static_cast<std::uint8_t>(input.ccr);
  machine.cpu().set_state(state);
  const auto trigger = bus.request_usb_in(UsbBuffer::endpoint2);
  require(trigger.response == UsbResponse::data && trigger.size == 0,
          "An own zero-length prior IN is a real transaction, not a fabricated completion flag");
  bus.finish_usb_in(true);
}

Writes expected_writes(const std::span<const std::uint8_t> bytes) {
  Writes writes{{input_control, acknowledged_word >> high_byte_shift}, {input_control + 1, 0}};
  if (!bytes.empty()) { writes.emplace_back(buffer_select, midi_buffers); }
  for (unsigned offset = 0; offset < bytes.size(); offset += event_bytes) {
    for (const auto byte : {2U, 3U, 0U, 1U}) {
      writes.emplace_back(local + offset + byte, bytes[offset + byte]);
    }
  }
  if (!bytes.empty()) {
    writes.emplace_back(input_control, ready >> high_byte_shift);
    writes.emplace_back(input_control + 1, bytes.size());
  }
  return writes;
}

unsigned service(McuMachine &machine, const std::span<const std::uint8_t> bytes, Writes &all) {
  auto &bus = machine.bus();
  auto &cpu = machine.cpu();
  auto expected = cpu.state();
  expected.ccr = static_cast<std::uint8_t>(expected.ccr & clear_other_flags);
  const auto cursor = bus.read(reader);
  const auto pending = cpu.pending_interrupt();
  require(pending && pending->vector == vector,
          "Actual endpoint2 ACK raises the original MIDI IN vector");
  Writes writes;
  bus.set_observer([&writes, &bus, cursor](const BusEvent &event) {
    // This observer reports peripheral writes, not RAM stores. Check the RAM
    // cursor when each payload write becomes observable instead.
    if (event.write) {
      if (event.address >= local && event.address < local + endpoint_bytes) {
        const auto slot = (event.address - local) / event_bytes;
        require(bus.read(reader) == ((cursor + (slot + 1) * event_bytes) & ring_mask),
                "The native read cursor advances before each event's first payload write");
      }
      writes.emplace_back(event.address, event.value);
    }
  });
  const auto entered = cpu.step();
  require(!entered.instruction_executed && entered.interrupt_vector == vector &&
              !entered.stack_reused && cpu.state().pc == handler,
          "The actual original vector enters its ROM ISR through native hardware stacking");
  unsigned retired = 0;
  while (cpu.state().pc != source_pc) {
    require(retired < instruction_limit, "Original MIDI IN handler and real flush must return");
    require(cpu.state().sp >= stack_low && cpu.state().sp <= source_stack - 9,
            "Every actual ISR/callee stack position stays within its independently bounded frame");
    const auto step = cpu.step();
    require(step.instruction_executed && !step.interrupt_vector,
            "Original ISR/callee/RTI run without synthetic returns or interrupt injection");
    ++retired;
  }
  bus.set_observer({});
  if (cpu.state().ccr != expected.ccr) {
    std::cout << "IRQ_CCR actual=" << hex(cpu.state().ccr, 2)
              << " expected=" << hex(expected.ccr, 2) << '\n';
  }
  require(registers(cpu.state()) == registers(expected) && !cpu.pending_interrupt(),
          "Original flush clears X; native RTI preserves that clear and restores every other "
          "bit/register");
  const auto status = idle_control | (bytes.empty() ? 0U : ready_without_completion | bytes.size());
  if (word(bus, input_control) != status || writes != expected_writes(bytes)) {
    std::cout << "USB_STATUS actual=" << hex(word(bus, input_control))
              << " expected=" << hex(status) << " reader=" << hex(bus.read(reader), 2) << '\n';
    for (const auto &[address, value] : writes) {
      std::cout << "ACTUAL_WRITE " << hex(address) << ' ' << hex(value, 2) << '\n';
    }
    for (const auto &[address, value] : expected_writes(bytes)) {
      std::cout << "EXPECTED_WRITE " << hex(address) << ' ' << hex(value, 2) << '\n';
    }
  }
  require(word(bus, input_control) == status && writes == expected_writes(bytes) &&
              bus.read(reader) == ((cursor + bytes.size()) & ring_mask),
          "Original cursor-before-data, second-word-first copy, whole status write and ACK match");
  all.insert(all.end(), writes.begin(), writes.end());
  return retired;
}

struct Outcome {
  Registers cpu;
  Bytes ram;
  std::array<std::uint8_t, endpoint_bytes> buffer{};
  Writes writes;
  unsigned retired = 0;
  unsigned irqs = 0;
  bool operator==(const Outcome &) const = default;
};

Outcome drain(McuMachine &machine, const Case input) {
  const auto expected = payload(input.events);
  Outcome result;
  result.buffer.fill(canary);
  for (unsigned offset = 0;;) {
    const auto count = std::min<std::size_t>(endpoint_bytes, expected.size() - offset);
    const auto chunk = std::span(expected).subspan(offset, count);
    result.retired += service(machine, chunk, result.writes);
    ++result.irqs;
    if (count == 0) { break; }
    std::ranges::copy(chunk, result.buffer.begin());
    const auto packet = machine.bus().request_usb_in(UsbBuffer::endpoint2);
    require(packet.response == UsbResponse::data && packet.size == count &&
                packet.data1 == (result.irqs % 2 != 0) &&
                std::ranges::equal(std::span(packet.data).first(count), chunk),
            "Only bytes copied by the original MCU are delivered by actual IN transactions");
    machine.bus().finish_usb_in(true);
    offset += static_cast<unsigned>(count);
  }
  const auto final = machine.bus().request_usb_in(UsbBuffer::endpoint2);
  require(final.response == UsbResponse::nak &&
              machine.bus().read(reader) == machine.bus().read(writer) &&
              result.irqs == (input.events + events_per_packet - 1) / events_per_packet + 1,
          "After the final real ACK, the original IRQ drains the ring and leaves actual NAK");
  auto copy = machine.bus().clone();
  copy->write(buffer_select, 2);
  for (unsigned i = 0; i < endpoint_bytes; ++i) {
    require(copy->read(static_cast<std::uint16_t>(local + i)) == result.buffer[i],
            "Bytes beyond the most recent native packet preserve the earlier buffer contents");
  }
  for (unsigned i = 0; i < ram_bytes; ++i) {
    result.ram.push_back(machine.bus().read(static_cast<std::uint16_t>(ram_begin + i)));
  }
  result.cpu = registers(machine.cpu().state());
  return result;
}

void run_case(const Firmware &firmware, const Case input) {
  McuMachine machine(McuFlashImage::from_known(firmware.mcu), BLOFELD_DEFAULT_SLA);
  prepare(machine, input);
  const auto flash = machine.bus().power_off_flash_image();
  Bytes before_ram;
  for (unsigned i = 0; i < ram_bytes; ++i) {
    before_ram.push_back(machine.bus().read(static_cast<std::uint16_t>(ram_begin + i)));
  }
  const auto checkpoint = machine.checkpoint();
  std::optional<Outcome> expected;
  for (unsigned replay = 0; replay < 3; ++replay) {
    if (replay != 0) { machine.restore(checkpoint); }
    const auto actual = drain(machine, input);
    for (unsigned i = 0; i < ram_bytes; ++i) {
      const auto address = ram_begin + i;
      if (address != reader && (address < stack_low || address >= source_stack)) {
        require(actual.ram[i] == before_ram[i],
                "Only the read cursor and independently bounded IRQ/callee stack may change RAM");
      }
    }
    if (!expected) { expected = actual; }
    const auto now = machine.bus().power_off_flash_image();
    require(
        actual == *expected && now.bytes == flash.bytes && now.known == flash.known &&
            now.fresh == flash.fresh,
        "Two pending-IRQ restores reproduce CPU/all RAM/USB bytes/write order and preserve NVM");
    std::cout << "CASE " << input.events << ' ' << input.cursor << ' ' << input.ccr << ' ' << replay
              << ' ' << actual.irqs << ' ' << actual.retired << '\n';
  }
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 2, "Usage: original-usb-midi original.zip");
    const auto firmware = Firmware::load(arguments[1]);
    for (const auto events : {0U, 1U, 15U, 16U, 17U, 31U}) {
      for (const auto cursor : {0U, ring_mask}) {
        for (const auto flags : {x_mask, 0xc5U, 0U}) {
          run_case(firmware, {.events = events, .cursor = cursor, .ccr = flags});
        }
      }
    }
    std::cout << "SUMMARY cases=36 replays=72 own-peripheral-configuration=1 original-boot=0\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
