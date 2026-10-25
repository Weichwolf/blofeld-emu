// SPDX-License-Identifier: GPL-3.0-only
#include "application_fixture.hpp"
#include "common/bus.hpp"
#include "firmware/firmware.hpp"
#include "lcd_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_midi.hpp"
#include "midi/host_midi.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <ios>
#include <iostream>
#include <memory>
#include <ostream>
#include <span>
#include <string>
#include <string_view>

namespace {
constexpr unsigned bus_hz = 30000000;
constexpr unsigned divisor = 60;
constexpr unsigned expected_control2 = 0x2c;
constexpr unsigned realtime_begin = 0xf8;
constexpr unsigned bit_cycles = 960;
constexpr unsigned frame_cycles = 9600;
constexpr unsigned sci_vector = 0xffd6;
constexpr unsigned sci_handler = 0xb5c1;
constexpr unsigned resume = 0x8315;
constexpr unsigned main_stack = 0x25f3;
constexpr unsigned mailbox = 0x19b6;
constexpr unsigned instruction_limit = 2000000;
constexpr std::array<std::uint8_t, 8> message{0xf8, 0x90, 60, 100, 0xf8, 0x80, 60, 0};

void require(const bool condition, const std::string_view text) {
  if (!condition) { throw blofeld::Fault(std::string(text)); }
}

void receive(blofeld::McuBus &bus,
             blofeld::Hc12 &cpu,
             blofeld::McuMidiInput &wire,
             const std::uint8_t byte,
             std::ostream &trace) {
  const auto before = cpu.state();
  require(cpu.waiting() && !cpu.pending_interrupt(), "Original main WAI precedes every host byte");
  wire.enqueue(std::span{&byte, 1});
  wire.advance_sci(bus, {.pc = before.pc, .ccr = before.ccr}, frame_cycles);
  const auto pending = cpu.pending_interrupt();
  require(pending && pending->vector == sci_vector,
          "Actual wire reception requests original SCI IRQ");
  const auto entry = cpu.step();
  require(entry.interrupt_vector == sci_vector && entry.stack_reused &&
              !entry.instruction_executed && cpu.state().pc == sci_handler,
          "The native vector reuses the original WAI frame");
  unsigned instructions = 0;
  do {
    require(instructions < instruction_limit, "Original SCI handler must return within its bound");
    const auto step = cpu.step();
    require(step.instruction_executed && !step.interrupt_vector,
            "The original handler returns through actual instructions and RTI");
    ++instructions;
  } while (cpu.state().pc != resume);
  const auto after = cpu.state();
  require(!cpu.waiting() && !cpu.pending_interrupt() && bus.read(mailbox) == byte &&
              after.sp == main_stack && after.d == before.d && after.x == before.x &&
              after.y == before.y && after.ccr == before.ccr &&
              instructions == (byte >= realtime_begin ? 12 : 16),
          "Independent ISR endpoints retain the host byte, complete CPU context and exact path");
  for (unsigned count = 0; !cpu.waiting() || cpu.pending_interrupt(); ++count) {
    require(count < instruction_limit, "Original foreground must return to main WAI");
    const auto step = cpu.step();
    require(step.instruction_executed && !step.interrupt_vector,
            "Only actual original foreground instructions return to main WAI");
  }
  require(cpu.state().pc == resume, "Actual original foreground reaches its main WAI again");
  trace << "HOST-MIDI byte=" << blofeld::hex(byte, 2) << " isr-instructions=" << instructions
        << " pc=" << blofeld::hex(cpu.state().pc) << '\n';
}

struct Inputs {
  std::filesystem::path firmware;
  std::filesystem::path source;
  std::filesystem::path trace;
};

void run(const Inputs &inputs) {
  const auto firmware = blofeld::Firmware::load(inputs.firmware);
  blofeld::McuMachine machine(blofeld::McuFlashImage::from_known(firmware.mcu),
                              BLOFELD_DEFAULT_SLA);
  auto display = std::make_unique<blofeld::test::LcdFixture>(machine.bus());
  std::ofstream trace(inputs.trace);
  trace.exceptions(std::ios::failbit | std::ios::badbit);
  static_cast<void>(blofeld::test::start_application(machine, display, trace, false));
  auto &bus = machine.bus();
  auto &cpu = machine.cpu();
  require(bus.read(blofeld::hc12::sci_baud_low) == divisor &&
              bus.read(blofeld::hc12::sci_control1) == 8 &&
              bus.read(blofeld::hc12::sci_control2) == expected_control2,
          "Unmodified startup supplies the native SCI setup; host code does not repair it");
  const auto initial = bus.power_off_flash_image();
  blofeld::HostMidiInput source(inputs.source, blofeld::MidiInputKind::bytes);
  blofeld::McuMidiInput wire(bus_hz, blofeld::SciReceivePin::pr2);
  const auto before = cpu.state();
  wire.advance_sci(bus, {.pc = before.pc, .ccr = before.ccr}, bit_cycles);
  unsigned cursor = 0;
  std::array<std::uint8_t, 2> bytes{};
  while (!source.exhausted()) {
    const auto count = source.read(bytes);
    for (const auto byte : std::span{bytes}.first(count)) {
      require(cursor < message.size() && byte == message.at(cursor), "Pinned own MIDI file order");
      receive(bus, cpu, wire, byte, trace);
      ++cursor;
    }
  }
  const auto final = bus.power_off_flash_image();
  require(cursor == message.size() && final.bytes == initial.bytes &&
              final.known == initial.known && final.fresh == initial.fresh,
          "Every file byte reaches original handlers without modifying any NVM state");
  std::cout << "Eight host-file bytes reached actual original SCI/ISR/RTI/main-WAI from the "
               "conditional untimed cold-start fixture. No live device, scheduled board, complete "
               "message processing, DSP/audio, physical timing or fuzzing claim.\n";
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 4) {
      throw blofeld::Fault("Usage: original-host-midi original.zip bytes.bin trace");
    }
    run({.firmware = arguments[1], .source = arguments[2], .trace = arguments[3]});
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
