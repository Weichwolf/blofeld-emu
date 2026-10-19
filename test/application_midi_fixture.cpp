// SPDX-License-Identifier: GPL-3.0-only
#include "application_midi_fixture.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"

#include <array>
#include <cstdint>
#include <ostream>

namespace blofeld::test {
namespace {
constexpr unsigned baud_divisor = 60;
constexpr unsigned samples_per_bit = 16;
constexpr unsigned bit_cycles = baud_divisor * samples_per_bit;
constexpr unsigned midi_rx = 4;
constexpr unsigned sci_vector = 0xffd6;
constexpr unsigned sci_handler = 0xb5c1;
constexpr unsigned application_resume = 0x8315;
constexpr unsigned main_stack = 0x25f3;
constexpr unsigned mailbox = 0x19b6;
constexpr unsigned activity_flags = 0x1812;
constexpr unsigned activity_bit = 4;
constexpr unsigned countdown = 0x1830;
constexpr unsigned activity_timeout = 20;
constexpr unsigned realtime_begin = 0xf8;
constexpr unsigned isr_limit = 1000;
constexpr unsigned foreground_limit = 2000000;
constexpr unsigned expected_control1 = 8;
constexpr unsigned expected_control2 = 0x2c;
constexpr unsigned data_byte_count = 256;

void require(const bool condition, const char *message) {
  if (!condition) { throw Fault(message); }
}

struct Activity {
  unsigned flags = 0;
  std::uint64_t timeout = 0;
};

Activity activity(McuBus &bus) {
  return {.flags = bus.read(activity_flags),
          .timeout = bus.read_memory({.address = countdown, .size = 2})};
}

void line(McuBus &bus, const bool high) {
  bus.set_pin_drive(McuPort::r,
                    {.mask = midi_rx, .levels = static_cast<std::uint8_t>(high ? midi_rx : 0)});
  bus.advance_sci(bit_cycles);
}

void wire_byte(McuBus &bus, const unsigned value) {
  line(bus, false);
  for (unsigned bit = 0; bit < 8; ++bit) { line(bus, ((value >> bit) & 1U) != 0); }
  line(bus, true);
}

void receive_interrupt(
    McuBus &bus, Hc12 &cpu, const unsigned value, const Activity previous, std::ostream &trace) {
  const auto before = cpu.state();
  const auto pending = cpu.pending_interrupt();
  require(cpu.waiting() && pending && pending->vector == sci_vector,
          "Actual SCI reception requests the native vector from original main WAI");
  const auto revision = bus.revision();
  const auto entered = cpu.step();
  require(!entered.instruction_executed && entered.interrupt_vector == sci_vector &&
              entered.stack_reused && cpu.state().pc == sci_handler &&
              cpu.state().sp == before.sp && bus.revision() == revision,
          "Original SCI entry reuses the WAI frame without additional stack or bus writes");
  unsigned count = 0;
  while (cpu.state().pc != application_resume) {
    require(count < isr_limit, "Original SCI ISR must return through bounded native RTI");
    trace << "sci-isr pc=" << hex(cpu.state().pc) << ' ' << cpu.disassemble() << '\n';
    const auto result = cpu.step();
    require(result.instruction_executed && !result.interrupt_vector,
            "Original SCI ISR executes and returns without synthetic entries or tail chains");
    ++count;
  }
  const auto after = cpu.state();
  require(!cpu.waiting() && !cpu.pending_interrupt() && after.sp == main_stack &&
              after.d == before.d && after.x == before.x && after.y == before.y &&
              after.ccr == before.ccr && bus.read(mailbox) == value,
          "Original ISR acknowledges received data, stores its exact byte and restores context");
  const bool realtime = value >= realtime_begin;
  const auto expected_flags = realtime ? previous.flags : (previous.flags | activity_bit);
  const auto expected_timeout = realtime ? previous.timeout : activity_timeout;
  const auto current = activity(bus);
  constexpr unsigned realtime_instructions = 12;
  constexpr unsigned ordinary_instructions = 16;
  require(current.flags == expected_flags && current.timeout == expected_timeout &&
              count == (realtime ? realtime_instructions : ordinary_instructions),
          "Independent original ISR paths distinguish realtime bytes from activity updates");
  trace << "sci-return value=" << hex(value, 2) << " instructions=" << count
        << " flags=" << hex(current.flags, 2) << " countdown=" << current.timeout << '\n';
}

void drain_foreground(Hc12 &cpu, std::ostream &trace) {
  unsigned count = 0;
  while (!cpu.waiting() || cpu.pending_interrupt()) {
    require(count < foreground_limit, "Original foreground must return to bounded main WAI");
    const auto result = cpu.step();
    require(result.instruction_executed && !result.interrupt_vector,
            "Foreground resumes through original instructions without further supplied clocks");
    ++count;
  }
  require(cpu.state().pc == application_resume,
          "Original foreground returns to the same main WAI after SCI reception");
  trace << "foreground-wai instructions=" << count << '\n';
}

void receive_byte(McuBus &bus, Hc12 &cpu, const unsigned value, std::ostream &trace) {
  const auto previous = activity(bus);
  wire_byte(bus, value);
  receive_interrupt(bus, cpu, value, previous, trace);
  drain_foreground(cpu, trace);
}
}

void verify_application_midi(McuBus &bus, Hc12 &cpu, std::ostream &trace) {
  require(cpu.waiting() && !cpu.pending_interrupt() && bus.read(hc12::sci_baud_high) == 0 &&
              bus.read(hc12::sci_baud_low) == baud_divisor &&
              bus.read(hc12::sci_control1) == expected_control1 &&
              bus.read(hc12::sci_control2) == expected_control2 && bus.read(hc12::modrr) == 0,
          "Native SCI setup qualifies the NRZ waveform and normal 100-pin PR2 input route");
  const auto initial = bus.power_off_flash_image();
  const auto idle = activity(bus);
  require((idle.flags & activity_bit) == 0 && idle.timeout == 0,
          "Cold application starts without MIDI receive activity");
  line(bus, true);
  constexpr std::array<unsigned, 8> midi{0xf8, 0x90, 60, 100, 0xf8, 0x80, 60, 0};
  for (const auto value : midi) { receive_byte(bus, cpu, value, trace); }
  for (unsigned value = 0; value < data_byte_count; ++value) {
    receive_byte(bus, cpu, value, trace);
  }
  const auto final = bus.power_off_flash_image();
  require(final.bytes == initial.bytes && final.known == initial.known &&
              final.fresh == initial.fresh,
          "Original SCI handlers and main wakeups preserve MCU NVM, knownness and history");
  constexpr unsigned received_bytes = midi.size() + data_byte_count;
  trace << "native-sci-received bytes=" << received_bytes
        << " all-byte-values=256 source=PR2-waveform host-mcu-flag-writes=0\n";
}
}
