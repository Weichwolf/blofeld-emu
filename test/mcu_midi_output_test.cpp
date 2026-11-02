// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_midi_output.hpp"
#include "midi/midi_serial_receiver.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string_view>

namespace {
using namespace blofeld;
namespace reg = hc12;
constexpr std::array<std::uint8_t, 32768> flash{};
constexpr ExecutionContext context{.pc = 0x8123, .ccr = 0x90};
constexpr std::uint64_t test_hz = 1000000;
constexpr std::uint64_t start = 32;
constexpr std::uint64_t stop = 320;
constexpr std::uint64_t completed = 336;
constexpr std::uint64_t spacing = 352;
constexpr std::uint8_t pr3_mask = 8;

void require(const bool condition, const char *reason) {
  if (!condition) { throw Fault(reason); }
}

void rejects(const auto &operation, const std::string_view expected) {
  bool rejected = false;
  try {
    operation();
  } catch (const Fault &error) {
    const std::string_view text(error.what());
    rejected = text.contains(expected) && text.contains("operation=observe") &&
               text.contains("address=0x0268") && text.contains("pc=0x8123") &&
               text.contains("ccr=0x90");
  }
  require(rejected, "Output refusal must retain the actual pad and MCU execution context");
}

void drive(McuBus &bus, McuMidiOutput &output, const std::uint64_t ticks, const bool high) {
  bus.set_pin_drive(McuPort::r, {.mask = pr3_mask, .levels = high ? pr3_mask : std::uint8_t{0}});
  output.observe(bus, ticks, context);
}

void zero(McuBus &bus, McuMidiOutput &output, const std::uint64_t at) {
  drive(bus, output, at, false);
  drive(bus, output, at + (stop - start), true);
  output.observe(bus, at + (completed - start), context);
}

void queue_copy_and_wrap() {
  McuBus bus(flash);
  McuMidiOutput output(SciTransmitPin::pr3, test_hz);
  require(!output.pop(), "Empty output cannot manufacture a MIDI byte");
  drive(bus, output, 0, true);
  drive(bus, output, start, false);
  auto copied = output;
  drive(bus, output, stop, true);
  copied.observe(bus, stop, context);
  output.observe(bus, completed, context);
  copied.observe(bus, completed, context);
  const MidiSerialByte expected{.value = 0, .start_ticks = start, .stop_sample_ticks = completed};
  require(output.pop() == expected && copied.pop() == expected && !copied.pop(),
          "A fresh value copy retains the independently expected partial00 frame");
  for (std::size_t i = 1; i <= McuMidiOutput::capacity * 2; ++i) {
    const auto at = start + (i * spacing);
    zero(bus, output, at);
    require(output.pop() == MidiSerialByte{.value = 0,
                                           .start_ticks = at,
                                           .stop_sample_ticks = at + (completed - start)},
            "More than two complete ring wraps preserve each byte and native timestamp");
  }
  constexpr unsigned expected_transmitted = 513;
  require(output.queued_bytes() == 0 && output.transmitted_bytes() == expected_transmitted,
          "Draining retires queue entries without changing transmitted-byte accounting");
}

void overflow_and_unknown() {
  McuBus bus(flash);
  McuMidiOutput output(SciTransmitPin::pr3, test_hz);
  output.observe(bus, 0, context);
  drive(bus, output, 1, true);
  for (std::size_t i = 0; i < McuMidiOutput::capacity; ++i) {
    zero(bus, output, start + (i * spacing));
  }
  const auto healthy = output;
  require(output.queued_bytes() == McuMidiOutput::capacity, "Bounded queue admits its last slot");
  rejects([&] { zero(bus, output, start + (McuMidiOutput::capacity * spacing)); }, "overflow");
  require(output.faulted() && output.transmitted_bytes() == McuMidiOutput::capacity + 1,
          "Overflow remains visible after the actual wire frame completed");
  output = healthy;
  const auto restored_byte = output.pop();
  require(!output.faulted() && restored_byte && restored_byte->value == 0,
          "Only an explicit healthy value restore recovers decoder and queue together");
  McuMidiOutput unknown(SciTransmitPin::pr3, test_hz);
  drive(bus, unknown, 0, true);
  drive(bus, unknown, start, false);
  bus.set_pin_drive(McuPort::r, {});
  rejects([&] { unknown.observe(bus, start + 1, context); }, "unknown active");
  require(unknown.faulted() && unknown.transmitted_bytes() == 0,
          "An unresolved active output cannot supply fabricated data or a stop bit");
}

void native_pad(const std::uint8_t routing, const SciTransmitPin pin) {
  constexpr unsigned bus_hz = 30000000;
  constexpr unsigned divisor = 60;
  constexpr unsigned frame_cycles = 19200;
  constexpr std::uint64_t data_start = 9600;
  constexpr std::uint64_t stop_center = 18720;
  constexpr std::uint8_t pattern = 0x55;
  McuBus bus(flash);
  bus.begin_instruction(context);
  bus.write(reg::modrr, routing);
  bus.write(reg::sci_baud_high, 0);
  bus.write(reg::sci_baud_low, divisor);
  bus.write(reg::sci_control2, 8);
  static_cast<void>(bus.read(reg::sci_status1));
  bus.write(reg::sci_data_low, pattern);
  McuMidiOutput output(pin, bus_hz);
  output.observe(bus, 0, context);
  for (unsigned cycle = 1; cycle <= frame_cycles; ++cycle) {
    bus.advance_sci(1);
    output.observe(bus, cycle, context);
  }
  require(output.pop() == MidiSerialByte{.value = pattern,
                                         .start_ticks = data_start,
                                         .stop_sample_ticks = stop_center} &&
              !output.pop() && output.transmitted_bytes() == 1,
          "Actual native SCI and resolved PR3/PS5 pads preserve55 and independent9600/18720 times");
}
}

int main() {
  try {
    queue_copy_and_wrap();
    overflow_and_unknown();
    native_pad(0, SciTransmitPin::pr3);
    native_pad(1, SciTransmitPin::ps5);
    std::cout << "MIDI output: literal partial copies, ring wrap, overflow/unknown context and "
                 "native SCI through both resolved pads passed; host output pending.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
