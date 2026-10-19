// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_clocked_midi.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_midi.hpp"
#include "blofeld/mcu_run_clock.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <string>

namespace {
namespace reg = blofeld::hc12;
constexpr unsigned oscillator_hz = 12000000;
constexpr unsigned bus_hz = 30000000;
constexpr unsigned bit_cycles = 960;
constexpr unsigned frame_cycles = 9600;
constexpr unsigned divisor = 60;
constexpr unsigned unmasked = 0xc0;
constexpr unsigned received = 0xe0;
constexpr unsigned timer_counter = 0x44;
constexpr unsigned counter_mask = 0xffff;
constexpr blofeld::ExecutionContext context{.pc = 0x8123, .ccr = unmasked};

void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const blofeld::Fault &error) { return error.what(); }
  throw blofeld::Fault("Expected clocked MIDI failure");
}

struct Fixture {
  static std::unique_ptr<blofeld::McuBus> make_bus() {
    const std::array<std::uint8_t, 32768> flash{};
    auto bus = std::make_unique<blofeld::McuBus>(flash);
    bus->set_pin_drive(blofeld::McuPort::e, {.mask = 3, .levels = 3});
    bus->write(reg::usb_control, 0x40);
    bus->observe_phy({.edges = 15});
    bus->write(reg::clock_select, 0x80);
    bus->write(reg::sci_baud_low, divisor);
    bus->write(reg::tscr1, 0x80);
    return bus;
  }

  std::unique_ptr<blofeld::McuBus> bus = make_bus();
  blofeld::McuRunClock clock{
      {.oscillator_hz = oscillator_hz,
       .switch_timing = blofeld::McuRunClockSwitch::reference_cycle_boundary}};
  blofeld::McuClockedMidiInput input{bus_hz, blofeld::SciReceivePin::pr2, clock};

  void advance(const unsigned cycles) {
    for (unsigned cycle = 0; cycle < cycles; ++cycle) {
      input.observe(*bus, clock, context);
      clock.advance_run_cycle(*bus, context);
    }
    input.synchronize(clock, context);
  }
};

void test_all_bytes_routes_and_single_clock_owner() {
  for (const bool small : {false, true}) {
    Fixture fixture;
    auto &bus = *fixture.bus;
    const auto port = small ? blofeld::McuPort::s : blofeld::McuPort::r;
    const auto mask = static_cast<std::uint8_t>(small ? 16 : 4);
    const auto pin = small ? blofeld::SciReceivePin::ps4 : blofeld::SciReceivePin::pr2;
    fixture.input = blofeld::McuClockedMidiInput(bus_hz, pin, fixture.clock);
    bus.set_pin_drive(port, {.mask = 1, .levels = 1});
    bus.write(reg::modrr, small ? 1 : 0);
    bus.write(reg::sci_control2, 4); // No live IRQ poll in this complete-frame comparison.
    fixture.advance(bit_cycles);
    constexpr unsigned byte_count = 256;
    for (unsigned value = 0; value < byte_count; ++value) {
      const std::array<std::uint8_t, 1> byte{static_cast<std::uint8_t>(value)};
      fixture.input.enqueue(byte);
      for (unsigned bit = 0; bit < 10; ++bit) {
        fixture.input.observe(bus, fixture.clock, context);
        const bool expected = bit == 9 || (bit != 0 && ((value >> (bit - 1)) & 1U) != 0);
        require(bus.observe_pads(port, mask).levels == (expected ? mask : 0),
                "Independent start, LSB-first data and stop levels match every literal input byte");
        fixture.advance(bit_cycles); // The repeated initial observation consumes zero time.
      }
      require(
          bus.read(reg::sci_status1) == received && bus.read(reg::sci_data_low) == value &&
              fixture.input.queued_bytes() == 0 && bus.observe_pads(port, 1).levels == 1,
          "Actual native SCI reception preserves the unrelated pad and drains exactly one byte");
    }
    const auto expected_cycles = bit_cycles + (byte_count * frame_cycles);
    require(fixture.clock.bus_cycles() == expected_cycles &&
                fixture.clock.elapsed_ticks() == std::uint64_t{2} * expected_cycles &&
                bus.read_memory({.address = timer_counter, .size = 2}) ==
                    (expected_cycles & counter_mask),
            "Only the owner advances IPBus/OSC/TIM; the transport never double-clocks SCI");
  }
}

void test_partial_peer_replay_and_unused_observation() {
  Fixture fixture;
  fixture.bus->write(reg::sci_control2, 4);
  fixture.advance(bit_cycles);
  // An observed period that the owner did not run can be cancelled at return.
  fixture.input.observe(*fixture.bus, fixture.clock, context);
  const std::array<std::uint8_t, 2> bytes{0x90, 60};
  static_cast<void>(rejects([&] { fixture.input.enqueue(bytes); }));
  fixture.input.synchronize(fixture.clock, context);
  fixture.input.enqueue(bytes);
  constexpr unsigned partial = (4 * bit_cycles) + 17;
  fixture.advance(partial);
  const auto frozen_bus = fixture.bus->clone();
  const auto frozen_clock = fixture.clock;
  const auto frozen_input = fixture.input;
  for (unsigned replay = 0; replay < 2; ++replay) {
    fixture.bus = frozen_bus->clone();
    fixture.clock = frozen_clock;
    fixture.input = frozen_input;
    fixture.advance(frame_cycles - partial);
    require(fixture.bus->read(reg::sci_status1) == received &&
                fixture.bus->read(reg::sci_data_low) == bytes.front() &&
                fixture.input.queued_bytes() == 1,
            "Coordinated bus/clock/wire restore retains an exact partial first frame");
    fixture.advance(frame_cycles);
    require(fixture.bus->read(reg::sci_status1) == received &&
                fixture.bus->read(reg::sci_data_low) == bytes.back() &&
                fixture.input.queued_bytes() == 0,
            "Restored queue and rational wire phase deliver the back-to-back second frame");
  }
}

void test_rate_observation_and_restore_failures() {
  for (unsigned kind = 0; kind < 4; ++kind) {
    Fixture fixture;
    const auto frozen_clock = fixture.clock;
    const auto frozen_input = fixture.input;
    fixture.advance(1);
    if (kind == 0) {
      fixture.clock.advance_run_cycle(*fixture.bus, context); // Missing observation.
    } else if (kind == 1) {
      fixture.clock = frozen_clock; // MCU-only restore.
    } else if (kind == 2) {
      fixture.advance(1);
      fixture.input = frozen_input; // Wire-only restore.
    } else {
      fixture.bus->write(reg::clock_select, 0); // Native source changes to 6 MHz IPBus.
    }
    const auto before = fixture.clock.bus_cycles();
    const auto error =
        rejects([&] { fixture.input.observe(*fixture.bus, fixture.clock, context); });
    require(
        fixture.input.faulted() && fixture.clock.bus_cycles() == before &&
            error.contains("address=0x0268") && error.contains("pc=0x8123") &&
            error.contains("ccr=0xc0"),
        "Missing periods, unilateral restore and unsupported rate changes fail with pad/context");
    const std::array<std::uint8_t, 1> byte{0x90};
    static_cast<void>(rejects([&] { fixture.input.enqueue(byte); }));
  }
}

void test_unknown_receiver_and_native_rate_switch_boundary() {
  Fixture fixture;
  fixture.bus->write(reg::modrr, 1); // Active SCI on PS4; explicit source on PR2.
  fixture.bus->write(reg::sci_control2, 4);
  const auto frozen_bus = fixture.bus->clone();
  const auto frozen_clock = fixture.clock;
  const auto frozen_input = fixture.input;
  static_cast<void>(rejects([&] { fixture.advance(divisor); }));
  require(fixture.clock.faulted(), "A disconnected active receiver faults the actual MCU clock");
  static_cast<void>(rejects([&] { fixture.input.synchronize(fixture.clock, context); }));
  require(fixture.input.faulted(), "The wire peer cannot continue after a failed MCU clock");
  fixture.bus = frozen_bus->clone();
  fixture.clock = frozen_clock;
  fixture.input = frozen_input;
  fixture.bus->write(reg::modrr, 0);
  fixture.input.observe(*fixture.bus, fixture.clock, context);
  fixture.clock.advance_run_cycle(*fixture.bus, context);
  fixture.bus->write(reg::clock_select, 0);
  fixture.input.synchronize(fixture.clock, context);
  require(!fixture.input.faulted() && fixture.clock.bus_cycles() == 1,
          "The last completed period commits using its offered rate before a native source switch");
  static_cast<void>(rejects([&] { fixture.input.observe(*fixture.bus, fixture.clock, context); }));
}
}

int main() {
  try {
    test_all_bytes_routes_and_single_clock_owner();
    test_partial_peer_replay_and_unused_observation();
    test_rate_observation_and_restore_failures();
    test_unknown_receiver_and_native_rate_switch_boundary();
    std::cout
        << "Owner-clock MIDI waveform, routes, time, coordinated replay and failures passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
