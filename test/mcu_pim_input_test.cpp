// SPDX-License-Identifier: GPL-3.0-only
#include "board/gpio_pads.hpp"
#include "common/bus.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_run_clock.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
constexpr unsigned flash_bytes = 32768;
constexpr unsigned oscillator_hz = 12000000;
constexpr std::uint64_t reference_bus_ticks = 10;
constexpr std::uint64_t divided_bus_ticks = 20;
constexpr std::uint64_t phy_bus_ticks = 2;
constexpr std::uint8_t all_bits = 0xff;
constexpr std::uint8_t low_nibble = 0x0f;
constexpr std::uint8_t first_value = 0x5a;
constexpr std::uint8_t next_value = 0xa5;
constexpr std::uint8_t final_value = 0xc3;
constexpr std::uint8_t conflict_bit = 0x02;
constexpr std::uint8_t first_low_nibble = 0x0a;
constexpr std::uint16_t q_input = hc12::ptq + 1;
constexpr std::uint16_t authored_pc = 0x8123;
constexpr std::uint8_t authored_ccr = 0xf0;
constexpr ExecutionContext context{.pc = authored_pc, .ccr = authored_ccr};

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw Fault(std::string(message)); }
}

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected synchronized-input refusal");
}

McuRunClock reference_clock() {
  return McuRunClock({.oscillator_hz = oscillator_hz,
                      .initial_oscillator_phase_ticks = 0,
                      .switch_timing = McuRunClockSwitch::reference_cycle_boundary});
}

void drive(McuBus &bus, const std::uint8_t value) {
  bus.set_pin_drive(McuPort::q, {.mask = all_bits, .levels = value});
}

void run(McuRunClock &clock, McuBus &bus) {
  clock.advance_run_cycle(bus, context);
}

void actual_bus_edges_and_reads() {
  const std::array<std::uint8_t, flash_bytes> image{};
  McuBus bus(image);
  auto clock = reference_clock();
  drive(bus, first_value);
  require(bus.read(hc12::ptq) == first_value,
          "Unclocked transaction fixture has no invented history");
  run(clock, bus);
  const auto refusal = rejects([&] { static_cast<void>(bus.read(hc12::ptq)); });
  require(refusal.contains("0x0260") && refusal.contains("pc=0x8123") &&
              refusal.contains("ccr=0xf0") && refusal.contains("mask=0xff"),
          "First real edge withdraws direct observations and reports unknown input with context");
  drive(bus, next_value);
  run(clock, bus);
  require(bus.read(hc12::ptq) == first_value && bus.read(q_input) == first_value,
          "PTQ input and PTIQ use previous native bus-edge pins");
  drive(bus, final_value);
  const auto count = clock.bus_cycles();
  require(bus.observe_pads(McuPort::q, all_bits).levels == final_value &&
              bus.read(hc12::ptq) == first_value && bus.read(q_input) == first_value &&
              clock.bus_cycles() == count,
          "Physical observation and repeated guest reads never create sampling edges");
  run(clock, bus);
  require(bus.read(hc12::ptq) == next_value && clock.bus_cycles() == 3 &&
              clock.elapsed_ticks() == 3 * reference_bus_ticks,
          "Exactly one input advance occurs per completed MCU bus cycle");
}

void output_latch_and_direction() {
  const std::array<std::uint8_t, flash_bytes> image{};
  McuBus bus(image);
  auto clock = reference_clock();
  drive(bus, first_value);
  run(clock, bus);
  run(clock, bus);
  bus.write(hc12::ptq, next_value);
  bus.write(hc12::ddrq, all_bits);
  require(bus.read(hc12::ptq) == next_value && bus.read(q_input) == first_value,
          "Output data reads its latch immediately; PTIQ still sees physical input history");
  bus.write(hc12::ddrq, 0);
  require(bus.read(hc12::ptq) == first_value, "Changing DDR does not manufacture a new pin sample");
  drive(bus, next_value);
  run(clock, bus);
  require(bus.read(hc12::ptq) == first_value, "Input settling follows the ordinary clocked path");
  run(clock, bus);
  require(bus.read(hc12::ptq) == next_value, "New physical input arrives after two sampling edges");
}

void masks_unknown_and_conflicts() {
  const std::array<std::uint8_t, flash_bytes> image{};
  McuBus bus(image);
  auto clock = reference_clock();
  GpioPadSample physical{.known = low_nibble, .levels = first_low_nibble};
  unsigned observations = 0;
  bus.attach_gpio_pad_source(McuPort::q, [&](const std::uint32_t mask) {
    ++observations;
    return GpioPadSample{.known = physical.known & mask,
                         .levels = physical.levels & mask,
                         .conflicts = physical.conflicts & mask};
  });
  run(clock, bus);
  run(clock, bus);
  const auto before = observations;
  require(bus.read_masked_byte({.address = hc12::ptq, .mask = low_nibble}) == first_low_nibble,
          "A qualified input subset does not require unrelated floating bits");
  const auto unknown = rejects([&] { static_cast<void>(bus.read(q_input)); });
  require(unknown.contains("mask=0xf0") && observations == before,
          "Unknown bits remain refused without sampling physical callbacks during reads");
  constexpr unsigned uncontested = all_bits ^ conflict_bit;
  physical = {.known = uncontested, .levels = uncontested, .conflicts = conflict_bit};
  run(clock, bus);
  run(clock, bus);
  const auto conflict = rejects([&] { static_cast<void>(bus.read(hc12::ptq)); });
  require(conflict.contains("mask=0x02") && conflict.contains("conflicts=0x02"),
          "Clocked conflicts remain distinct from undriven inputs");
  physical = {};
  run(clock, bus);
  run(clock, bus);
  const auto released = rejects([&] { static_cast<void>(bus.read(hc12::ptq)); });
  require(released.contains("mask=0xff") && released.contains("conflicts=0x00"),
          "Released pins never fall back to the last known byte");
}

void native_wait_and_clock_switch() {
  const std::array<std::uint8_t, flash_bytes> image{};
  McuBus bus(image);
  auto clock = reference_clock();
  drive(bus, first_value);
  run(clock, bus);
  drive(bus, next_value);
  clock.advance_wait_cycle(bus, context);
  require(bus.read(hc12::ptq) == first_value && clock.bus_cycles() == 2,
          "Native WAI bus edges continue sampling input registers");
  bus.write(hc12::refdv, 1);
  drive(bus, final_value);
  require(clock.next_cycle_ticks(bus) == divided_bus_ticks && bus.read(hc12::ptq) == first_value,
          "Clock duration offers and route writes do not advance the pipeline");
  run(clock, bus);
  require(bus.read(hc12::ptq) == next_value &&
              clock.elapsed_ticks() == (2 * reference_bus_ticks) + divided_bus_ticks,
          "A longer actual bus period samples once and preserves history");
  constexpr std::uint8_t phy_start = 0x40;
  constexpr std::uint8_t pll_select = 0x80;
  bus.write(hc12::refdv, 0);
  bus.write(hc12::usb_control, phy_start);
  bus.observe_phy({.edges = 15});
  bus.write(hc12::clock_select, pll_select);
  drive(bus, first_value);
  require(clock.next_cycle_ticks(bus) == phy_bus_ticks, "Actual qualified PHY route is selected");
  run(clock, bus);
  require(
      bus.read(hc12::ptq) == final_value && clock.bus_cycles() == 4 &&
          clock.elapsed_ticks() == (2 * reference_bus_ticks) + divided_bus_ticks + phy_bus_ticks,
      "A shorter actual bus period does not insert fixed-time padding or discard input history");
}

void clone_and_reset() {
  const std::array<std::uint8_t, flash_bytes> image{};
  McuBus bus(image);
  auto clock = reference_clock();
  drive(bus, first_value);
  run(clock, bus);
  drive(bus, next_value);
  run(clock, bus);
  auto copy = bus.clone();
  auto copy_clock = clock;
  drive(*copy, final_value);
  run(copy_clock, *copy);
  require(copy->read(hc12::ptq) == next_value && bus.read(hc12::ptq) == first_value &&
              copy_clock.bus_cycles() == 3 && clock.bus_cycles() == 2,
          "A modeled-state clone retains both stages and advances independently");
  bus.reset();
  static_cast<void>(rejects([&] { static_cast<void>(bus.read(hc12::ptq)); }));
  require(copy->read(hc12::ptq) == next_value, "Reset cannot modify another bus's history");
  drive(bus, final_value);
  run(clock, bus);
  static_cast<void>(rejects([&] { static_cast<void>(bus.read(hc12::ptq)); }));
  run(clock, bus);
  require(bus.read(hc12::ptq) == final_value, "Reset requires two fresh sampling edges");
}

void clone_pad_binding() {
  const std::array<std::uint8_t, flash_bytes> image{};
  McuBus bus(image);
  auto clock = reference_clock();
  unsigned source_calls = 0;
  auto physical = first_value;
  bus.attach_gpio_pad_source(McuPort::q, [&](const std::uint32_t mask) {
    ++source_calls;
    return GpioPadSample{.known = mask, .levels = physical & mask};
  });
  run(clock, bus);
  run(clock, bus);
  physical = next_value;
  run(clock, bus);
  auto copy = bus.clone();
  auto copy_clock = clock;
  require(copy->read(hc12::ptq) == first_value,
          "A clone can inspect its own retained input history without sampling another board");
  auto unbound = bus.clone();
  auto unbound_clock = clock;
  static_cast<void>(rejects([&] { run(unbound_clock, *unbound); }));
  require(unbound_clock.faulted() && unbound_clock.bus_cycles() == clock.bus_cycles(),
          "A missing clone pad binding refuses its next clock edge without counting completion");
  const auto before = source_calls;
  copy->attach_gpio_pad_source(McuPort::q, [](const std::uint32_t mask) {
    return GpioPadSample{.known = mask, .levels = final_value & mask};
  });
  run(copy_clock, *copy);
  require(copy->read(hc12::ptq) == next_value && source_calls == before &&
              bus.read(hc12::ptq) == first_value,
          "Rebinding preserves copied pipeline registers without borrowing a live source callback");
  run(copy_clock, *copy);
  require(copy->read(hc12::ptq) == final_value && source_calls == before,
          "Later copied edges sample only the clone's physical provider");
}
}

int main() {
  try {
    actual_bus_edges_and_reads();
    output_latch_and_direction();
    masks_unknown_and_conflicts();
    native_wait_and_clock_switch();
    clone_and_reset();
    clone_pad_binding();
    std::cout << "PIM integration: native bus "
                 "edges/registers/masks/direction/WAI/clock/reset/clone passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
