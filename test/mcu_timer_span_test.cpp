// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/mcu_timer.hpp"

#include <array>
#include <bit>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
constexpr std::uint16_t tios = 0x40;
constexpr std::uint16_t control1 = 0x46;
constexpr std::uint16_t control2 = 0x4d;
constexpr std::uint16_t flags1 = 0x4e;
constexpr std::uint16_t flags2 = 0x4f;
constexpr std::uint16_t channel0 = 0x50;
constexpr std::uint16_t channel7 = 0x5e;
constexpr unsigned enabled = 0x80;
constexpr unsigned counter_range = 65536;
constexpr unsigned maximum_slice = 1048576;
constexpr std::uint16_t registers_end = 0x60;
constexpr std::uint16_t output_control_low = 0x49;
constexpr std::uint16_t compare7_mask = 0x42;
constexpr std::uint16_t compare7_data = 0x43;
constexpr std::uint16_t edge_control_low = 0x4b;
constexpr unsigned compare_channels = 0x81;
constexpr unsigned fault_slice = 1000;
unsigned assertions = 0;
std::uint64_t signature = 0;

void require(const bool condition, const std::string &message) {
  ++assertions;
  if (!condition) { throw blofeld::Fault(message); }
}

void record(const unsigned value) {
  signature = std::rotl(signature, 7) ^ value;
}

void write(blofeld::McuTimer &timer, const std::uint16_t address, const unsigned value) {
  timer.write({.address = address, .value = static_cast<std::uint8_t>(value)});
}

void word(blofeld::McuTimer &timer, const std::uint16_t address, const unsigned value) {
  write(timer, address, value >> 8U);
  write(timer, static_cast<std::uint16_t>(address + 1U), value);
}

unsigned counter(blofeld::McuTimer timer) {
  constexpr std::uint16_t high = 0x44;
  constexpr std::uint16_t low = 0x45;
  return (static_cast<unsigned>(timer.read(high)) << 8U) | timer.read(low);
}

std::string refuses(const auto &operation) {
  try {
    operation();
  } catch (const blofeld::Fault &error) { return error.what(); }
  throw blofeld::Fault("Expected timer refusal");
}

void snapshot(blofeld::McuTimer timer) {
  for (std::uint16_t address = tios; address < registers_end; ++address) {
    record(timer.read(address));
  }
  const auto pins = timer.pins();
  for (const auto value : {pins.inputs, pins.outputs, pins.levels, pins.unknown}) { record(value); }
  record(timer.interrupt_channels());
  record(timer.overflow_interrupt() ? 1U : 0U);
  record(timer.cycles_until_tick());
  record(timer.cycles_until_event());
}

void literal_counts_and_overflow() {
  constexpr std::array lengths{0U, 1U, 127U, 128U, 129U, 65535U, 65536U, maximum_slice};
  for (unsigned prescale = 0; prescale < 8; ++prescale) {
    for (const auto length : lengths) {
      blofeld::McuTimer timer;
      require(timer.cycles_until_event() == 0, "Disabled timer offers no clock effect");
      write(timer, control2, prescale);
      write(timer, control1, enabled);
      const auto divider = 1U << prescale;
      require(timer.cycles_until_event() == counter_range * divider,
              "First enabled effect is literal16-bit overflow at the chosen divider");
      timer.advance(length);
      require(counter(timer) == (length / divider) % counter_range,
              "Counter is independently expected floor(cycles/divisor) modulo65536");
      require(timer.read(flags2) == (length / divider >= counter_range ? enabled : 0U),
              "Only an actual wrap sets overflow");
      snapshot(timer);
      timer.advance(1);
      require(counter(timer) == ((length + 1) / divider) % counter_range,
              "A scalar suffix retains the exact bulk prescaler phase");
    }
  }
}

void delayed_prescale_and_splits() {
  blofeld::McuTimer timer;
  write(timer, control1, enabled);
  timer.advance(3);
  constexpr unsigned before_latch = 124;
  constexpr unsigned before_boundary = 127;
  constexpr unsigned after_boundary = 129;
  constexpr unsigned overflow_distance = 261629;
  write(timer, control2, 2);
  require(timer.cycles_until_event() == before_latch + 1,
          "Divisor request latches at phase128, not the next count");
  timer.advance(before_latch);
  require(counter(timer) == before_boundary && timer.cycles_until_event() == 1,
          "Old divider remains active before its final latch period");
  timer.advance(1);
  timer.advance(3);
  require(counter(timer) == 128 && timer.cycles_until_event() == overflow_distance,
          "New divide-by-four phase and overflow distance are independently specified");
  timer.advance(1);
  require(counter(timer) == after_boundary, "Fourth post-latch cycle increments exactly once");
  snapshot(timer);
  constexpr std::array pieces{17U, 111U, 1025U, 65535U, 104857U};
  for (const auto piece : pieces) {
    timer.advance(piece);
    snapshot(timer);
  }
}

void compares_and_reset_phase() {
  for (unsigned prescale = 0; prescale < 8; ++prescale) {
    blofeld::McuTimer timer;
    write(timer, tios, enabled);
    word(timer, channel7, 3);
    write(timer, control2, prescale | 8U);
    write(timer, control1, enabled);
    require(timer.cycles_until_event() == 1,
            "New comparator activation validates before time passes");
    const auto divider = 1U << prescale;
    timer.advance(3 * divider);
    require(counter(timer) == 3 && timer.cycles_until_event() == 1 && timer.read(flags1) == enabled,
            "TC7 compare precedes its next-bus-cycle counter reset for all dividers");
    timer.advance(1);
    require(counter(timer) == 0 && timer.cycles_until_event() == 3 * divider,
            "Reset consumes one bus period and establishes a fresh divider phase");
    const auto period = (3 * divider) + 1;
    constexpr unsigned frames = 97;
    timer.advance((frames * period) + (2 * divider));
    require(counter(timer) == 2, "Repeated TC7 resets preserve literal periodic count two");
    snapshot(timer);
  }
  blofeld::McuTimer timer;
  write(timer, tios, compare_channels);
  word(timer, channel0, 3);
  word(timer, channel7, 3);
  write(timer, output_control_low, 1);
  write(timer, compare7_mask, compare_channels);
  write(timer, compare7_data, 1);
  write(timer, control1, enabled);
  timer.advance(3);
  require(timer.pins().levels == 1 && timer.pins().unknown == 0 &&
              timer.read(flags1) == compare_channels,
          "TC7 override resolves the simultaneous unknown toggle without changing priority");
  snapshot(timer);
}

void visible_failures_and_partial_state() {
  blofeld::McuTimer timer;
  require(refuses([&] { timer.advance(maximum_slice + 1); }).contains("cycle limit"),
          "Oversized input fails even when disabled");
  write(timer, edge_control_low, 1);
  write(timer, control1, enabled);
  require(timer.cycles_until_event() == 1 &&
              refuses([&] { timer.advance(fault_slice); }).contains("Unknown active timer capture"),
          "Unknown capture input retains its first-cycle fault boundary");
  require(counter(timer) == 0, "Unknown capture cannot publish skipped counter time");
  timer.reset();
  write(timer, tios, 1);
  word(timer, channel0, 0);
  write(timer, control1, enabled);
  require(timer.cycles_until_event() == 1 &&
              refuses([&] { timer.advance(fault_slice); }).contains("activation"),
          "Compare-at-current activation remains an immediate refusal");
  require(counter(timer) == 0, "Activation refusal leaves counter unchanged");
  timer.reset();
  write(timer, tios, 1);
  word(timer, channel0, 7);
  write(timer, output_control_low, 1);
  write(timer, control1, enabled);
  require(refuses([&] { timer.advance(fault_slice); }).contains("Unknown timer output latch"),
          "Bulk advance retains unknown toggle fault instead of inventing an output");
  require(counter(timer) == 7 && timer.cycles_until_tick() == 1,
          "A failing effect preserves original partial count and prescaler order");
  snapshot(timer);
}
}

int main() {
  try {
    literal_counts_and_overflow();
    delayed_prescale_and_splits();
    compares_and_reset_phase();
    visible_failures_and_partial_state();
    std::cout << "Timer spans: " << assertions << " assertions, signature " << signature << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
