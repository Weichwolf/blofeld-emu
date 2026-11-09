// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/timer.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
using blofeld::vi::mcu::Timer;
constexpr std::uint16_t tios = 0x40, tcnt = 0x44, tscr1 = 0x46, tctl3 = 0x4a, tctl4 = 0x4b;
constexpr std::uint16_t tie = 0x4c, tscr2 = 0x4d, flags1 = 0x4e, flags2 = 0x4f;
constexpr std::uint16_t tc0 = 0x50, tc1 = 0x52, tc7 = 0x5e, pulse_control = 0x60;
constexpr std::uint16_t enabled = 0x80, fast = 0x90, wait_disabled = 0xc0;
constexpr std::uint16_t irq0 = 0xffee, irq1 = 0xffec, irq7 = 0xffe0, irq_overflow = 0xffde;
constexpr Timer::Cycle period = 65536;
constexpr std::uint16_t all_channels = 255, output_control = 0x48;
constexpr std::uint16_t compare_mask = 0x42, compare_data = 0x43;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void advance(Timer &timer, const Timer::Cycle time) {
  require(timer.advance(time), "Advance monotonic device cycles");
}

void write(Timer &timer, const Timer::Write access) {
  require(timer.write(access), "Write a supported timer register");
}

std::uint16_t read(Timer &timer, const Timer::Read access) {
  std::uint16_t value = 0;
  require(timer.read(access, value), "Read a supported timer register");
  return value;
}

void counter_and_gates() {
  Timer timer;
  constexpr Timer::Cycle initial = 1000;
  constexpr Timer::Cycle interval = 100;
  advance(timer, initial);
  require(read(timer, {.offset = tcnt, .bytes = 2}) == 0, "Disabled counter does not advance");
  write(timer, {.offset = tscr1, .value = enabled});
  advance(timer, initial + 5);
  require(read(timer, {.offset = tcnt, .bytes = 2}) == 5, "Counter counts actual elapsed cycles");
  constexpr std::uint16_t all_bits = 0xffff;
  write(timer, {.offset = tcnt, .value = all_bits, .bytes = 2});
  require(read(timer, {.offset = tcnt, .bytes = 2}) == 5, "TCNT writes are ignored in normal mode");
  write(timer, {.offset = tscr1, .value = 0});
  advance(timer, initial + interval);
  write(timer, {.offset = tscr1, .value = enabled});
  advance(timer, initial + interval + 1);
  require(read(timer, {.offset = tcnt, .bytes = 2}) == 6, "Disable preserves the retained count");
  require(!timer.advance(initial), "A backwards timestamp is refused");
  write(timer, {.offset = tscr1, .value = wait_disabled});
  timer.activity(Timer::Activity::waiting);
  advance(timer, initial + (interval * 2));
  require(read(timer, {.offset = tcnt, .bytes = 2}) == 6, "TSWAI gates a sleeping timer");
  timer.activity(Timer::Activity::running);
  advance(timer, initial + (interval * 2) + 1);
  require(read(timer, {.offset = tcnt, .bytes = 2}) == 7, "Wake resumes the retained counter");
  timer.activity(Timer::Activity::stopped);
  advance(timer, initial + (interval * 3));
  require(read(timer, {.offset = tcnt, .bytes = 2}) == 7 && !timer.deadline(),
          "STOP supplies no timer clocks or future timer interrupt");
}

void prescaler_and_deadlines() {
  Timer timer;
  write(timer, {.offset = tios, .value = 1});
  constexpr std::uint16_t compare = 130;
  write(timer, {.offset = tc0, .value = compare, .bytes = 2});
  write(timer, {.offset = tie, .value = 1});
  write(timer, {.offset = tscr1, .value = enabled});
  require(timer.deadline() == compare, "Deadline is the real output compare time");
  advance(timer, 3);
  write(timer, {.offset = tscr2, .value = 2});
  constexpr Timer::Cycle synchronized = 128;
  constexpr Timer::Cycle expected_deadline = 136;
  require(timer.deadline() == expected_deadline,
          "Divider change waits for all seven stages to zero");
  advance(timer, synchronized - 1);
  require(read(timer, {.offset = tcnt, .bytes = 2}) == synchronized - 1,
          "Previous divider remains active before the synchronization edge");
  advance(timer, expected_deadline - 1);
  require(read(timer, {.offset = tcnt, .bytes = 2}) == compare - 1 && !timer.pending(),
          "New divide-by-four phase precedes the actual compare");
  advance(timer, expected_deadline);
  require(timer.pending() == irq0 && !timer.deadline(),
          "Latched compare asserts the actual vector");
  write(timer, {.offset = flags1, .value = 1});
  require(!timer.pending() && timer.deadline() == expected_deadline + (period * 4),
          "Clearing the flag restores the next full-wrap compare deadline");
  Timer paused;
  write(paused, {.offset = tscr1, .value = enabled});
  advance(paused, 3);
  write(paused, {.offset = tscr1, .value = 0});
  write(paused, {.offset = tscr2, .value = 2});
  write(paused, {.offset = tios, .value = 1});
  write(paused, {.offset = tc0, .value = compare, .bytes = 2});
  write(paused, {.offset = tie, .value = 1});
  constexpr Timer::Cycle resume_time = 100;
  constexpr Timer::Cycle resumed_deadline = 233;
  advance(paused, resume_time);
  write(paused, {.offset = tscr1, .value = enabled});
  require(paused.deadline() == resumed_deadline,
          "A paused divider cannot latch a pending selection before its synchronization edge");
}

void flags_and_priority() {
  Timer timer;
  write(timer, {.offset = tios, .value = 3});
  constexpr std::uint16_t later = 100;
  constexpr std::uint16_t earlier = 50;
  write(timer, {.offset = tc0, .value = later, .bytes = 2});
  write(timer, {.offset = tc1, .value = earlier, .bytes = 2});
  write(timer, {.offset = tie, .value = 3});
  write(timer, {.offset = tscr1, .value = enabled});
  require(timer.deadline() == earlier, "Earliest enabled comparison owns the deadline");
  advance(timer, earlier);
  require(timer.pending() == irq1 && timer.deadline() == later,
          "Pending flags do not hide later sources");
  advance(timer, later);
  require(timer.pending() == irq0 && read(timer, {.offset = flags1}) == 3,
          "Channel zero has fixed priority and both flags remain asserted");
  write(timer, {.offset = flags1, .value = 0});
  require(read(timer, {.offset = flags1}) == 3, "Writing zero does not acknowledge flags");
  write(timer, {.offset = tscr1, .value = 0});
  write(timer, {.offset = flags1, .value = 3});
  require(read(timer, {.offset = flags1}) == 3, "W1C requires TEN enabled");
  write(timer, {.offset = tscr1, .value = enabled});
  write(timer, {.offset = flags1, .value = 1});
  require(timer.pending() == irq1, "Acknowledgement clears only selected flags");
}

void capture_and_fast_clear() {
  Timer timer;
  write(timer, {.offset = tctl4, .value = 1});
  write(timer, {.offset = tie, .value = 1});
  write(timer, {.offset = tscr1, .value = fast});
  constexpr Timer::Cycle captured = 23;
  advance(timer, captured);
  require(timer.capture({.channel = 0, .rising = false}) && !timer.pending(),
          "Wrong capture polarity does not set flags");
  require(timer.capture({.channel = 0, .rising = true}) && timer.pending() == irq0,
          "Selected input edge captures the current device time");
  require(read(timer, {.offset = tc0, .bytes = 2}) == captured && !timer.pending(),
          "Reading the input capture value fast-clears its flag");
  write(timer, {.offset = tc0, .value = 1, .bytes = 2});
  require(read(timer, {.offset = tc0, .bytes = 2}) == captured, "Input capture writes are ignored");
  require(timer.capture({.channel = 0, .rising = true}), "Capture another event");
  write(timer, {.offset = tios, .value = 1});
  static_cast<void>(read(timer, {.offset = tc0, .bytes = 2}));
  require(timer.pending() == irq0, "Reading output compare does not fast-clear");
  write(timer, {.offset = tc0, .value = 1, .bytes = 2});
  require(!timer.pending(), "Writing output compare fast-clears");
  constexpr std::uint16_t both_edges_seven = 0xc0;
  write(timer, {.offset = tctl3, .value = both_edges_seven});
  write(timer, {.offset = tie, .value = enabled});
  require(timer.capture({.channel = 7, .rising = false}) && timer.pending() == irq7,
          "Upper channels use their own edge selector and interrupt vector");
  require(read(timer, {.offset = tc7, .bytes = 2}) == captured && !timer.pending(),
          "Channel seven capture has its independent fast-clear flag");
}

void overflow_and_atomic_access() {
  Timer timer;
  write(timer, {.offset = tscr1, .value = enabled});
  write(timer, {.offset = tscr2, .value = enabled});
  require(timer.deadline() == period, "Overflow deadline is the counter wrap");
  advance(timer, period);
  require(timer.pending() == irq_overflow && read(timer, {.offset = flags2}) == enabled,
          "Overflow latches TOF and the documented vector");
  write(timer, {.offset = tscr1, .value = fast});
  static_cast<void>(read(timer, {.offset = tcnt, .bytes = 2}));
  require(!timer.pending(), "TCNT access fast-clears overflow");
  advance(timer, period + all_channels);
  const auto high = read(timer, {.offset = tcnt});
  advance(timer, period + 256);
  const auto low = read(timer, {.offset = static_cast<std::uint16_t>(tcnt + 1)});
  require(high == 0 && low == 0 && read(timer, {.offset = tcnt, .bytes = 2}) == 256,
          "Separate byte samples can tear; a word access samples one instant");
  Timer large;
  write(large, {.offset = tios, .value = all_channels});
  write(large, {.offset = tscr1, .value = enabled});
  advance(large, std::numeric_limits<Timer::Cycle>::max());
  constexpr std::uint16_t last_count = 0xffff;
  require(read(large, {.offset = tcnt, .bytes = 2}) == last_count &&
              read(large, {.offset = flags1}) == all_channels &&
              read(large, {.offset = flags2}) == enabled,
          "Huge elapsed time coalesces real flags without per-cycle iteration or overflow");
}

void unsupported_contracts() {
  Timer timer;
  require(!timer.write({.offset = tscr2, .value = 8}), "TCRE is explicitly unsupported");
  require(!timer.write({.offset = pulse_control, .value = 0x40}),
          "Pulse clocking is explicitly unsupported");
  require(!timer.write({.offset = output_control, .value = 1}),
          "Unconnected compare outputs cannot be enabled");
  require(!timer.capture({.channel = 8, .rising = true}), "Unknown capture channel fails");
}

void retained_compare_mask() {
  Timer timer;
  constexpr std::uint16_t mask = 0x40;
  constexpr std::uint16_t data = 0xa5;
  constexpr Timer::Cycle elapsed = 100;
  write(timer, {.offset = compare_mask, .value = mask});
  write(timer, {.offset = compare_data, .value = data});
  write(timer, {.offset = tscr1, .value = enabled});
  write(timer, {.offset = tie, .value = all_channels});
  advance(timer, elapsed);
  require(read(timer, {.offset = compare_mask}) == mask &&
              read(timer, {.offset = compare_data}) == data && read(timer, {.offset = tios}) == 0 &&
              !timer.pending() && !timer.deadline(),
          "Input capture retains OC7M/OC7D without inventing output comparisons");
  require(!timer.write({.offset = tios, .value = mask}) && read(timer, {.offset = tios}) == 0 &&
              read(timer, {.offset = compare_mask}) == mask,
          "Selecting a masked output fails before changing either retained register");
  write(timer, {.offset = tios, .value = 1});
  require(!timer.write({.offset = compare_mask, .value = mask | 1}) &&
              read(timer, {.offset = compare_mask}) == mask,
          "Adding a mask for an existing output cannot silently enable unsupported routing");
  write(timer, {.offset = compare_mask, .value = 0});
  write(timer, {.offset = tios, .value = mask});
  require(!timer.write({.offset = compare_mask, .value = mask}) &&
              read(timer, {.offset = compare_mask}) == 0,
          "An output selected first observes the same unsupported-route boundary");
}
}

int main() try {
  counter_and_gates();
  prescaler_and_deadlines();
  flags_and_priority();
  capture_and_fast_clear();
  overflow_and_atomic_access();
  unsupported_contracts();
  retained_compare_mask();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
