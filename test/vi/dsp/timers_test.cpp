// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/timers.hpp"

#include <cstdint>
#include <dsp56300/memory.hpp>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {
using blofeld::vi::dsp::Timers;
using dsp56300::Word;
constexpr Word tplr = 0xffff83, tpcr = 0xffff82, control = 0xffff8f;
constexpr Word load = 0xffff8e, compare = 0xffff8d, count = 0xffff8c;
constexpr Word tcf = 0x200000, tof = 0x100000, input_bit = 0x1000;
constexpr Word invert = 0x100, reload = 0x200, prescale = 0x8000;
constexpr Word overflow_vector = 0x56, compare_vector = 0x54;
constexpr std::uint64_t idle_end = 100, external_end = 200;
constexpr std::uint64_t prescaled_compare = 24, resumed_compare = 124, reloaded_compare = 606;
constexpr std::uint64_t word_modulus = 0x1000000;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void put(Timers &timers, const Timers::Write access) {
  require(timers.write(access), "Write a supported DSP timer register");
}

Word read(const Timers &timers, const Word address) {
  Word value = 0;
  require(timers.read(address, value), "Read a readable DSP timer register");
  return value;
}

void reset_and_access() {
  Timers timers;
  Word value = 0;
  require(read(timers, tplr) == 0 && read(timers, tpcr) == 0 && read(timers, count) == 0 &&
              read(timers, control) == 0 && !timers.read(load, value) &&
              !Timers::contains(tplr - 2),
          "Documented count/control reset values and write-only TLR remain distinct");
  put(timers, {.address = load, .value = 7});
  put(timers, {.address = compare, .value = 13});
  put(timers, {.address = tpcr, .value = 9});
  put(timers, {.address = count, .value = 9});
  require(read(timers, count) == 0 && read(timers, tpcr) == 0,
          "Read-only register writes never manufacture counter progress");
  timers.reset();
  require(read(timers, compare) == 13, "Unspecified reset values retain the written compare");
  put(timers, {.address = control, .value = 1});
  require(timers.advance(2) && read(timers, count) == 7,
          "The first CLK/2 event loads retained TLR");
  require(timers.advance(3) && read(timers, count) == 7 && timers.advance(4) &&
              read(timers, count) == 8,
          "An odd half-clock carries into the next update without being counted twice");
  put(timers, {.address = control, .value = 1});
  require(read(timers, count) == 8 && timers.advance(6) && read(timers, count) == 9,
          "TE=1 to TE=1 preserves the running counter and its phase");
  put(timers, {.address = control, .value = 0});
  require(timers.advance(idle_end) && read(timers, count) == 9 && !timers.deadline(),
          "Disabled timers hold their counters and request no execution horizon");
}

void compare_overflow_and_flags() {
  Timers timers;
  put(timers, {.address = load, .value = 7});
  put(timers, {.address = compare, .value = 9});
  put(timers, {.address = control, .value = 7});
  require(timers.deadline() == 6 && timers.advance(5) && !timers.interrupt() && timers.advance(6) &&
              timers.interrupt() == compare_vector && timers.take_requests() == 1 &&
              timers.take_requests() == 0,
          "Loading then two increments reaches TCPR exactly at the predicted compare boundary");
  put(timers, {.address = control, .value = 7});
  require((read(timers, control) & tcf) != 0, "A zero write never acknowledges TCF");
  put(timers, {.address = control, .value = tcf | 7});
  require(!timers.interrupt() && (read(timers, control) & tcf) == 0,
          "A one write acknowledges only its selected flag");
  put(timers, {.address = control, .value = 0});
  put(timers, {.address = load, .value = dsp56300::word_mask});
  put(timers, {.address = compare, .value = 0});
  put(timers, {.address = control, .value = 7});
  require(timers.advance(10) && read(timers, count) == 0 &&
              (read(timers, control) & (tcf | tof)) == (tcf | tof) &&
              timers.interrupt() == overflow_vector,
          "Wrap preserves both matching flags and overflow has higher local priority");
  put(timers, {.address = control, .value = tof | 7});
  require(timers.interrupt() == compare_vector, "Overflow acknowledgement preserves compare");
  put(timers, {.address = control, .value = 0});
  require(read(timers, control) == 0 && !timers.interrupt(),
          "Disable clears both documented flags");
}

void reload_and_batching() {
  Timers batch;
  Timers split;
  for (auto *timers : {&batch, &split}) {
    put(*timers, {.address = load, .value = 7});
    put(*timers, {.address = compare, .value = 9});
    put(*timers, {.address = control, .value = reload | 1});
  }
  constexpr std::uint64_t end = 601;
  require(batch.advance(end), "Advance whole event intervals analytically");
  for (std::uint64_t cycle = 1; cycle <= end; ++cycle) {
    require(split.advance(cycle), "Advance independent single-cycle partitions");
  }
  require(read(batch, count) == 9 && read(split, count) == 9 &&
              read(batch, control) == read(split, control) && batch.deadline() == reloaded_compare,
          "Reload consumes its own event; partitioning preserves the 7,8,9 cycle");
  Timers wrap;
  put(wrap, {.address = load, .value = 0});
  put(wrap, {.address = compare, .value = 9});
  put(wrap, {.address = control, .value = 1});
  require(wrap.advance((word_modulus + 6) * 2) && read(wrap, count) == 5 &&
              (read(wrap, control) & (tcf | tof)) == (tcf | tof),
          "Large analytical advances retain actual wrap and compare events");
}

void prescaler_and_external_events() {
  Timers timers;
  put(timers, {.address = tplr, .value = 3});
  put(timers, {.address = load, .value = 7});
  put(timers, {.address = compare, .value = 9});
  put(timers, {.address = control, .value = prescale | 1});
  require(timers.deadline() == prescaled_compare && read(timers, tpcr) == 3 && timers.advance(6) &&
              read(timers, count) == 0 && read(timers, tpcr) == 0 && timers.advance(8) &&
              read(timers, count) == 7 && timers.advance(prescaled_compare) &&
              read(timers, count) == 9,
          "The shared downcounter emits one pulse per preload plus one input clocks");
  put(timers, {.address = control, .value = 0});
  require(timers.advance(idle_end), "Disabled prescaler does not accumulate hidden elapsed pulses");
  put(timers, {.address = control, .value = prescale | 1});
  require(read(timers, tpcr) == 3 && timers.deadline() == resumed_compare,
          "Enabling the first prescaled timer reloads the shared count");
  Timers external;
  constexpr Word tio1_source = 0x400001;
  put(external, {.address = tplr, .value = tio1_source});
  put(external, {.address = load, .value = 7});
  put(external, {.address = compare, .value = 9});
  put(external, {.address = control, .value = prescale | 1});
  require(!external.deadline() && external.advance(idle_end) && read(external, count) == 0,
          "An external prescaler cannot progress on host or internal clocks");
  for (unsigned pulse = 0; pulse < 6; ++pulse) {
    require(external.input(1, true) && external.input(1, false), "Supply real TIO1 transitions");
  }
  require(read(external, count) == 9 && external.take_requests() == 1,
          "Six external edges through a divide-by-two prescaler load and increment three times");
}

void capture_and_event_counter() {
  Timers timers;
  put(timers, {.address = load, .value = 7});
  constexpr Word capture_falling = 0x165;
  put(timers, {.address = control, .value = capture_falling});
  require(timers.input(0, true) && timers.advance(8) && read(timers, count) == 0 &&
              !timers.interrupt() && (read(timers, control) & input_bit) == 0,
          "Capture waits for the selected edge; INV also affects DI independently of writes");
  require(timers.input(0, false) && read(timers, count) == 10 &&
              timers.interrupt() == compare_vector && !timers.deadline() &&
              timers.advance(idle_end) && read(timers, count) == 10,
          "The first falling edge captures real elapsed count and halts measurement");
  put(timers, {.address = control, .value = invert});
  put(timers, {.address = compare, .value = 9});
  constexpr Word event_counter = 0x131;
  put(timers, {.address = control, .value = event_counter});
  require(!timers.deadline() && timers.advance(external_end),
          "External event mode has no internal deadline");
  for (unsigned pulse = 0; pulse < 3; ++pulse) {
    require(timers.input(0, true) && timers.input(0, false), "Count actual falling TIO0 edges");
  }
  require(read(timers, count) == 9 && (read(timers, control) & tcf) != 0,
          "Three selected edges load 7 and increment through 8 to 9");
}

void gpio_and_rejected_modes() {
  Timers timers;
  constexpr Word gpio_output = 0x2900;
  put(timers, {.address = control, .value = gpio_output});
  require(read(timers, control) == (gpio_output | input_bit),
          "Output inversion followed by input inversion preserves the written GPIO data");
  put(timers, {.address = control, .value = invert});
  require(read(timers, control) == (invert | input_bit) && timers.input(0, true) &&
              read(timers, control) == invert,
          "GPIO input inversion samples actual supplied input without altering a counter");
  constexpr Word unimplemented_pulse = 0x11;
  require(!timers.write({.address = control, .value = unimplemented_pulse}) &&
              timers.error() != nullptr,
          "An unimplemented mode cannot silently run as a GPIO timer");
  timers.reset();
  constexpr Word timer2_control = 0xffff87;
  constexpr Word capture = 0x61;
  require(!timers.write({.address = timer2_control, .value = capture}) && timers.error() != nullptr,
          "Timer2 accepts only its documented GPIO timer mode");
}
}

int main() try {
  reset_and_access();
  compare_overflow_and_flags();
  reload_and_batching();
  prescaler_and_external_events();
  capture_and_event_counter();
  gpio_and_rejected_modes();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
