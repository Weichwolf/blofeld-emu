// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_timers.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {
// DSP56371 register/CSR coordinates; timer 0 is the highest address block.
constexpr unsigned control_base = 0xffff8f;
constexpr unsigned prescaler_load = 0xffff83;
constexpr unsigned prescaler_count = 0xffff82;
constexpr unsigned enable = 1, overflow_irq = 2, compare_irq = 4;
constexpr unsigned capture = 0x60, event = 0x30, reload = 0x200, prescale = 0x8000;
constexpr unsigned overflow_flag = 0x100000, compare_flag = 0x200000;
constexpr unsigned flags = overflow_flag | compare_flag;
constexpr unsigned word_max = 0xffffff;
constexpr unsigned prescaler_max = 0x1fffff;
constexpr unsigned compare_vector = 0x54;
constexpr std::uint64_t word_range = 0x1000000;
std::uint64_t assertions = 0;

void require(const bool accepted, const std::string &message) {
  ++assertions;
  if (!accepted) { throw blofeld::Fault(message); }
}

struct Configuration {
  unsigned load;
  unsigned compare;
  unsigned control;
};

struct Timer {
  blofeld::DspTimers timers;
  unsigned csr;

  explicit Timer(const unsigned index) : csr(control_base - (4 * index)) {}

  void write(const unsigned offset, const unsigned value) {
    timers.write({.address = csr - offset, .value = value});
  }

  void configure(const Configuration value) {
    write(1, value.load);
    write(2, value.compare);
    write(0, value.control);
  }

  void pending(const unsigned offset) const {
    require(timers.interrupt_vectors() == std::vector{compare_vector + control_base - csr + offset},
            "The literal clock/input edge must latch its actual enabled IRQ");
  }

  void deadline(const std::optional<std::uint64_t> expected) const {
    const auto count = timers.read(csr - 3);
    const auto status = timers.read(csr, flags);
    const auto prescaler = timers.read(prescaler_count);
    const auto vectors = timers.interrupt_vectors();
    const auto actual = timers.next_clk2_boundary();
    require(actual == expected && (!actual || *actual > 0),
            "Literal timer boundary at " + blofeld::hex(csr, 6) +
                " expected=" + std::to_string(expected.value_or(0)) +
                " actual=" + std::to_string(actual.value_or(0)));
    require(timers.next_clk2_boundary() == expected, "Repeated query must remain pure");
    require(timers.read(csr - 3) == count && timers.read(csr, flags) == status &&
                timers.read(prescaler_count) == prescaler && timers.interrupt_vectors() == vectors,
            "Boundary inspection must not mutate counters, flags or IRQs");
  }
};

void test_free_running(const unsigned index) {
  Timer timer(index);
  timer.deadline(std::nullopt);
  timer.configure({.load = 3, .compare = 7, .control = enable | compare_irq | overflow_irq});
  // Preload at edge 1; literal 3,4,5,6,7 gives compare at edge 5.
  for (unsigned elapsed = 0; elapsed < 5; ++elapsed) {
    timer.deadline(5 - elapsed);
    require(timer.timers.interrupt_vectors().empty(), "No IRQ before the literal compare edge");
    timer.timers.advance_clk2(1);
  }
  timer.deadline(word_range - 7);
  require(timer.timers.interrupt_vectors() == std::vector{compare_vector + (4 * index)},
          "The compare edge admits the derivative's actual vector");
  timer.timers.acknowledge_interrupt(compare_vector + (4 * index));
  timer.deadline(word_range - 7);
  timer.timers.advance_clk2(word_range - 8);
  timer.deadline(1);
  require(timer.timers.interrupt_vectors().empty(), "Overflow has not happened one edge early");
  timer.timers.advance_clk2(1);
  timer.deadline(7);
  timer.pending(2);
}

void test_reload_and_live_writes(const unsigned index) {
  Timer timer(index);
  timer.configure({.load = 3, .compare = 5, .control = enable | reload | compare_irq});
  timer.deadline(3);
  timer.timers.advance_clk2(3);
  timer.deadline(std::nullopt);
  timer.pending(0);
  timer.write(0, enable | reload | compare_irq | flags);
  timer.deadline(3);
  timer.write(0, enable | reload | overflow_irq);
  timer.deadline(std::nullopt);
  timer.write(1, word_max - 1);
  timer.write(2, 1);
  timer.deadline(3);
  timer.timers.advance_clk2(2);
  timer.deadline(1);
  timer.timers.advance_clk2(1);
  timer.deadline(std::nullopt);
  timer.pending(2);

  Timer next(index);
  next.configure({.load = 3, .compare = 7, .control = enable | reload | overflow_irq});
  next.timers.advance_clk2(3); // count 5; compare two increments away.
  next.write(1, word_max - 1);
  next.deadline(5); // 6,7,load(max-1),max,0.
  next.timers.advance_clk2(4);
  next.deadline(1);
  require(next.timers.interrupt_vectors().empty(), "Reloaded overflow must not be early");
  next.timers.advance_clk2(1);
  next.deadline(std::nullopt);
  next.pending(2);
}

void test_equal_and_wrap(const unsigned index) {
  Timer timer(index);
  timer.configure({.load = 7, .compare = 7, .control = enable | reload | overflow_irq});
  timer.deadline(std::nullopt);
  timer.timers.advance_clk2(word_range + 1);
  timer.deadline(std::nullopt);
  timer.write(0, enable | reload | compare_irq);
  timer.deadline(std::nullopt); // A pending, masked IRQ cannot create a zero-time region.
  timer.pending(0);

  Timer wrap(index);
  wrap.configure({.load = word_max - 1,
                  .compare = 1,
                  .control = enable | reload | compare_irq | overflow_irq});
  wrap.deadline(3);
  wrap.timers.advance_clk2(3);
  wrap.deadline(1);
  wrap.pending(2);

  Timer live(index);
  live.configure({.load = 3, .compare = 7, .control = enable | compare_irq});
  live.timers.advance_clk2(2);
  live.write(2, 4); // TCPR == current count does not invent an earlier match.
  live.deadline(word_range);
  live.timers.advance_clk2(word_range - 1);
  live.deadline(1);
  live.timers.advance_clk2(1);
  live.deadline(std::nullopt);
  live.pending(0);
}

void test_prescaled(const unsigned index) {
  constexpr std::array divisors{0U, 1U, 3U, prescaler_max};
  for (const auto preload : divisors) {
    Timer timer(index);
    timer.timers.write({.address = prescaler_load, .value = preload});
    timer.configure({.load = 3, .compare = 5, .control = enable | compare_irq | prescale});
    const auto period = static_cast<std::uint64_t>(preload) + 1;
    timer.deadline(3 * period);
    timer.timers.advance_clk2(period);
    timer.deadline(2 * period);
    timer.timers.write({.address = prescaler_load, .value = 7});
    timer.deadline(period + 8); // Current countdown unchanged; following pulse uses 8.
    timer.timers.advance_clk2(period + 7);
    timer.deadline(1);
    timer.timers.advance_clk2(1);
    timer.deadline(std::nullopt);
    timer.pending(0);
  }
}

void test_capture_and_unknown_inputs() {
  for (unsigned index = 0; index < 2; ++index) {
    Timer timer(index);
    timer.timers.observe_synchronized_tio(index, false);
    timer.configure(
        {.load = word_max - 1, .compare = 0, .control = enable | capture | overflow_irq});
    timer.deadline(3);
    timer.timers.advance_clk2(2);
    timer.deadline(1);
    timer.timers.observe_synchronized_tio(index, true); // Capture halts the counter.
    timer.deadline(std::nullopt);
    timer.write(0, enable | capture | overflow_irq | compare_irq);
    timer.deadline(std::nullopt);
    timer.pending(0);

    Timer external(index);
    external.timers.observe_synchronized_tio(index, false);
    external.configure({.load = 3, .compare = 5, .control = enable | event | compare_irq});
    external.deadline(std::nullopt);            // Actual synchronized input owns the deadline.
    external.timers.set_tio_drive(index, true); // Makes synchronization unqualified.
    external.deadline(1);
    bool refused = false;
    try {
      external.timers.advance_clk2(1);
    } catch (const blofeld::Fault &) { refused = true; }
    require(refused, "Unknown synchronized input still fails at the next actual clock");
  }
  Timer missing(2);
  missing.write(0, enable | prescale);
  missing.deadline(1);
  bool refused = false;
  try {
    missing.timers.advance_clk2(1);
  } catch (const blofeld::Fault &) { refused = true; }
  require(refused, "Unknown load/compare must not disappear inside a native region");
}

void test_shared_prescaler_and_earliest_timer() {
  Timer first(0);
  first.timers.write({.address = prescaler_load, .value = 3});
  first.configure({.load = 0, .compare = 10, .control = enable | compare_irq | prescale});
  constexpr std::uint64_t first_compare_edge =
      44; // Eleven prescaled pulses, four CLK/2 edges each.
  first.deadline(first_compare_edge);
  first.timers.advance_clk2(2);
  first.deadline(first_compare_edge - 2);
  constexpr unsigned second = control_base - 4;
  constexpr unsigned third = control_base - 8;
  const auto configure = [&](const unsigned csr, const Configuration values) {
    first.timers.write({.address = csr - 1, .value = values.load});
    first.timers.write({.address = csr - 2, .value = values.compare});
    first.timers.write({.address = csr, .value = values.control});
  };
  configure(second, {.load = 9, .compare = 10, .control = enable | compare_irq | prescale});
  first.deadline(6); // Shared countdown is still 2; enabling timer 1 did not reset it.
  configure(third, {.load = 0, .compare = 3, .control = enable | compare_irq});
  first.deadline(4);
  first.timers.advance_clk2(3);
  first.deadline(1);
  first.timers.advance_clk2(1);
  first.deadline(2);
  require(first.timers.interrupt_vectors() == std::vector{compare_vector + 8},
          "Only the unscaled timer admits its independent earliest compare");
  first.timers.acknowledge_interrupt(compare_vector + 8);
  first.deadline(2);
  first.timers.advance_clk2(2);
  first.deadline(first_compare_edge - 8);
  require(first.timers.interrupt_vectors() == std::vector{compare_vector + 4},
          "The later prescaled compare retains the shared phase");
  first.timers.acknowledge_interrupt(compare_vector + 4);
  first.deadline(first_compare_edge - 8);
  for (const auto csr : {control_base, second, third}) {
    first.timers.write({.address = csr, .value = 0});
  }
  first.configure({.load = 3, .compare = 3, .control = enable | compare_irq | prescale});
  first.deadline(4); // Starting the stopped global prescaler reloads its full period.
}

void test_reset_and_requalification() {
  Timer timer(0);
  timer.deadline(std::nullopt);
  timer.timers.observe_synchronized_tio(0, false);
  timer.configure({.load = word_max - 1, .compare = 0, .control = enable | capture | overflow_irq});
  timer.deadline(3);
  timer.timers.set_tio_drive(0, false);
  timer.deadline(1); // The drive is now known, but has not been synchronized yet.
  const auto before = timer.timers.read(timer.csr - 3);
  bool rejected = false;
  try {
    timer.timers.advance_clk2(1);
  } catch (const blofeld::Fault &) { rejected = true; }
  require(rejected && timer.timers.read(timer.csr - 3) == before,
          "A failed input check cannot consume its deadline or update the timer count");
  timer.deadline(1);
  timer.timers.observe_synchronized_tio(0, false);
  timer.deadline(3); // Same level, newly qualified: the old check deadline is gone.
  timer.timers.advance_clk2(1);
  timer.deadline(2);
  timer.timers.observe_synchronized_tio(0, true);
  timer.deadline(std::nullopt); // The actual capture halts the running counter.
  timer.timers.reset();
  timer.deadline(std::nullopt);
  timer.configure({.load = 3, .compare = 5, .control = enable | compare_irq});
  timer.deadline(3);
  timer.timers.advance_clk2(2);
  timer.deadline(1);
  timer.timers.advance_clk2(1);
  timer.deadline(std::nullopt);
  timer.pending(0);
}

void test_large_deadline() {
  Timer timer(2);
  timer.timers.write({.address = prescaler_load, .value = prescaler_max});
  timer.configure({.load = 0, .compare = word_max, .control = enable | compare_irq | prescale});
  constexpr std::uint64_t expected = word_range * (prescaler_max + 1);
  timer.deadline(expected);
  timer.timers.advance_clk2(expected - 1);
  timer.deadline(1);
  require(timer.timers.interrupt_vectors().empty(), "A 45-bit horizon cannot admit an early IRQ");
  timer.timers.advance_clk2(1);
  timer.deadline(std::nullopt);
  require(timer.timers.interrupt_vectors() == std::vector{compare_vector + 8},
          "The final prescaled compare still occurs on its exact literal clock");
}
}

int main() {
  try {
    for (unsigned index = 0; index < 3; ++index) {
      test_free_running(index);
      test_reload_and_live_writes(index);
      test_equal_and_wrap(index);
      test_prescaled(index);
    }
    test_capture_and_unknown_inputs();
    test_shared_prescaler_and_earliest_timer();
    test_reset_and_requalification();
    test_large_deadline();
    std::cout << "Timer native-region boundaries: " << assertions << " assertions passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
