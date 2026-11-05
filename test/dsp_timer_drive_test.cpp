// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_timers.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>

namespace {
constexpr unsigned first_control = 0xffff8f;
constexpr unsigned enable_irq = 7;
constexpr unsigned event_mode = 0x30;
constexpr unsigned capture_mode = 0x60;
constexpr unsigned invert = 0x100;
constexpr unsigned direction = 0x800;
constexpr unsigned data_in = 0x1000;
constexpr unsigned compare_flag = 0x200000;
constexpr std::uint64_t counter_range = 0x1000000;

struct Checks {
  unsigned count = 0;

  void require(const bool condition, const std::string_view message) {
    if (!condition) { throw blofeld::Fault(std::string(message)); }
    ++count;
  }

  void rejects(const auto &operation, const std::string_view diagnostic) {
    try {
      operation();
    } catch (const blofeld::Fault &error) {
      require(std::string_view(error.what()).contains(diagnostic), "Specific fault retained");
      return;
    }
    throw blofeld::Fault("Expected fault: " + std::string(diagnostic));
  }
};

struct Timer {
  blofeld::DspTimers model;
  unsigned index;
  unsigned control;

  explicit Timer(const unsigned pin) : index(pin), control(first_control - (4 * pin)) {}

  void configure(const unsigned mode, const bool inverted) {
    model.set_tio_drive(index, inverted);
    model.observe_synchronized_tio(index, inverted);
    model.write({.address = control - 1, .value = 3});
    model.write({.address = control - 2, .value = 5});
    model.write({.address = control, .value = mode | enable_irq | (inverted ? invert : 0U)});
  }

  void sample(const bool level) {
    model.set_tio_drive(index, level);
    model.observe_synchronized_tio(index, level);
  }

  [[nodiscard]] unsigned count() const { return model.read(control - 3); }
};

void test_external_levels(Checks &checks) {
  constexpr std::array<std::optional<bool>, 3> levels{std::nullopt, false, true};
  for (unsigned index = 0; index < 2; ++index) {
    Timer timer(index);
    for (const auto level : levels) {
      timer.model.set_tio_drive(index, level);
      timer.model.set_tio_drive(index, level);
      const auto pin = timer.model.observe_tio(index);
      checks.require(!pin.output && pin.level == level, "Repeated optional drive is preserved");
      checks.require(timer.count() == 0, "A drive does not clock a disabled timer");
      if (!level) {
        checks.rejects([&] { static_cast<void>(timer.model.read(timer.control)); }, "Unknown");
      } else {
        checks.require(timer.model.read(timer.control, data_in) == (*level ? data_in : 0U),
                       "Literal input level determines DI");
      }
    }
    timer.model.reset();
    timer.model.set_tio_drive(index, true);
    checks.require(timer.model.observe_tio(index).level == true,
                   "Reset retains the external drive and permits the same-level path");
    timer.model.set_tio_drive(index, std::nullopt);
    timer.model.write({.address = timer.control, .value = direction});
    timer.model.set_tio_drive(index, false);
    checks.rejects([&] { timer.model.set_tio_drive(index, true); }, "TIO contention");
    timer.model.set_tio_drive(index, false);
    checks.require(timer.model.observe_tio(index).output &&
                       timer.model.observe_tio(index).level == false,
                   "Rejected opposite drive preserves the prior agreeing output");
  }
  blofeld::DspTimers model;
  for (const auto level : levels) {
    checks.rejects([&] { model.set_tio_drive(2, level); }, "Invalid DSP timer TIO drive index");
  }
}

void test_event_edges(Checks &checks, const unsigned index, const bool inverted) {
  Timer timer(index);
  timer.configure(event_mode, inverted);
  timer.model.set_tio_drive(index, inverted);
  timer.model.advance_clk2(10);
  checks.require(timer.count() == 0 && !timer.model.next_clk2_boundary(),
                 "Duplicate qualified baseline does not count an event or lose qualification");
  timer.model.set_tio_drive(index, !inverted);
  timer.model.set_tio_drive(index, !inverted);
  checks.require(timer.model.next_clk2_boundary() == 1,
                 "Duplicate changed raw level cannot qualify an unobserved event edge");
  checks.rejects([&] { timer.model.advance_clk2(1); }, "Unknown DSP event-counter input");
  checks.require(timer.count() == 0, "Rejected unsynchronized clock leaves count untouched");
  timer.model.observe_synchronized_tio(index, !inverted);
  checks.require(timer.count() == 3, "First selected edge preloads literal three");
  timer.model.set_tio_drive(index, !inverted);
  timer.model.advance_clk2(10);
  checks.require(timer.count() == 3 && !timer.model.next_clk2_boundary(),
                 "Repeated qualified selected level inserts no additional event");
  timer.sample(inverted);
  timer.sample(!inverted);
  checks.require(timer.count() == 4, "Second actual selected edge increments once");
  timer.sample(inverted);
  timer.sample(!inverted);
  checks.require(timer.count() == 5 &&
                     timer.model.read(timer.control, compare_flag) == compare_flag,
                 "Third actual selected edge reaches independently expected compare");
}

void test_capture_qualification(Checks &checks, const unsigned index, const bool inverted) {
  Timer timer(index);
  timer.configure(capture_mode, inverted);
  timer.model.advance_clk2(2);
  checks.require(timer.count() == 0, "Capture register waits for its actual selected input edge");
  timer.model.set_tio_drive(index, inverted);
  checks.require(timer.model.next_clk2_boundary() == counter_range - 4,
                 "Duplicate qualified level retains the independently expected overflow boundary");
  timer.model.set_tio_drive(index, !inverted);
  timer.model.set_tio_drive(index, !inverted);
  checks.require(timer.model.next_clk2_boundary() == 1,
                 "Duplicate unqualified raw level retains the one-clock input check");
  checks.rejects([&] { timer.model.advance_clk2(1); }, "Unknown DSP timer capture input");
  timer.model.observe_synchronized_tio(index, !inverted);
  checks.require(timer.model.read(timer.control - 3) == 4 &&
                     timer.model.read(timer.control, compare_flag) == compare_flag,
                 "Observed selected edge captures four and sets its actual flag");
  timer.model.set_tio_drive(index, !inverted);
  timer.model.advance_clk2(10);
  checks.require(timer.count() == 4 && !timer.model.next_clk2_boundary(),
                 "Duplicate captured level preserves halt and captured count");
}
}

int main() {
  try {
    Checks checks;
    test_external_levels(checks);
    for (unsigned index = 0; index < 2; ++index) {
      for (const bool inverted : {false, true}) {
        test_event_edges(checks, index, inverted);
        test_capture_qualification(checks, index, inverted);
      }
    }
    std::cout << "PASS " << checks.count << " timer drive assertions\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
