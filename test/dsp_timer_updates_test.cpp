// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_timers.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <string>

namespace {
constexpr unsigned first_control = 0xffff8f;
constexpr unsigned enable = 1;
constexpr unsigned interrupt_enables = 6;
constexpr unsigned capture = 0x60;
constexpr unsigned invert = 0x100;
constexpr unsigned reload = 0x200;
constexpr unsigned prescale = 0x8000;
constexpr unsigned overflow = 0x100000;
constexpr unsigned match = 0x200000;
constexpr unsigned flags = overflow | match;
constexpr unsigned first_compare_vector = 0x54;
constexpr unsigned word_max = 0xffffff;
constexpr std::uint64_t word_range = 0x1000000;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void rejects(const auto &operation, const std::string &message) {
  try {
    operation();
  } catch (const blofeld::Fault &error) {
    require(std::string(error.what()).contains("Unqualified active DSP timer"),
            "The unsupported ordered transition retains a specific timer diagnostic");
    return;
  }
  throw blofeld::Fault(message);
}

struct Configuration {
  unsigned load;
  unsigned compare;
  unsigned control;
};

struct Timer {
  blofeld::DspTimers timers;
  unsigned csr;

  explicit Timer(const unsigned index) : csr(first_control - (4 * index)) {}

  void write(const unsigned offset, const unsigned value) {
    timers.write({.address = csr - offset, .value = value});
  }

  [[nodiscard]] unsigned count() const { return timers.read(csr - 3); }

  [[nodiscard]] unsigned status() const { return timers.read(csr, flags); }

  void configure(const Configuration configuration) {
    write(1, configuration.load);
    write(2, configuration.compare);
    write(0, configuration.control | interrupt_enables);
  }
};

void test_combined_disable_and_reconfigure() {
  for (unsigned index = 0; index < 2; ++index) {
    Timer timer(index);
    timer.timers.set_tio_drive(index, true);
    timer.timers.observe_synchronized_tio(index, true);
    timer.configure({.load = word_max - 1, .compare = 7, .control = enable | capture | invert});
    timer.timers.advance_clk2(4);
    timer.timers.set_tio_drive(index, false);
    timer.timers.observe_synchronized_tio(index, false);
    require(timer.count() == 1 && timer.status() == flags,
            "Independent capture input follows one preload, wrap, and a selected falling edge");
    timer.write(0, invert);
    require(timer.count() == 1 && timer.status() == 0 && timer.timers.interrupt_vectors().empty(),
            "An ordered TE-clear/mode-zero write disables before the new mode can clock");
    timer.timers.advance_clk2(word_range);
    require(timer.count() == 1, "Disabled mode changes cannot insert a counter clock");
    timer.configure({.load = 5, .compare = 7, .control = enable});
    timer.timers.advance_clk2(3);
    require(timer.count() == 7 && timer.status() == match,
            "Re-enabling clears and preloads once, then reaches the independent compare");
    rejects([&] { timer.write(0, enable | capture); },
            "A mode change that retains TE remains unsupported");
    require(timer.count() == 7 && timer.status() == match,
            "A rejected enabled mode change preserves the counter and source flag");
    timer.write(0, capture);
    require(timer.count() == 7 && timer.status() == 0,
            "Disabling into capture preserves the previous readable count and clears flags");
    timer.write(0, capture | enable);
    timer.write(2, 6);
    timer.timers.advance_clk2(2);
    timer.timers.set_tio_drive(index, true);
    timer.timers.observe_synchronized_tio(index, true);
    require(timer.count() == 6 && timer.status() == match && timer.timers.read(timer.csr - 2) == 6,
            "Capture ignores a live TCPR write and still measures literal preload/increment edges");
  }
}

void test_free_running_live_compare() {
  for (unsigned index = 0; index < 3; ++index) {
    Timer timer(index);
    timer.configure({.load = 3, .compare = 7, .control = enable});
    timer.timers.advance_clk2(2);
    timer.write(2, 5);
    require(timer.count() == 4 && timer.status() == 0,
            "Live TCPR is stored without an invented match, load or clock");
    timer.timers.advance_clk2(1);
    require(timer.count() == 5 && timer.status() == match,
            "The next actual increment compares against the new TCPR");
    timer.write(2, 4);
    require(timer.count() == 5 && timer.status() == match,
            "Changing TCPR does not acknowledge an existing compare source");
    timer.write(0, enable | interrupt_enables | flags);
    timer.timers.advance_clk2(word_range - 1);
    require(timer.count() == 4 && timer.status() == flags,
            "A new lower compare is reached only after the independently expected wrap");
    timer.write(0, enable | interrupt_enables | flags);
    timer.write(2, 4);
    timer.timers.advance_clk2(1);
    require(timer.count() == 5 && timer.status() == 0,
            "A TCPR write equal to the old count does not retroactively create an event");
    timer.write(0, 0);
    timer.write(1, 3);
    timer.write(0, enable | interrupt_enables);
    timer.write(2, 3);
    timer.timers.advance_clk2(1);
    require(timer.count() == 3 && timer.status() == match,
            "Live TCPR before the first clock participates in the genuine first preload match");
  }
}

void test_pending_reload_survives_compare_and_flag_writes() {
  for (unsigned index = 0; index < 3; ++index) {
    Timer timer(index);
    timer.configure({.load = 3, .compare = 5, .control = enable | reload});
    timer.timers.advance_clk2(3);
    timer.write(2, 7);
    timer.write(0, enable | reload | interrupt_enables | flags);
    require(timer.count() == 5 && timer.status() == 0,
            "TCPR/W1C writes do not replace the counter or a previously requested reload");
    timer.timers.advance_clk2(1);
    require(timer.count() == 3 && timer.status() == 0,
            "The already triggered reload occurs despite a changed comparison value");
    timer.timers.advance_clk2(4);
    require(timer.count() == 7 && timer.status() == match,
            "After reload, only the new compare generates the following event");
    timer.write(2, 3);
    timer.write(0, enable | reload | interrupt_enables | flags);
    timer.timers.advance_clk2(std::numeric_limits<std::uint64_t>::max());
    require(timer.count() == 3 && timer.status() == match,
            "A changed one-clock period handles UINT64_MAX from a prior unequal pending count");
  }
}

void test_equal_compare_write_does_not_request_reload() {
  Timer timer(2);
  timer.configure({.load = 3, .compare = 9, .control = enable | reload});
  timer.timers.advance_clk2(2);
  timer.write(2, 4);
  timer.timers.advance_clk2(1);
  require(timer.count() == 5 && timer.status() == 0,
          "Equality created by TCPR write is not an earlier clocked reload request");
  timer.timers.advance_clk2(word_range - 1);
  require(timer.count() == 4 && timer.status() == flags,
          "The new compare first reload request follows the next full counter wrap");
  timer.write(2, 6);
  timer.timers.acknowledge_interrupt(first_control - timer.csr + first_compare_vector);
  timer.timers.advance_clk2(1);
  require(timer.count() == 3 && timer.status() == overflow,
          "Interrupt acknowledgement clears its flag without consuming a pending reload");
}

void test_ordered_reload_mode_changes_and_retained_failures() {
  Timer timer(2);
  timer.configure({.load = 3, .compare = 7, .control = enable});
  timer.timers.advance_clk2(2);
  timer.write(0, enable | reload | interrupt_enables);
  require(timer.count() == 4 && timer.status() == 0,
          "Setting TRM between compare events does not restart or preload the counter");
  timer.timers.advance_clk2(3);
  require(timer.count() == 7 && timer.status() == match, "TRM applies to the next actual compare");
  rejects([&] { timer.write(0, enable | interrupt_enables); },
          "Changing TRM while a reload is pending needs separate edge-phase qualification");
  timer.write(2, 8);
  rejects([&] { timer.write(0, enable | interrupt_enables); },
          "Changing TCPR cannot disguise the still-pending reload boundary");
  timer.timers.advance_clk2(2);
  require(timer.count() == 4 && timer.status() == match,
          "Rejected TRM changes preserve the exact requested reload and following increment");
  timer.write(0, enable | interrupt_enables | flags);
  timer.timers.advance_clk2(4);
  require(timer.count() == 8 && timer.status() == match,
          "Cleared TRM at a safe boundary leaves the next compare free-running");
  rejects([&] { timer.write(0, enable | reload | interrupt_enables); },
          "Enabling TRM immediately at a previous comparison remains explicitly unqualified");
  timer.write(2, 9);
  timer.write(0, enable | reload | interrupt_enables);
  rejects([&] { timer.write(0, enable | reload | prescale | interrupt_enables); },
          "Enabled PCE changes remain unsupported");
  timer.write(0, 0);
  require(timer.status() == 0, "Disable clears pending sources and internal reload state");
  timer.write(0, enable | interrupt_enables);
  timer.timers.advance_clk2(1);
  require(timer.count() == 3 && timer.status() == 0,
          "Fresh enable starts with one load rather than a stale internal reload request");
}

// Literal chapter-11 clock events, independent of the component's modular batches.
struct Expected {
  unsigned load;
  unsigned compare;
  bool reload;
  unsigned count = 0;
  unsigned status = 0;
  bool first = true;
  bool pending = false;

  void clock() {
    if (first || pending) {
      count = load;
      first = false;
      pending = false;
    } else {
      if (count == word_max) { status |= overflow; }
      count = (count + 1) & word_max;
    }
    if (count == compare) {
      status |= match;
      pending = reload;
    }
  }
};

unsigned live_update_sequence(const unsigned index, const Configuration configuration) {
  constexpr std::array<unsigned, 7> batches{1, 2, 3, 5, 8, 31, 257};
  Timer timer(index);
  Expected expected{.load = configuration.load,
                    .compare = configuration.compare,
                    .reload = (configuration.control & reload) != 0};
  auto control = configuration.control | interrupt_enables;
  timer.configure(configuration);
  for (const auto batch : batches) {
    timer.write(2, (batch & 1U) != 0 ? configuration.compare : word_max - configuration.compare);
    expected.compare = (batch & 1U) != 0 ? configuration.compare : word_max - configuration.compare;
    timer.write(1, word_max - expected.compare);
    expected.load = word_max - expected.compare;
    if (!expected.pending && (expected.first || expected.count != expected.compare)) {
      expected.reload = !expected.reload;
      control ^= reload;
      timer.write(0, control);
    }
    require(timer.count() == expected.count && timer.status() == expected.status,
            "Live compare/load/TRM writes add no clock, match or counter replacement");
    for (unsigned edge = 0; edge < batch; ++edge) { expected.clock(); }
    timer.timers.advance_clk2(batch);
    require(timer.count() == expected.count && timer.status() == expected.status,
            "Bulk clocks with live updates agree with independently ordered literal edges");
    const auto clear = (batch & 1U) != 0 ? match : overflow;
    timer.write(0, control | clear);
    expected.status &= ~clear;
  }
  return static_cast<unsigned>(batches.size());
}

void test_fixed_live_update_sequences() {
  constexpr std::array<unsigned, 8> values{0, 1, 3, 5, 7, word_max - 2, word_max - 1, word_max};
  unsigned checked = 0;
  for (unsigned index = 0; index < 3; ++index) {
    for (const auto initial : values) {
      for (const auto compare : values) {
        for (const bool reload_enabled : {false, true}) {
          checked += live_update_sequence(index,
                                          {.load = initial,
                                           .compare = compare,
                                           .control = enable | (reload_enabled ? reload : 0U)});
        }
      }
    }
  }
  constexpr unsigned expected_checks = 2688;
  require(checked == expected_checks, "Every fixed index/load/compare/TRM/batch case executes");
  std::cout << checked << " fixed live-update/edge comparisons passed\n";
}
}

int main() {
  try {
    test_combined_disable_and_reconfigure();
    test_free_running_live_compare();
    test_pending_reload_survives_compare_and_flag_writes();
    test_equal_compare_write_does_not_request_reload();
    test_ordered_reload_mode_changes_and_retained_failures();
    test_fixed_live_update_sequences();
    std::cout << "Ordered timer disable, live TCPR/TRM and pending-reload checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
