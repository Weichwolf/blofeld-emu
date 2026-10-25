// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_timers.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace {
constexpr std::array<unsigned, 3> controls{0xffff8f, 0xffff8b, 0xffff87};
constexpr unsigned tplr = 0xffff83;
constexpr unsigned tpcr = 0xffff82;
constexpr unsigned te = 1;
constexpr unsigned irq_enables = 6;
constexpr unsigned inv = 0x100;
constexpr unsigned trm = 0x200;
constexpr unsigned dir = 0x800;
constexpr unsigned di = 0x1000;
constexpr unsigned dout = 0x2000;
constexpr unsigned pce = 0x8000;
constexpr unsigned tof = 0x100000;
constexpr unsigned tcf = 0x200000;
constexpr unsigned flags = tof | tcf;
constexpr unsigned word_max = 0xffffff;
constexpr std::uint64_t word_range = 0x1000000;
constexpr unsigned prescaler_max = 0x1fffff;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void reject(const auto &operation, const std::string &message) {
  try {
    operation();
  } catch (const blofeld::Fault &) { return; }
  throw blofeld::Fault(message);
}

struct Configuration {
  unsigned index;
  unsigned load;
  unsigned compare;
  unsigned control;
};

void configure(blofeld::DspTimers &timers, const Configuration configuration) {
  const auto address = controls.at(configuration.index);
  timers.write({.address = address - 1, .value = configuration.load});
  timers.write({.address = address - 2, .value = configuration.compare});
  timers.write({.address = address, .value = configuration.control});
}

void test_reset_permissions_and_tio_polarity() {
  blofeld::DspTimers timers;
  for (const auto csr : controls) {
    require(timers.read(csr - 3) == 0 && timers.read(csr, te) == 0,
            "NXP Appendix C resets TCR and TE to zero");
    reject([&timers, csr] { static_cast<void>(timers.read(csr - 2)); },
           "TCPR reset is undefined rather than a zero compare value");
    reject([&timers, csr] { static_cast<void>(timers.read(csr - 1)); }, "TLR is write-only");
    reject([&timers, csr] { timers.write({.address = csr - 3}); }, "TCR is read-only");
  }
  require(timers.read(tplr) == 0 && timers.read(tpcr) == 0, "Prescaler reset fields are zero");
  reject([&timers] { timers.write({.address = tpcr}); }, "TPCR is read-only");
  for (unsigned index = 0; index < 2; ++index) {
    const auto csr = controls.at(index);
    reject([&timers, csr] { static_cast<void>(timers.read(csr)); },
           "A released TIO input cannot be replaced with the DI reset-latch value");
    timers.set_tio_drive(index, false);
    require(timers.read(csr) == 0, "Observed low input produces DI=0");
    timers.set_tio_drive(index, true);
    require(timers.read(csr, di) == di, "Observed high input produces DI=1");
    timers.write({.address = csr, .value = inv | di});
    require(timers.read(csr, di) == 0,
            "INV inverts input; writing read-only DI cannot force it high");
    timers.set_tio_drive(index, false);
    require(timers.read(csr, di) == di, "Inverted low input produces DI=1");
    timers.set_tio_drive(index, std::nullopt);
    timers.write({.address = csr, .value = dir});
    require(timers.observe_tio(index).output && timers.observe_tio(index).level == false,
            "DO reset is defined zero and DIR enables its GPIO output independently of TE");
    timers.write({.address = csr, .value = dir | dout | inv});
    require(timers.observe_tio(index).level == false && timers.read(csr, di) == di,
            "INV applies on output and again on DI readback, leaving logical DO as readback");
    reject([&timers, index] { timers.set_tio_drive(index, true); }, "Opposed TIO drivers fail");
    timers.set_tio_drive(index, false);
    reject([&timers, csr] { timers.write({.address = csr, .value = dir | dout}); },
           "A CPU output-polarity change that opposes an external drive fails");
    require(timers.observe_tio(index).level == false, "Rejected output changes preserve state");
    timers.reset();
    require(!timers.observe_tio(index).output && timers.observe_tio(index).level == false,
            "Reset returns TIO to input while retaining the external board observation");
    timers.set_tio_drive(index, std::nullopt);
  }
  require(timers.read(controls.at(2)) == 0, "Timer 2 has no external TIO input");
  reject([&timers] { timers.set_tio_drive(2, true); }, "Timer 2 cannot expose a phantom pin");
  reject([&timers] { timers.write({.address = controls.at(2), .value = dir}); },
         "Timer 2 pin-control use is unqualified");
}

void test_preload_compare_and_reload_edges() {
  blofeld::DspTimers timers;
  const auto csr = controls.at(2);
  configure(timers, {.index = 2, .load = 3, .compare = 5, .control = te | trm});
  timers.advance_clk2(0);
  require(timers.read(csr - 3) == 0, "TE clears TCR but does not preload before a clock");
  timers.advance_clk2(1);
  require(timers.read(csr - 3) == 3 && timers.read(csr, flags) == 0,
          "First clock loads N=3 rather than incrementing a fabricated zero preload");
  timers.advance_clk2(1);
  require(timers.read(csr - 3) == 4 && timers.read(csr, flags) == 0,
          "Second clock increments to four");
  timers.advance_clk2(1);
  require(timers.read(csr - 3) == 5 && timers.read(csr, flags) == tcf,
          "Compare M=5 occurs after exactly M-N+1=3 events");
  timers.write({.address = csr, .value = te | trm | tcf});
  require(timers.read(csr - 3) == 5 && timers.read(csr, flags) == 0,
          "Clearing a flag with TE already set does not restart the counter");
  timers.advance_clk2(1);
  require(timers.read(csr - 3) == 3 && timers.read(csr, flags) == 0,
          "Reload follows compare on the next clock");
  timers.advance_clk2(2);
  require(timers.read(csr - 3) == 5 && timers.read(csr, flags) == tcf,
          "Reload period is three clocks");
  timers.advance_clk2(std::numeric_limits<std::uint64_t>::max());
  require(timers.read(csr - 3) == 5 && timers.read(csr, flags) == tcf,
          "UINT64_MAX is divisible by the three-clock period without overflowing host arithmetic");
  timers.write({.address = csr - 1, .value = 4});
  timers.write({.address = csr, .value = te | trm | tcf});
  require(timers.read(csr - 3) == 5, "A live TLR write does not immediately overwrite TCR");
  timers.advance_clk2(1);
  require(timers.read(csr - 3) == 4 && timers.read(csr, flags) == 0,
          "The new live preload is used at the next scheduled reload");
  timers.advance_clk2(1);
  require(timers.read(csr - 3) == 5 && timers.read(csr, flags) == tcf,
          "The updated preload establishes a two-clock reload period");
  timers.write({.address = csr, .value = trm});
  timers.advance_clk2(word_range);
  require(timers.read(csr - 3) == 5 && timers.read(csr, flags) == 0,
          "Disabling freezes the count and clears both flags");
}

void test_overflow_flags_and_interrupt_acknowledgement() {
  blofeld::DspTimers timers;
  const auto csr = controls.at(0);
  constexpr unsigned initial = 0xfffffe;
  constexpr unsigned compare_irq = 0x54;
  constexpr unsigned overflow_irq = 0x56;
  configure(timers,
            {.index = 0, .load = initial, .compare = word_max, .control = te | irq_enables});
  timers.advance_clk2(1);
  require(timers.read(csr - 3) == initial && timers.interrupt_vectors().empty(),
          "Preload is not overflow");
  timers.advance_clk2(1);
  require(timers.read(csr, flags) == tcf, "Reaching the maximum compare first sets only TCF");
  timers.advance_clk2(1);
  require(timers.read(csr - 3) == 0 && timers.read(csr, flags) == flags &&
              timers.interrupt_vectors() == std::vector<std::uint32_t>{overflow_irq, compare_irq},
          "Wrap sets TOF and source enumeration puts overflow before compare");
  timers.write({.address = csr, .value = te | irq_enables});
  require(timers.read(csr, flags) == flags, "Zero writes do not clear W1C flags");
  timers.write({.address = csr, .value = te | irq_enables | tcf});
  require(timers.read(csr, flags) == tof, "Writing only TCF=1 leaves TOF set");
  timers.acknowledge_interrupt(overflow_irq);
  require(timers.interrupt_vectors().empty() && timers.read(csr, flags) == 0,
          "Servicing the enabled overflow source clears its flag");
  reject([&timers] { timers.acknowledge_interrupt(overflow_irq); },
         "Acknowledgement requires a pending source");
  timers.write({.address = csr});
  configure(timers,
            {.index = 0, .load = word_max, .compare = word_max, .control = te | irq_enables});
  timers.advance_clk2(2);
  require(timers.read(csr, flags) == flags,
          "Equal preload/compare followed by wrap sets both flags");
  timers.acknowledge_interrupt(compare_irq);
  require(timers.read(csr, flags) == tof, "Compare service cannot clear overflow");
  timers.write({.address = csr});
  require(timers.read(csr, flags) == 0 && timers.interrupt_vectors().empty(),
          "Disable clears pending sources");
}

void test_prescaler_phase_sharing_and_new_load() {
  blofeld::DspTimers timers;
  const auto first_csr = controls.at(0);
  const auto direct_csr = controls.at(1);
  const auto second_csr = controls.at(2);
  configure(timers, {.index = 1, .load = 0, .compare = 15, .control = te});
  timers.write({.address = tplr, .value = 2});
  configure(timers, {.index = 0, .load = 10, .compare = 12, .control = te | pce});
  require(timers.read(tpcr) == 2, "First prescaled user loads the shared prescaler");
  timers.advance_clk2(1);
  require(timers.read(tpcr) == 1 && timers.read(first_csr - 3) == 0,
          "No early timer preload before prescaler pulse");
  configure(timers, {.index = 2, .load = 7, .compare = 9, .control = te | pce});
  require(timers.read(tpcr) == 1, "A second enabled user cannot restart the shared phase");
  timers.advance_clk2(1);
  require(timers.read(tpcr) == 0 && timers.read(second_csr - 3) == 0,
          "Preload+1 input events define the divider");
  timers.advance_clk2(1);
  require(timers.read(tpcr) == 2 && timers.read(first_csr - 3) == 10 &&
              timers.read(second_csr - 3) == 7 && timers.read(direct_csr - 3) == 2,
          "One shared pulse preloads both users while the direct timer sees all three input edges");
  timers.advance_clk2(6);
  require(timers.read(first_csr - 3) == 12 && timers.read(second_csr - 3) == 9 &&
              timers.read(first_csr, tcf) == tcf && timers.read(second_csr, tcf) == tcf &&
              timers.read(direct_csr - 3) == 8,
          "Three prescaler pulses produce the two expected compare events");
  timers.write({.address = tplr});
  timers.advance_clk2(2);
  require(timers.read(tpcr) == 0 && timers.read(first_csr - 3) == 12,
          "Changing preload does not truncate the already running prescaler period");
  timers.advance_clk2(1);
  require(timers.read(tpcr) == 0 && timers.read(first_csr - 3) == 13,
          "New preload zero takes effect on reload, then divides by one");
  timers.write({.address = first_csr});
  timers.write({.address = second_csr});
  timers.advance_clk2(std::numeric_limits<std::uint64_t>::max());
  require(timers.read(tpcr) == 0, "An unused prescaler cannot accumulate elapsed input clocks");
  timers.write({.address = tplr, .value = prescaler_max});
  timers.write({.address = first_csr, .value = te | pce});
  timers.advance_clk2(prescaler_max);
  require(timers.read(tpcr) == 0 && timers.read(first_csr - 3) == 0,
          "21-bit maximum preload does not pulse early");
  timers.advance_clk2(1);
  require(timers.read(tpcr) == prescaler_max && timers.read(first_csr - 3) == 10,
          "Maximum prescaler period is exactly 2^21 input clocks");
}

void test_unknown_clock_inputs_rejection_and_reset() {
  blofeld::DspTimers timers;
  const auto csr = controls.at(0);
  timers.write({.address = csr, .value = te | pce});
  timers.advance_clk2(0);
  reject([&timers] { timers.advance_clk2(1); }, "A first clock cannot invent reset-undefined TLR");
  require(timers.read(csr - 3) == 0 && timers.read(tpcr) == 0,
          "Rejected batch leaves counter/prescaler unchanged");
  timers.write({.address = csr});
  timers.write({.address = csr - 1, .value = 5});
  timers.write({.address = csr, .value = te});
  reject([&timers] { timers.advance_clk2(1); },
         "An unknown compare cannot silently suppress events");
  timers.write({.address = csr});
  configure(timers, {.index = 0, .load = 5, .compare = 7, .control = te});
  timers.advance_clk2(1);
  timers.write({.address = controls.at(1), .value = te});
  reject([&timers] { timers.advance_clk2(5); },
         "Unknown second timer rejects the entire shared-clock batch");
  require(timers.read(csr - 3) == 5,
          "Earlier timers cannot partially advance before another timer rejects time");
  timers.write({.address = csr - 2, .value = 6});
  require(timers.read(csr - 3) == 5 && timers.read(csr - 2) == 6,
          "Live compare updates change TCPR without advancing the counter");
  reject([&timers] { timers.write({.address = csr, .value = te | pce}); },
         "Active prescaler-source changes are unsafe");
  constexpr unsigned external_source = 0x200000;
  reject([&timers] { timers.write({.address = tplr, .value = external_source}); },
         "External prescaler timing is unsupported");
  timers.reset();
  require(timers.read(csr - 3) == 0 && timers.read(tplr) == 0 && timers.read(tpcr) == 0,
          "Reset clears counter, prescaler and control state");
  reject([&timers] { static_cast<void>(timers.read(csr - 2)); },
         "Reset re-establishes undefined compare content");
}

void test_reload_wrap_and_equal_zero_compare() {
  blofeld::DspTimers timers;
  const auto csr = controls.at(2);
  configure(timers, {.index = 2, .load = 0, .compare = word_max, .control = te | trm});
  timers.advance_clk2(word_range + 1);
  require(timers.read(csr - 3) == 0 && timers.read(csr, flags) == tcf,
          "Reload after maximum compare loads zero without an increment-overflow event");
  timers.write({.address = csr, .value = trm});
  configure(timers, {.index = 2, .load = 0, .compare = 0, .control = te | trm});
  timers.advance_clk2(1);
  require(timers.read(csr - 3) == 0 && timers.read(csr, flags) == tcf,
          "N=M=0 requests compare after M-N+1=1 event per the manual");
  timers.write({.address = csr, .value = te | trm | tcf});
  timers.advance_clk2(std::numeric_limits<std::uint64_t>::max());
  require(timers.read(csr - 3) == 0 && timers.read(csr, flags) == tcf,
          "One-clock reload period handles UINT64_MAX without looping or overflow");
}

// Literal chapter-11 edge sequence, independent of the component's bulk arithmetic.
struct ExpectedCounter {
  unsigned load;
  unsigned compare;
  bool reload;
  unsigned count = 0;
  unsigned status = 0;
  bool first_edge = true;
  bool reload_next = false;

  void clock() {
    if (first_edge || reload_next) {
      count = load;
      first_edge = false;
      reload_next = false;
    } else {
      if (count == word_max) { status |= tof; }
      count = (count + 1) & word_max;
    }
    if (count == compare) {
      status |= tcf;
      reload_next = reload;
    }
  }
};

void test_bulk_clock_against_independent_edge_sequence() {
  struct Scenario {
    unsigned load;
    unsigned compare;
  };

  constexpr std::array<Scenario, 8> scenarios{{{.load = 0, .compare = 0},
                                               {.load = 0, .compare = 3},
                                               {.load = 3, .compare = 5},
                                               {.load = 5, .compare = 3},
                                               {.load = word_max, .compare = 0},
                                               {.load = word_max - 1, .compare = word_max},
                                               {.load = word_max, .compare = word_max},
                                               {.load = word_max - 2, .compare = 2}}};
  constexpr std::array<unsigned, 7> batches{1, 2, 3, 5, 8, 31, 257};
  const auto csr = controls.at(2);
  for (const auto &scenario : scenarios) {
    for (const bool reload : {false, true}) {
      blofeld::DspTimers timers;
      ExpectedCounter expected{
          .load = scenario.load, .compare = scenario.compare, .reload = reload};
      const auto control = te | (reload ? trm : 0U);
      configure(
          timers,
          {.index = 2, .load = scenario.load, .compare = scenario.compare, .control = control});
      for (const auto batch : batches) {
        for (unsigned edge = 0; edge < batch; ++edge) { expected.clock(); }
        timers.advance_clk2(batch);
        require(timers.read(csr - 3) == expected.count &&
                    timers.read(csr, flags) == expected.status,
                "Bulk clocks match literal preload/increment/compare edges across wrap and reload");
        const auto clear = (batch & 1U) != 0 ? tcf : tof;
        timers.write({.address = csr, .value = control | clear});
        expected.status &= ~clear;
        expected.load ^= 1U;
        timers.write({.address = csr - 1, .value = expected.load});
        require(timers.read(csr - 3) == expected.count &&
                    timers.read(csr, flags) == expected.status,
                "W1C and live preload writes do not insert an extra clock or counter load");
      }
    }
  }
}
}

int main() {
  try {
    test_reset_permissions_and_tio_polarity();
    test_preload_compare_and_reload_edges();
    test_overflow_flags_and_interrupt_acknowledgement();
    test_prescaler_phase_sharing_and_new_load();
    test_unknown_clock_inputs_rejection_and_reset();
    test_reload_wrap_and_equal_zero_compare();
    test_bulk_clock_against_independent_edge_sequence();
    std::cout << "DSP mode-zero timer clocks, GPIO, W1C, prescaler and source requests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
