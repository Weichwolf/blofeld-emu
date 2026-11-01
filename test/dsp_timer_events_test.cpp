// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_interrupts.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "timer_native_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <string_view>

namespace {
using blofeld::DspTimers;
using blofeld::test::timer_native::require;
constexpr unsigned event_mode = 0x30, capture_mode = 0x60;
constexpr unsigned enable = 1, interrupt_enables = 6, invert = 0x100, reload = 0x200;
constexpr unsigned direction = 0x800, input = 0x1000, data_out = 0x2000, prescale = 0x8000;
constexpr unsigned overflow = 0x100000, match = 0x200000, flags = overflow | match;
constexpr unsigned first_control = 0xffff8f, prescaler_load = 0xffff83, prescaler_count = 0xffff82;
constexpr unsigned maximum = 0xffffff, first_compare_vector = 0x54;
constexpr unsigned peripheral_priority = 0xfffffe, timer_priority = 0x100;
constexpr unsigned ordinary_status = 0xc00000;
constexpr unsigned edges_per_case = 40;
constexpr auto maximum_edges = std::numeric_limits<std::uint64_t>::max();

void rejects(const auto &operation, const std::string_view text) {
  try {
    operation();
  } catch (const blofeld::Fault &error) {
    require(std::string_view(error.what()).contains(text),
            "Rejected event-counter operation must identify its actual qualification boundary");
    return;
  }
  throw blofeld::Fault("Unqualified event-counter operation succeeded");
}

struct Configuration {
  unsigned load;
  unsigned compare;
  unsigned control;
};

struct Fixture {
  DspTimers timers;
  unsigned index;
  unsigned csr;

  explicit Fixture(const unsigned selected)
      : index(selected), csr(first_control - (4 * selected)) {}

  void configure(const Configuration setup) {
    timers.write({.address = csr - 1, .value = setup.load});
    timers.write({.address = csr - 2, .value = setup.compare});
    timers.write({.address = csr, .value = setup.control});
  }

  void sample(const bool level) {
    timers.set_tio_drive(index, level);
    timers.observe_synchronized_tio(index, level);
  }

  [[nodiscard]] unsigned count() const { return timers.read(csr - 3); }

  [[nodiscard]] unsigned status() const { return timers.read(csr, flags); }
};

// Independent literal edge transitions from UM11.3.1/11.3.1.4, without bulk formulas.
struct Expected {
  unsigned load;
  unsigned compare;
  bool repeats;
  unsigned count = 0;
  unsigned status = 0;
  bool first = true;
  bool pending = false;

  void edge() {
    if (first || pending) {
      count = load;
    } else if (count == maximum) {
      count = 0;
      status |= overflow;
    } else {
      ++count;
    }
    first = false;
    pending = repeats && count == compare;
    if (count == compare) { status |= match; }
  }
};

void literal_external_case(const unsigned index, const Configuration setup) {
  Fixture actual(index);
  const bool inverted = (setup.control & invert) != 0;
  actual.sample(inverted);
  actual.configure(setup);
  Expected expected{
      .load = setup.load, .compare = setup.compare, .repeats = (setup.control & reload) != 0};
  for (unsigned edge = 0; edge < edges_per_case; ++edge) {
    actual.timers.advance_clk2(maximum_edges);
    require(actual.count() == expected.count && actual.status() == expected.status,
            "Internal clocks cannot become external events, even in a UINT64_MAX batch");
    actual.sample(!inverted);
    expected.edge();
    actual.sample(!inverted);
    actual.sample(inverted);
    require(actual.count() == expected.count && actual.status() == expected.status,
            "Only the selected synchronized transition preloads/increments/compares");
    const auto clear = (edge % 2 == 0) ? match : overflow;
    actual.timers.write({.address = actual.csr, .value = setup.control | clear});
    expected.status &= ~clear;
  }
  const auto original_count = actual.count();
  const auto original_status = actual.status();
  auto saved = actual.timers;
  saved.observe_synchronized_tio(index, !inverted);
  expected.edge();
  require(saved.read(actual.csr - 3) == expected.count &&
              saved.read(actual.csr, flags) == expected.status,
          "Copied external phase, first-load and pending-reload state retain literal edges");
  require(actual.count() == original_count && actual.status() == original_status,
          "Advancing the copied component leaves the original counter intact");
}

void literal_external_sequences() {
  constexpr std::array pairs{std::array{0U, 0U},
                             std::array{0U, 1U},
                             std::array{1U, 0U},
                             std::array{maximum, 0U},
                             std::array{maximum - 1, 1U},
                             std::array{2U, 4U},
                             std::array{4U, 2U},
                             std::array{maximum, maximum}};
  for (unsigned index = 0; index < 2; ++index) {
    for (const bool inverted : {false, true}) {
      for (const bool repeats : {false, true}) {
        for (const auto pair : pairs) {
          const unsigned control = event_mode | enable | interrupt_enables |
                                   (inverted ? invert : 0) | (repeats ? reload : 0);
          literal_external_case(index, {.load = pair[0], .compare = pair[1], .control = control});
        }
      }
    }
  }
}

void shared_prescaler_and_limits() {
  DspTimers timers;
  timers.write({.address = prescaler_load, .value = 3});
  for (unsigned index = 0; index < 3; ++index) {
    const unsigned csr = first_control - (4 * index);
    timers.write({.address = csr - 1, .value = 10 + index});
    timers.write({.address = csr - 2, .value = 12 + index});
    timers.write({.address = csr,
                  .value = event_mode | enable | prescale | reload | interrupt_enables |
                           (index < 2 ? invert : 0)});
  }
  timers.advance_clk2(3);
  require(timers.read(first_control - 3) == 0 && timers.read(prescaler_count) == 0,
          "The first prescaler period must finish before the first event-counter preload");
  timers.observe_synchronized_tio(0, false);
  timers.observe_synchronized_tio(0, true);
  require(timers.read(first_control - 3) == 0, "PCE selects pulses independently of TIO edges");
  timers.advance_clk2(1);
  timers.advance_clk2(8);
  for (unsigned index = 0; index < 3; ++index) {
    const unsigned csr = first_control - (4 * index);
    require(timers.read(csr - 3) == 12 + index && timers.read(csr, flags) == match,
            "All three event counters share one phase; INV cannot invert prescaler pulses");
    timers.acknowledge_interrupt(first_compare_vector + (4 * index));
  }
  timers.advance_clk2(4);
  timers.write({.address = prescaler_load, .value = 0});
  timers.advance_clk2(3);
  require(timers.read(first_control - 3) == 10 && timers.read(prescaler_count) == 0,
          "Acknowledgement retains reload; a live prescaler load cannot shorten the old period");
  timers.advance_clk2(2);
  require(timers.read(first_control - 3) == 12 && timers.read(first_control, flags) == match,
          "The replacement one-clock period applies only after the retained reload pulse");
  Fixture large(2);
  large.configure({.load = 0, .compare = 0, .control = event_mode | enable | prescale | reload});
  large.timers.advance_clk2(maximum_edges);
  require(large.count() == 0 && large.status() == match && large.timers.read(prescaler_count) == 0,
          "UINT64_MAX prescaled events retain equal-zero reload without a false overflow");
}

void missing_registers_do_not_consume_edges() {
  for (const bool missing_load : {false, true}) {
    Fixture actual(0);
    actual.sample(false);
    actual.timers.write({.address = actual.csr - (missing_load ? 2U : 1U), .value = 7});
    actual.timers.write({.address = actual.csr, .value = event_mode | enable});
    rejects([&] { actual.sample(true); },
            missing_load ? "Unknown DSP timer load" : "Unknown DSP timer compare");
    require(actual.count() == 0 && actual.status() == 0,
            "Missing reset-undefined registers cannot produce a preload or completion");
    actual.timers.write({.address = actual.csr - (missing_load ? 1U : 2U), .value = 7});
    actual.sample(true);
    require(actual.count() == 7 && actual.status() == match,
            "A rejected edge retains the prior synchronizer state for explicit qualified retry");
  }
}

void input_qualification_and_atomic_batches() {
  Fixture actual(1);
  actual.configure({.load = 7, .compare = 8, .control = event_mode | enable});
  actual.timers.write({.address = prescaler_load, .value = 3});
  actual.timers.write({.address = first_control - 1, .value = 5});
  actual.timers.write({.address = first_control - 2, .value = 9});
  actual.timers.write({.address = first_control, .value = enable | prescale});
  actual.timers.advance_clk2(0);
  rejects([&] { actual.timers.advance_clk2(4); }, "Unknown DSP event-counter input");
  require(actual.count() == 0 && actual.timers.read(first_control - 3) == 0 &&
              actual.timers.read(prescaler_count) == 3,
          "Unknown external input rejects the whole batch including another timer/prescaler");
  rejects([&] { actual.sample(true); }, "Unknown previous DSP synchronized TIO");
  actual.timers.write({.address = actual.csr, .value = event_mode});
  actual.sample(false);
  actual.timers.write({.address = actual.csr, .value = event_mode | enable});
  actual.timers.set_tio_drive(1, true);
  rejects([&] { actual.timers.advance_clk2(4); }, "Unknown DSP event-counter input");
  actual.timers.observe_synchronized_tio(1, false);
  actual.timers.advance_clk2(4);
  require(actual.count() == 0 && actual.timers.read(first_control - 3) == 5,
          "A qualified delayed synchronizer level lets internal users proceed without an edge");
  actual.timers.observe_synchronized_tio(1, true);
  require(actual.count() == 7 && actual.status() == 0,
          "Raw-pad updates alone cannot consume the first event-counter preload");
  actual.timers.reset();
  require(actual.count() == 0 && actual.status() == 0 && actual.timers.observe_tio(1).level == true,
          "Reset clears mode/count/flags while retaining the independently supplied pad level");
}

void pin_modes_and_rejections() {
  Fixture actual(0);
  actual.sample(false);
  constexpr unsigned control = event_mode | enable | direction | data_out;
  actual.configure({.load = 7, .compare = 9, .control = control});
  const auto pin = actual.timers.observe_tio(0);
  require(!pin.output && pin.level == false && actual.timers.read(actual.csr, input) == 0,
          "Mode-three TIO is an input; stored GPIO DIR/DO cannot drive or oppose it");
  actual.sample(true);
  rejects([&] { actual.timers.write({.address = actual.csr, .value = control | invert}); },
          "event polarity change");
  rejects([&] { actual.timers.write({.address = actual.csr, .value = enable}); },
          "active DSP timer mode change");
  actual.sample(false);
  rejects([&] { actual.timers.write({.address = actual.csr, .value = direction | data_out}); },
          "TIO contention");
  require(actual.count() == 7 && !actual.timers.observe_tio(0).output,
          "A rejected GPIO handover retains the actual event-counter mode and count");
  actual.timers.write({.address = actual.csr, .value = event_mode});
  actual.timers.write({.address = actual.csr, .value = capture_mode});
  require(actual.count() == 7, "Switching from a disabled event counter retains capture readout");
  actual.timers.write({.address = actual.csr, .value = capture_mode | enable});
  actual.timers.advance_clk2(4);
  actual.sample(true);
  require(actual.count() == 10 && actual.status() == match,
          "A separately rearmed capture retains ordinary preload/edge semantics after mode three");
  Fixture absent(2);
  rejects([&] { absent.configure({.load = 0, .compare = 1, .control = event_mode | enable}); },
          "Unsupported DSP timer control");
  rejects([&] { actual.timers.observe_synchronized_tio(2, false); }, "no synchronized TIO");
}

void native_event_case(const unsigned index) {
  using namespace blofeld;
  using namespace test::dma_test;
  namespace native = test::timer_native;
  const unsigned csr = first_control - (4 * index);
  const unsigned compare_vector = first_compare_vector + (4 * index);
  NativeProgram program;
  for (const auto opcode : {native::move_sr, native::move_r2, native::move_r3, native::move_r7}) {
    program.word(opcode);
    program.word(opcode == native::move_sr ? ordinary_status : 0);
  }
  program.write({.space = DspSpace::x, .address = peripheral_priority, .value = timer_priority});
  program.write({.space = DspSpace::x, .address = csr - 1, .value = maximum});
  program.write({.space = DspSpace::x, .address = csr - 2, .value = 0});
  program.write(
      {.space = DspSpace::x, .address = csr, .value = event_mode | enable | interrupt_enables});
  const auto foreground = program.pc();
  // FM table2-7 resumes the normal stream after the two fast-vector words.
  // The explicit NOP also releases the core's conservative post-vector hold;
  // this untimed fixture does not qualify physical interrupt arbitration latency.
  program.word(0);
  program.word(native::foreground_move);
  const auto complete = program.pc();
  OwnDspMachine machine(native_clocks);
  reset_native(machine, program);
  native::install_vector(machine, compare_vector, native::short_r3);
  native::install_vector(machine, compare_vector + 2, native::short_r2);
  native::sample(machine.bus().timers(), index, false);
  for (unsigned instruction = 0; instruction < 8; ++instruction) { step_native(machine); }
  require(machine.cpu().registers().pc == foreground &&
              machine.cpu().progress().instructions == 8 &&
              machine.cpu().progress().estimated_cycles == 16,
          "Eight literal native setup instructions retire before any external counter edge");
  for (const bool level : {true, false, true}) {
    native::sample(machine.bus().timers(), index, level);
  }
  require(machine.bus().timers().read(csr, flags) == flags,
          "Independent preload-ffffff then wrap-zero requests both real timer sources");
  const auto saved = machine.checkpoint();
  std::optional<dsp56300::Registers> completed;
  std::optional<dsp56300::Progress> progress;
  for (unsigned replay = 0; replay < 4; ++replay) {
    OwnDspMachine fresh(native_clocks);
    auto &owner = replay == 3 ? fresh : machine;
    owner.restore(saved);
    auto expected = owner.cpu().registers();
    const auto before = owner.cpu().progress();
    const auto ticks = owner.clock().elapsed_ticks();
    native::service(owner, {.vector = compare_vector + 2}, {.address = foreground, .opcode = 0});
    expected.r[2] = 7;
    require(
        owner.cpu().registers() == expected && owner.bus().timers().read(csr, flags) == match,
        "Native overflow service changes only R2 and retains the independently pending compare");
    require(!owner.admit_interrupt({.address = foreground, .opcode = 0}) &&
                owner.bus().interrupt_request(0) == DspInterruptRequest{.vector = compare_vector},
            "The conservative post-vector admission hold cannot consume the pending compare");
    step_native(owner);
    ++expected.pc;
    require(owner.cpu().registers() == expected && owner.bus().timers().read(csr, flags) == match,
            "The actual foreground NOP preserves both the compare event and all data registers");
    native::service(owner,
                    {.vector = compare_vector},
                    {.address = foreground + 1, .opcode = native::foreground_move});
    expected.r[3] = 8;
    require(owner.cpu().registers() == expected && owner.bus().timers().read(csr, flags) == 0,
            "The retained compare executes its own real vector and clears only its source");
    step_native(owner);
    expected.pc = complete;
    expected.r[7] = 9;
    require(owner.cpu().registers() == expected && owner.bus().timers().read(csr - 3) == 0 &&
                owner.cpu().progress().instructions == before.instructions + 6 &&
                owner.cpu().progress().estimated_cycles == before.estimated_cycles + 6 &&
                owner.clock().elapsed_ticks() == ticks,
            "Six real vector/foreground instructions leave the external count unchanged");
    if (completed && progress) {
      require(owner.cpu().registers() == *completed && owner.cpu().progress() == *progress,
              "Same/fresh-owner restore repeats complete native registers, stack and counters");
    }
    completed = owner.cpu().registers();
    progress = owner.cpu().progress();
  }
}

void native_vectors_and_owner_restore() {
  for (unsigned index = 0; index < 2; ++index) { native_event_case(index); }
  std::cout << "PASS TIMER_NATIVE_EVENTS contexts=2 continuations=8\n";
}
}

int main() {
  try {
    literal_external_sequences();
    shared_prescaler_and_limits();
    missing_registers_do_not_consume_edges();
    input_qualification_and_atomic_batches();
    pin_modes_and_rejections();
    native_vectors_and_owner_restore();
    std::cout
        << "Mode-three external/prescaled events, strict inputs, native IRQs and restore passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
