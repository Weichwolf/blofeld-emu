// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "timer_native_fixture.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace blofeld::test {
namespace {
using timer_native::require;
constexpr unsigned capture_mode = 0x60;
constexpr unsigned enable = 1;
constexpr unsigned overflow_enable = 2;
constexpr unsigned capture_enable = 4;
constexpr unsigned inv = 0x100;
constexpr unsigned reload = 0x200;
constexpr unsigned direction = 0x800;
constexpr unsigned input = 0x1000;
constexpr unsigned prescaled = 0x8000;
constexpr unsigned overflow_flag = 0x100000;
constexpr unsigned capture_flag = 0x200000;
constexpr unsigned flags = overflow_flag | capture_flag;
constexpr unsigned control0 = 0xffff8f;
constexpr unsigned control1 = 0xffff8b;
constexpr unsigned control2 = 0xffff87;
constexpr unsigned prescaler_load = 0xffff83;
constexpr unsigned prescaler_count = 0xffff82;
constexpr unsigned timer_vector = 0x54;
constexpr unsigned overflow_vector = 0x56;
constexpr unsigned priorities = 0xfffffe;
constexpr unsigned timer_priority = 0x100;
constexpr unsigned ordinary_status = 0xc00000;
constexpr unsigned word_mask = 0xffffff;

void rejects(const auto &operation, const std::string &reason) {
  try {
    operation();
  } catch (const Fault &error) {
    require(std::string(error.what()).contains(reason),
            "Unexpected capture rejection: " + std::string(error.what()));
    return;
  }
  throw Fault("Unqualified capture operation succeeded: " + reason);
}

void configure(DspTimers &timers,
               const unsigned polarity = 0,
               const bool start_at_selected_level = false) {
  timers.write({.address = control0, .value = capture_mode | polarity});
  const bool initial = start_at_selected_level ? polarity == 0 : polarity != 0;
  timers.observe_synchronized_tio(0, initial);
  timers.write({.address = control0 - 1, .value = 10});
  timers.write({.address = control0, .value = capture_mode | polarity | enable | capture_enable});
}

void test_first_selected_edge_and_halt() {
  for (const unsigned polarity : {0U, inv}) {
    DspTimers timers;
    configure(timers, polarity, true);
    require(timers.read(control0 - 3) == 0, "TCR retains its defined zero reset until capture");
    timers.advance_clk2(4); // First clock loads10; next clocks produce11,12,13.
    timers.observe_synchronized_tio(0, polarity != 0);
    require(timers.read(control0, flags) == 0 && timers.read(control0 - 3) == 0,
            "The opposite synchronized edge does not capture the running internal count");
    timers.observe_synchronized_tio(0, polarity == 0);
    require(timers.read(control0 - 3) == 13 && timers.read(control0, flags) == capture_flag &&
                timers.interrupt_vectors() == std::vector<std::uint32_t>{timer_vector},
            "Selected rising/falling edge captures13 and requests the actual compare vector");
    timers.advance_clk2(std::numeric_limits<std::uint64_t>::max());
    timers.acknowledge_interrupt(timer_vector);
    timers.observe_synchronized_tio(0, polarity != 0);
    timers.observe_synchronized_tio(0, polarity == 0);
    require(timers.read(control0 - 3) == 13 && timers.read(control0, flags) == 0,
            "Captured counter stays halted after IRQ acknowledgement and later selected edges");
    timers.write({.address = control0, .value = capture_mode | polarity});
    timers.observe_synchronized_tio(0, polarity != 0);
    timers.write({.address = control0, .value = capture_mode | polarity | enable});
    require(timers.read(control0 - 3) == 13,
            "Rearming clears the internal counter while retaining the previous TCR capture");
    timers.observe_synchronized_tio(0, polarity == 0);
    require(timers.read(control0 - 3) == 0 && timers.read(control0, flags) == capture_flag,
            "An explicitly ordered edge before the first timer clock captures the cleared zero");
  }
}

void test_compare_ignored_and_overflow() {
  DspTimers timers;
  timers.write({.address = control0 - 2, .value = 12});
  configure(timers);
  timers.advance_clk2(4);
  require(timers.read(control0, flags) == 0,
          "Measurement mode ignores TCPR equality instead of inventing a compare event");
  timers.write({.address = control0, .value = capture_mode});
  timers.write({.address = control0 - 1, .value = word_mask});
  timers.write(
      {.address = control0, .value = capture_mode | enable | capture_enable | overflow_enable});
  timers.advance_clk2(2); // Loadffffff, then increment to0 and set overflow.
  timers.observe_synchronized_tio(0, true);
  const std::vector<std::uint32_t> both{overflow_vector, timer_vector};
  require(timers.read(control0 - 3) == 0 && timers.read(control0, flags) == flags &&
              timers.interrupt_vectors() == both,
          "Overflow remains sticky when the following capture latches zero");
  timers.write({.address = control0, .value = capture_mode | enable | capture_flag});
  require(timers.read(control0, flags) == overflow_flag,
          "Capture W1C clears only its selected sticky flag and does not rearm the counter");
  timers.write({.address = control0, .value = capture_mode});
  timers.observe_synchronized_tio(0, false);
  timers.write({.address = control0 - 1, .value = 3});
  timers.write({.address = control0, .value = capture_mode | enable});
  timers.advance_clk2(std::numeric_limits<std::uint64_t>::max());
  timers.observe_synchronized_tio(0, true);
  require(timers.read(control0 - 3) == 1 && timers.read(control0, flags) == flags,
          "Load3 followed by UINT64_MAX-1 increments wraps to1 without host overflow");
}

void test_prescaler_and_unknown_batch() {
  DspTimers timers;
  timers.write({.address = prescaler_load, .value = 3});
  timers.write({.address = control0 - 1, .value = 5});
  timers.write({.address = control0 - 2, .value = 9});
  timers.write({.address = control0, .value = prescaled | enable});
  timers.write({.address = control1, .value = capture_mode | prescaled | enable});
  rejects([&] { timers.advance_clk2(4); }, "Unknown DSP timer capture load");
  require(timers.read(control0 - 3) == 0 && timers.read(prescaler_count) == 3,
          "Unknown capture preload rejects the whole batch including another timer and prescaler");
  timers.write({.address = control1, .value = capture_mode | prescaled});
  timers.observe_synchronized_tio(1, false);
  timers.write({.address = control1 - 1, .value = 7});
  timers.write({.address = control1, .value = capture_mode | prescaled | enable});
  timers.advance_clk2(8); // Prescaler pulses on clocks4/8: load7, then8.
  timers.observe_synchronized_tio(1, true);
  require(timers.read(control1 - 3) == 8 && timers.read(control0 - 3) == 6 &&
              timers.read(prescaler_count) == 3,
          "Mode-six and mode-zero users share the existing four-clock prescaler phase");
}

void test_input_qualification_and_rejections() {
  DspTimers timers;
  timers.observe_synchronized_tio(0, false);
  require(timers.read(control0, flags) == 0,
          "Qualified synchronizer observations can precede enabling measurement mode");
  timers.reset();
  rejects([&] { timers.observe_synchronized_tio(2, false); }, "no synchronized TIO");
  rejects([&] { timers.write({.address = control2, .value = capture_mode}); },
          "Unsupported DSP timer control");
  rejects([&] { timers.write({.address = control0, .value = capture_mode | reload}); },
          "Unsupported DSP timer control");
  rejects([&] { timers.write({.address = control0, .value = capture_mode | direction}); },
          "Unsupported DSP timer control");
  timers.write({.address = control0, .value = capture_mode | enable});
  rejects([&] { timers.observe_synchronized_tio(0, true); }, "Unknown previous DSP synchronized");
  timers.write({.address = control0 - 1, .value = 10});
  rejects([&] { timers.advance_clk2(1); }, "Unknown DSP timer capture input during clock");
  require(timers.read(control0 - 3) == 0,
          "Unknown synchronized input rejects the entire capture clock observation");
  require(timers.read(control0, flags) == 0,
          "A rejected unknown previous input does not invent capture");
  rejects([&] { timers.write({.address = control0, .value = enable}); },
          "active DSP timer mode change");
  rejects([&] { timers.write({.address = control0, .value = capture_mode | enable | inv}); },
          "capture polarity change");
  timers.set_tio_drive(0, false);
  timers.set_tio_drive(0, true);
  require(timers.read(control0, input) == input && timers.read(control0, flags) == 0,
          "Raw pad levels qualify DI without guessing synchronized edge/counter timing");
  timers.write({.address = control0, .value = capture_mode});
  timers.observe_synchronized_tio(0, false);
  timers.write({.address = control0, .value = capture_mode | enable});
  timers.observe_synchronized_tio(0, true);
  timers.reset();
  require(timers.read(control0 - 3) == 0 && timers.read(control0, flags) == 0 &&
              timers.read(control0, input) == input,
          "Reset clears capture/count/flags while preserving the observed external pad");
}

void test_changed_raw_input_requires_new_qualification() {
  DspTimers timers;
  configure(timers);
  timers.set_tio_drive(0, false);
  timers.observe_synchronized_tio(0, false);
  timers.advance_clk2(2); // Load10, then11.
  timers.set_tio_drive(0, true);
  rejects([&] { timers.advance_clk2(1); }, "Unknown DSP timer capture input during clock");
  // Explicitly observed old synchronizer output may lag the changed raw pad.
  timers.observe_synchronized_tio(0, false);
  timers.advance_clk2(1);
  timers.observe_synchronized_tio(0, true);
  require(timers.read(control0 - 3) == 12,
          "Unqualified raw-edge batch must not advance the counter before a qualified capture");
}

struct NativeCaptureInput {
  unsigned index;
  unsigned polarity;
};

void native_capture_case(const NativeCaptureInput setup) {
  using namespace dma_test;
  const auto index = setup.index;
  const auto polarity = setup.polarity;
  const auto csr = control0 - (4 * index);
  const auto vector = timer_vector + (4 * index);
  NativeProgram program;
  program.word(timer_native::move_sr);
  program.word(ordinary_status);
  program.word(timer_native::move_r2);
  program.word(0);
  program.word(timer_native::move_r7);
  program.word(0);
  program.write({.space = DspSpace::x, .address = priorities, .value = timer_priority});
  program.write({.space = DspSpace::x, .address = csr - 1, .value = 10});
  const auto enable_pc = program.pc();
  program.write({.space = DspSpace::x,
                 .address = csr,
                 .value = capture_mode | polarity | enable | capture_enable | flags});
  const auto foreground = program.pc();
  program.word(timer_native::foreground_move);
  const auto complete = program.pc();
  OwnDspMachine machine(native_clocks);
  reset_native(machine, program);
  timer_native::install_vector(machine, vector, timer_native::short_r2);
  for (unsigned instruction = 0; instruction < 5; ++instruction) { step_native(machine); }
  require(machine.cpu().registers().pc == enable_pc,
          "Five literal native setup instructions stop before the actual TCSR enable write");
  const bool initial = polarity != 0;
  timer_native::sample(machine.bus().timers(), index, initial);
  step_native(machine);
  require(machine.cpu().registers().pc == foreground &&
              machine.cpu().progress().instructions == 6 &&
              machine.cpu().progress().estimated_cycles == 12,
          "Six actual two-word native instructions establish the capture foreground");
  machine.bus().timers().advance_clk2(4); // Actual load10, then11,12,13.
  timer_native::sample(machine.bus().timers(), index, !initial);
  require(machine.bus().timers().read(csr - 3) == 13 &&
              machine.bus().timers().read(csr, flags) == capture_flag,
          "Only the selected synchronized edge captures thirteen and requests TCF");
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
    timer_native::service(owner,
                          {.vector = vector},
                          {.address = foreground, .opcode = timer_native::foreground_move});
    expected.r[2] = 7;
    require(
        owner.cpu().registers() == expected && !owner.bus().interrupt_request(0) &&
            owner.bus().timers().read(csr - 3) == 13 && owner.bus().timers().read(csr, flags) == 0,
        "Actual fast vector changes only R2 and acknowledges the captured source without rearming");
    step_native(owner);
    expected.pc = complete;
    expected.r[7] = 9;
    require(owner.cpu().registers() == expected &&
                owner.cpu().progress().instructions == before.instructions + 3 &&
                owner.cpu().progress().estimated_cycles == before.estimated_cycles + 3 &&
                owner.clock().elapsed_ticks() == ticks,
            "Real foreground MOVE changes only R7/PC with three literal instruction/base costs");
    if (completed && progress) {
      require(owner.cpu().registers() == *completed && owner.cpu().progress() == *progress,
              "Same/fresh-owner restore repeats complete native registers, stack and counters");
    }
    completed = owner.cpu().registers();
    progress = owner.cpu().progress();
  }
}

void test_native_capture_interrupt() {
  for (unsigned index = 0; index < 2; ++index) {
    for (const unsigned polarity : {0U, inv}) {
      native_capture_case({.index = index, .polarity = polarity});
    }
  }
  std::cout << "PASS TIMER_NATIVE_CAPTURE contexts=4 continuations=16\n";
}
}
}

int main() {
  try {
    blofeld::test::test_first_selected_edge_and_halt();
    blofeld::test::test_compare_ignored_and_overflow();
    blofeld::test::test_prescaler_and_unknown_batch();
    blofeld::test::test_input_qualification_and_rejections();
    blofeld::test::test_changed_raw_input_requires_new_qualification();
    blofeld::test::test_native_capture_interrupt();
    std::cout
        << "Mode-six capture clocks, selected edges, halt/rearm, flags and native IRQ passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
