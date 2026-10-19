// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_timers.hpp"
#include "dsp56kEmu/assembler.h"
#include "dsp_norm_fixture.hpp"
#include "nand_program_fixture.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace blofeld::test {
namespace {
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
  rejects([&] { timers.write({.address = control0, .value = 0}); }, "active DSP timer mode change");
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

void test_native_capture_interrupt() {
  DspProgram program;
  program.immediate("sr", ordinary_status);
  program.immediate("r2", 0);
  program.immediate("r7", 0);
  movep(program, timer_priority, priorities, 'x');
  movep(program, 10, control0 - 1, 'x');
  const auto enable_pc = program.pc();
  movep(program, capture_mode | enable | capture_enable | flags, control0, 'x');
  const auto foreground = program.pc();
  program.emit("move #<9,r7");
  DspBus bus;
  program.install(bus);
  const dsp56k::Assembler assembler;
  const auto vector = assembler.assemble("move #<7,r2");
  require(vector.wordCount == 1,
          "Own fast capture vector must have one explicit instruction plus NOP");
  bus.write({.address = timer_vector, .value = vector.word[0]});
  bus.write({.address = timer_vector + 1, .value = 0});
  DspCore cpu(bus);
  while (cpu.state().pc != enable_pc) { cpu.step(); }
  bus.timers().observe_synchronized_tio(0, false);
  cpu.step();
  require(cpu.state().pc == foreground, "Original-shaped native mode-six control write executes");
  bus.timers().advance_clk2(4);
  bus.timers().observe_synchronized_tio(0, true);
  cpu.step();
  require(cpu.state().pc == foreground && cpu.state().r[2] == 7 && cpu.state().r[7] == 0 &&
              bus.timers().read(control0 - 3) == 13 && bus.timers().read(control0, flags) == 0,
          "Native fast capture IRQ executes its vector and acknowledges only the source flag");
  cpu.step();
  require(cpu.state().pc == program.pc() && cpu.state().r[7] == 9,
          "Native foreground resumes after capture IRQ without host CPU-register replacement");
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
