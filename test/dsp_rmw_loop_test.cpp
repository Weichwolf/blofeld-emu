// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp56kEmu/assembler.h"

#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld;
constexpr unsigned boot = 0xff0000, extal = 12000000, timebase = 1080000000;
constexpr unsigned pin = 0x200, padding = 16, maximum_periods = 160;
constexpr unsigned iterations = 3, body_words = 6, loop_flag = 0x8000;
constexpr std::uint64_t reset_ticks = 180;
// FM13-20/56/62: BCHG #9,X:$ffffbd; DO/DOR #3, own six-word body.
constexpr unsigned gpio_change = 0x017d09, do_three = 0x060380, dor_three = 0x060390;

void require(const bool condition, const char *message) {
  if (!condition) { throw Fault(message); }
}

DspFetchCycleInputs inputs() {
  return {.known_requests = 15, .grant = DspDmaGrant::available};
}

DspFetchRunInputs settings() {
  return {.extal_hz = extal,
          .timebase_hz = timebase,
          .timing = DspFetchTiming::reference_cycle_order,
          .execution = DspReferenceExecution::staged_register_boundaries,
          .pipeline = DspFetchPipeline::automatic_memory,
          .resets = DspFetchResetTiming::reference_cycle_boundary};
}

struct Layout {
  unsigned setup;
  unsigned body;
  unsigned last;
  unsigned bit;
};

class Program {
public:
  void emit(const char *source) {
    const auto result = assembler_.assemble(source);
    require(result.success(), "Own counted-loop program must assemble");
    for (unsigned i = 0; i < result.wordCount; ++i) { words_.push_back(result.word[i]); }
  }

  Layout install(DspBus &bus, const bool relative, const bool penultimate) {
    emit("movep #$000000,x:<<$ffffbf");
    emit("movep #$000200,x:<<$ffffbe");
    emit("movep #$000200,x:<<$ffffbd");
    for (unsigned i = 0; i < padding; ++i) { emit("nop"); }
    const auto setup = boot + static_cast<unsigned>(words_.size());
    const auto body = setup + 2;
    const auto last = body + body_words - 1;
    const auto bit = last - static_cast<unsigned>(penultimate);
    words_.push_back(relative ? dor_three : do_three);
    words_.push_back(relative ? last - setup : last);
    for (unsigned i = 0; i < body_words; ++i) {
      words_.push_back(body + i == bit ? gpio_change : 0U);
    }
    for (unsigned i = 0; i < padding; ++i) { emit("nop"); }
    bus.install_rom({.start = boot, .words = words_});
    return {.setup = setup, .body = body, .last = last, .bit = bit};
  }

private:
  dsp56k::Assembler assembler_;
  std::vector<std::uint32_t> words_;
};

DspDebugState until_setup(DspMachine &machine, DspFetchRun &run, const Layout layout) {
  for (unsigned i = 0; i < maximum_periods; ++i) {
    static_cast<void>(run.advance_cycle(machine, inputs()));
    if (machine.cpu().state().pc == layout.setup) { return machine.cpu().debug_state(); }
  }
  throw Fault("Native program did not reach its own DO/DOR setup");
}

void write(DspMachine &machine,
           DspFetchRun &run,
           const Layout layout,
           const DspDebugState &initial,
           const unsigned iteration) {
  const auto before = machine.cpu().debug_state();
  const bool old = (iteration & 1U) == 0;
  auto expected = before;
  ++expected.cpu.pc;
  expected.cpu.sr = (expected.cpu.sr & ~1U) | static_cast<unsigned>(old);
  ++expected.native_instructions;
  expected.native_cycles += 2;
  if (layout.bit == layout.last) {
    if (iteration + 1 < iterations) {
      expected.cpu.pc = layout.body;
      --expected.cpu.lc;
    } else {
      expected.cpu.la = initial.cpu.la;
      expected.cpu.lc = initial.cpu.lc;
      expected.cpu.sp = initial.cpu.sp;
      expected.cpu.stack_pc = initial.cpu.stack_pc;
      expected.cpu.stack_sr = initial.cpu.stack_sr;
      expected.sc = initial.sc;
      expected.cpu.sr &= ~loop_flag;
    }
  }
  const auto cycle = run.advance_cycle(machine, inputs());
  const auto actual = machine.cpu().debug_state();
  if (actual != expected) {
    std::cerr << "iteration=" << iteration << " pc=" << actual.cpu.pc << '/' << expected.cpu.pc
              << " sr=" << actual.cpu.sr << '/' << expected.cpu.sr << " la=" << actual.cpu.la << '/'
              << expected.cpu.la << " lc=" << actual.cpu.lc << '/' << expected.cpu.lc
              << " sp=" << actual.cpu.sp << '/' << expected.cpu.sp << " sc=" << actual.sc << '/'
              << expected.sc << " top=" << actual.cpu.stack_pc << '/' << expected.cpu.stack_pc
              << ',' << actual.cpu.stack_sr << '/' << expected.cpu.stack_sr
              << " counters=" << actual.native_instructions << '/' << expected.native_instructions
              << ',' << actual.native_cycles << '/' << expected.native_cycles << '\n';
  }
  require(cycle.bus.bit_write && cycle.bus.bit_write->value == static_cast<unsigned>(old) &&
              cycle.pipeline && cycle.pipeline->retirement_published &&
              machine.cpu().debug_state() == expected,
          "RMW E2 preserves literal DO/DOR backedge/fallthrough, LC, stack, Carry and native cost");
  const auto output = machine.bus().gpio(DspPort::c).outputs();
  require((output.known & pin) == pin && (output.levels & pin) == (old ? 0U : pin),
          "Each actual loop occurrence toggles its real GPIO exactly once");
}

void body(DspMachine &machine,
          DspFetchRun &run,
          const Layout layout,
          const DspDebugState &initial) {
  unsigned reads = 0;
  for (unsigned i = 0; i < maximum_periods; ++i) {
    const auto cycle = run.advance_cycle(machine, inputs());
    if (cycle.bus.memory && cycle.bus.memory->bit_read) {
      require(reads < iterations && machine.cpu().state().pc == layout.bit &&
                  machine.cpu().state().lc == iterations - reads,
              "Each genuine RMW E1 is paired with its literal loop occurrence and remaining LC");
      const auto saved = run.checkpoint(machine);
      write(machine, run, layout, initial, reads);
      run.restore(saved, machine);
      write(machine, run, layout, initial, reads);
      ++reads;
    }
    if (machine.cpu().state().pc == layout.last + 1 && reads == iterations) { return; }
  }
  throw Fault("Own counted loop did not reach its native fallthrough");
}

void execute(const bool relative, const bool penultimate) {
  DspMachine machine;
  Program program;
  const auto layout = program.install(machine.bus(), relative, penultimate);
  DspFetchRun run(settings(),
                  machine,
                  DspHardwareResetInputs{
                      .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2});
  require(run.advance_hardware_reset_ticks(machine, reset_ticks) == reset_ticks,
          "Counted-loop fixture supplies actual reset hold");
  run.release_hardware_reset(machine);
  require(run.advance_hardware_reset_ticks(machine, reset_ticks) == reset_ticks,
          "Counted-loop fixture supplies actual reset recovery");
  const auto initial = until_setup(machine, run, layout);
  body(machine, run, layout, initial);
  auto expected = initial;
  expected.cpu.pc = layout.last + 1;
  expected.cpu.sr |= 1;
  expected.stack_high.at(1) = initial.cpu.la;
  expected.stack_low.at(1) = initial.cpu.lc;
  expected.stack_high.at(2) = layout.body;
  expected.stack_low.at(2) = initial.cpu.sr;
  const auto actual = machine.cpu().debug_state();
  require(actual.cpu == expected.cpu && actual.stack_high == expected.stack_high &&
              actual.stack_low == expected.stack_low && actual.sc == expected.sc &&
              actual.native_instructions == initial.native_instructions + 1 +
                                                static_cast<std::uint64_t>(iterations) * body_words,
          "Final full CPU/stack and retirement count match independent loop semantics");
  std::cout << "PASS " << (relative ? "DOR" : "DO") << " bit-at=" << (penultimate ? "LA-1" : "LA")
            << " iterations=3 restored-E1-suffixes=3\n";
}
}

int main(const int argc, const char *const *argv) {
  try {
    require(argc == 2, "Choose do-last, do-before-last, dor-last or dor-before-last");
    const std::string_view requested(argv[1]);
    require(requested == "do-last" || requested == "do-before-last" || requested == "dor-last" ||
                requested == "dor-before-last",
            "Unknown own counted-loop RMW case");
    execute(requested.starts_with("dor"), requested.ends_with("before-last"));
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "FAIL RMW counted loop: " << error.what() << '\n';
    return 1;
  }
}
