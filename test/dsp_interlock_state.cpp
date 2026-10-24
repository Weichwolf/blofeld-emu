// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/dsp_scheduled_run.hpp"
#include "blofeld/dsp_timers.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_interlock_fixture.hpp"
#include "dsp_interlocks.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <string_view>

namespace blofeld::test::interlocks {
namespace {
constexpr unsigned address = 0x80;
constexpr unsigned value = 0x13579b;
constexpr unsigned other = 0x2468ac;
constexpr unsigned prefix_steps = 11;

void quiet(DspProgram &code) {
  for (unsigned index = 0; index < 8; ++index) { code.emit("nop"); }
}

void steps(DspMachine &machine, const unsigned count) {
  for (unsigned index = 0; index < count; ++index) { machine.cpu().step(); }
}

void expected_stalls(const DspMachine &machine, const unsigned stalls) {
  const auto before = machine.cpu().debug_state();
  const auto plan = machine.plan_step();
  require(plan.interlock_cycle_units == stalls && plan == machine.plan_step() &&
              machine.cpu().debug_state() == before,
          "Independent dependency clocks and pure repeated preview");
}

void seed(DspProgram &code) {
  code.immediate("r0", address);
  code.immediate("m0", word_mask);
  code.immediate("x0", address);
  quiet(code);
}

void register_sets() {
  for (unsigned reg = 0; reg < 8; ++reg) {
    const auto index = std::to_string(reg);
    for (const std::string_view member : {"r", "n", "m"}) {
      for (unsigned gap = 0; gap < 4; ++gap) {
        DspMachine machine(contract());
        DspProgram code;
        code.immediate("r" + index, address);
        code.immediate("m" + index, word_mask);
        code.immediate("x0", member == "m" ? word_mask : address);
        quiet(code);
        code.emit("move x0," + std::string(member) + index);
        for (unsigned padding = 0; padding < gap; ++padding) { code.emit("nop"); }
        code.emit("move x:(r" + index + "),y0");
        initialize(machine, code);
        machine.bus().write({.space = DspSpace::x, .address = address, .value = value});
        steps(machine, prefix_steps);
        const auto start = machine.run_clock().elapsed_ticks();
        steps(machine, 1 + gap);
        expected_stalls(machine, 3 - gap);
        machine.cpu().step();
        require(machine.run_clock().elapsed_ticks() - start == 5 &&
                    machine.run_clock().interlock_cycle_units() == 3 - gap &&
                    (machine.cpu().state().y & word_mask) == value,
                "All eight explicit R/N/M destination groups obey the literal3/2/1/0 delay");
      }
    }
  }
}

void elapsed_and_postupdate() {
  DspMachine machine(contract());
  DspProgram code;
  seed(code);
  code.emit("move x0,r0");
  code.emit("move #>$456789,y1"); // Two clock units, not one instruction of age.
  code.emit("move x:(r0)+,y1");
  code.emit("move x:(r0)+,y0");
  initialize(machine, code);
  machine.bus().write({.space = DspSpace::x, .address = address, .value = value});
  machine.bus().write({.space = DspSpace::x, .address = address + 1, .value = other});
  steps(machine, prefix_steps + 2);
  expected_stalls(machine, 1);
  machine.cpu().step();
  expected_stalls(machine, 0);
  machine.cpu().step();
  require(machine.run_clock().interlock_cycle_units() == 1 &&
              machine.cpu().state().r[0] == address + 2 &&
              machine.cpu().state().y == (std::uint64_t{value} << word_bits | other),
          "Elapsed clocks age dependencies; ordinary AGU post-updates never rearm move stalls");
}

void producer_forms() {
  struct Case {
    const char *source;
    unsigned native;
    unsigned remaining;
  };

  // Fresh NXP initial-map windows distinguish extension fetch from long data EA.
  constexpr std::array cases{Case{.source = "move x0,r0", .native = 1, .remaining = 3},
                             Case{.source = "move #<$80,r0", .native = 1, .remaining = 3},
                             Case{.source = "move #>$80,r0", .native = 2, .remaining = 2},
                             Case{.source = "move x:(r1),r0", .native = 1, .remaining = 3},
                             Case{.source = "move x:>$82,r0", .native = 2, .remaining = 3}};
  for (const auto &item : cases) {
    for (unsigned gap = 0; gap < 4; ++gap) {
      DspMachine machine(contract());
      DspProgram code;
      code.immediate("r0", address);
      code.immediate("m0", word_mask);
      code.immediate("r1", address + 2);
      code.immediate("m1", word_mask);
      code.immediate("x0", address);
      quiet(code);
      code.emit(item.source);
      for (unsigned padding = 0; padding < gap; ++padding) { code.emit("nop"); }
      code.emit("move x:(r0),y0");
      initialize(machine, code);
      machine.bus().write({.space = DspSpace::x, .address = address, .value = value});
      machine.bus().write({.space = DspSpace::x, .address = address + 2, .value = address});
      constexpr unsigned setup = 13;
      steps(machine, setup);
      const auto before = machine.run_clock().elapsed_ticks();
      steps(machine, gap + 1);
      const auto stalls = item.remaining > gap ? item.remaining - gap : 0;
      expected_stalls(machine, stalls);
      machine.cpu().step();
      require(machine.run_clock().elapsed_ticks() - before == item.native + gap + 1 + stalls &&
                  (machine.cpu().state().y & word_mask) == value,
              "Independent immediate/register/memory producer latency and actual pointer value");
    }
  }
}

void checkpoint_schedule_and_reset() {
  DspMachine machine(contract());
  DspProgram code;
  seed(code);
  code.emit("move x0,r0");
  code.emit("nop");
  code.emit("move x:(r0),y0");
  initialize(machine, code);
  machine.bus().write({.space = DspSpace::x, .address = address, .value = value});
  steps(machine, prefix_steps + 1);
  constexpr unsigned timer_control = 0xffff8f;
  constexpr unsigned timer_load = 0xffff8e;
  constexpr unsigned timer_compare = 0xffff8d;
  constexpr unsigned timer_count = 0xffff8c;
  constexpr unsigned compare_flag = 0x200000;
  machine.bus().timers().write({.address = timer_control, .value = 0});
  machine.bus().timers().write({.address = timer_load, .value = 0});
  machine.bus().timers().write({.address = timer_compare, .value = 1});
  machine.bus().timers().write({.address = timer_control, .value = 7});
  const auto saved = machine.checkpoint();
  constexpr unsigned completion = 19;
  constexpr unsigned retired_native = 17;
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(saved);
    DspScheduledRun schedule({.observation = DspStepObservation::reference_completion_boundary},
                             machine);
    constexpr unsigned maximum_steps = 1;
    require(schedule.advance_to(machine, 16, DspScheduleBoundary::through, maximum_steps) == 1,
            "The real pending NOP consumes one dependency clock");
    expected_stalls(machine, 2);
    require(machine.bus().timers().read(timer_control, compare_flag) == 0,
            "The first actual CLK2 edge only loads the timer, before the extra dependency clocks");
    require(schedule.next_completion_ticks(machine) == completion &&
                schedule.advance_to(
                    machine, completion - 1, DspScheduleBoundary::through, maximum_steps) == 0,
            "The scheduler cannot retire a dependency-stalled load at the nominal deadline");
    const auto pending = machine.checkpoint();
    const auto scheduled = schedule.checkpoint(machine);
    require(schedule.advance_to(machine, completion, DspScheduleBoundary::through, maximum_steps) ==
                1,
            "Native load retires at its explicitly declared dependency completion");
    machine.restore(pending);
    schedule.restore(scheduled, machine);
    require(
        schedule.advance_to(machine, completion, DspScheduleBoundary::through, maximum_steps) ==
                1 &&
            machine.run_clock().native_cycle_units() == retired_native &&
            machine.run_clock().interlock_cycle_units() == 2 &&
            machine.run_clock().timer_clk2_edges() == 9 &&
            machine.bus().timers().read(timer_count) == 1 &&
            machine.bus().timers().read(timer_control, compare_flag) == compare_flag &&
            (machine.cpu().state().y & word_mask) == value,
        "Joint restores retain actual native state, dependency ages, pending deadline and CLK2");
  }
  machine.cpu().reset_spi_boot();
  const std::array nop{0U};
  machine.bus().install_rom({.start = boot, .words = nop});
  machine.cpu().step();
  require(machine.run_clock().native_cycle_units() == retired_native + 1 &&
              machine.run_clock().interlock_cycle_units() == 2 &&
              machine.run_clock().elapsed_ticks() == completion + 1 &&
              machine.run_clock().epoch_native_cycle_units() == 1 &&
              machine.run_clock().epoch_interlock_cycle_units() == 0 &&
              machine.run_clock().timer_clk2_edges() == 9,
          "Nominal reset epoch retains lifetime counters and starts a fresh CLK2 phase");
}

void visible_failure_and_restore() {
  DspMachine machine(contract());
  DspProgram code;
  seed(code);
  code.emit("move x0,r0");
  const auto unsupported = code.pc();
  code.emit("rep #2");
  code.emit("move x:(r0),y0");
  initialize(machine, code);
  machine.bus().write({.space = DspSpace::x, .address = address, .value = value});
  steps(machine, prefix_steps + 1);
  const auto saved = machine.checkpoint();
  const auto before = machine.cpu().debug_state();
  const auto preview = checkpoint::rejects([&] { static_cast<void>(machine.plan_step()); },
                                           "Unqualified repeat timing unexpectedly previewed");
  require(!machine.timing_faulted() && machine.cpu().debug_state() == before &&
              preview.contains("opcode read P:" + hex(unsupported, 6)),
          "Unqualified repeat preview is pure and contextual");
  const auto error =
      checkpoint::rejects([&] { machine.cpu().step(); }, "Unqualified repeat executed");
  require(machine.timing_faulted() && machine.cpu().debug_state() == before &&
              error.contains("omr=0x000305") && error.contains("pc=" + hex(unsupported, 6)),
          "Unsupported attempted execution quarantines without retiring the instruction");
  machine.restore(saved);
  const std::array nop{0U};
  machine.bus().install_rom({.start = unsupported, .words = nop});
  machine.cpu().step();
  expected_stalls(machine, 2);
  machine.cpu().step();
  require((machine.cpu().state().y & word_mask) == value,
          "Trusted restore recovers pre-fault dependency history before the real load");
}

void status_checkpoint() {
  constexpr unsigned initial_status = 0xc00300;
  constexpr unsigned scaled_status = 0xc00380;
  constexpr unsigned scaled_value = 0x400000;
  constexpr unsigned setup_steps = 15;
  for (const bool preceding_partial : {false, true}) {
    DspMachine machine(contract());
    DspProgram code;
    code.immediate("a0", 0);
    code.immediate("a1", scaled_value);
    code.immediate("a2", 0);
    code.immediate("b0", 0);
    code.immediate("b1", 0);
    code.immediate("b2", 0);
    code.immediate("sr", initial_status);
    quiet(code);
    if (preceding_partial) { code.emit("move a1,x0"); }
    code.emit("move a,x0");
    code.emit("move sr,x1");
    code.emit("btst #7,sr");
    initialize(machine, code);
    steps(machine, setup_steps + (preceding_partial ? 2U : 1U));
    require(machine.cpu().state().sr == scaled_status,
            "A full accumulator bus read sets sticky S from the independent A46 XOR A45 rule");
    expected_stalls(machine, 2);
    const auto saved = machine.checkpoint();
    const auto start = machine.run_clock().elapsed_ticks();
    for (unsigned replay = 0; replay < 2; ++replay) {
      machine.restore(saved);
      DspScheduledRun schedule({.observation = DspStepObservation::reference_completion_boundary},
                               machine);
      constexpr unsigned maximum_steps = 1;
      require(schedule.next_completion_ticks(machine) == start + 3 &&
                  schedule.advance_to(
                      machine, start + 2, DspScheduleBoundary::through, maximum_steps) == 0 &&
                  schedule.advance_to(
                      machine, start + 3, DspScheduleBoundary::through, maximum_steps) == 1,
              "Restored SR dependency completes at the actual three-clock MOVE deadline");
      require((machine.cpu().state().x >> word_bits) == scaled_status &&
                  machine.run_clock().interlock_cycle_units() == 2,
              "A later known status writer dominates the older bounded partial-read uncertainty");
      machine.cpu().step();
      require(machine.cpu().state().sr == (scaled_status | 1U) &&
                  machine.run_clock().elapsed_ticks() == start + 5,
              "The actual two-clock BTST reads sticky S into C without clearing S");
    }
  }
}

void status_agu_dominance() {
  constexpr unsigned status = 0xc00300;
  constexpr unsigned target = 0x100;
  constexpr unsigned setup_steps = 12;
  DspMachine machine(contract());
  DspProgram code;
  code.immediate("a0", 0);
  code.immediate("a1", target);
  code.immediate("a2", 0);
  code.immediate("sr", status);
  quiet(code);
  code.emit("move a1,r0");
  code.emit("move sr,x:(r0)");
  initialize(machine, code);
  steps(machine, setup_steps + 1);
  expected_stalls(machine, 3);
  const auto before = machine.run_clock().elapsed_ticks();
  machine.cpu().step();
  require(machine.bus().read({.space = DspSpace::x, .address = target}) == status &&
              machine.run_clock().interlock_cycle_units() == 3 &&
              machine.run_clock().elapsed_ticks() == before + 4,
          "The known three-clock AGU wait dominates older bounded SR timing uncertainty");
}

void fetched_identity() {
  DspMachine machine(contract());
  DspProgram code;
  seed(code);
  const auto changed = code.pc();
  code.emit("move x0,r0");
  code.emit("move x:(r0),y0");
  initialize(machine, code);
  steps(machine, prefix_steps);
  const auto before = machine.cpu().debug_state();
  const std::array nop{0U};
  const auto error = checkpoint::rejects(
      [&] {
        machine.cpu().step({.observe = [&](const DspNativeAccess &access) {
          // Deliberately invalid observer: prove the timing guard catches mutation.
          if (access.kind == DspNativeAccessKind::opcode &&
              access.stage == DspNativeAccessStage::before) {
            machine.bus().install_rom({.start = changed, .words = nop});
          }
        }});
      },
      "Mutated fetched instruction bypassed its timing guard");
  require(error.contains("after opcode at P:" + hex(changed, 6)) &&
              error.contains("changed after its timing preview") && machine.timing_faulted() &&
              machine.cpu().debug_state() == before,
          "Actual fetched opcode identity is checked before architectural effects");
}

void history_reset_and_unknown_modes() {
  DspInterlocks history;
  const auto write = checkpoint::assemble("move x0,r0").front();
  const auto read = checkpoint::assemble("move x:(r0),y0").front();
  history.prepare(write, 1);
  history.observe_opcode(write);
  require(history.complete(1) == 0 && history.preview(read, 1) == 3,
          "Independent explicit destination arms three future dependency clocks");
  history.reset();
  require(history.preview(read, 1) == 0, "Reset clears history without assuming reset R0 contents");
  auto incompatible = contract();
  incompatible.interrupts = DspInterruptTiming::native_vector_estimates;
  require(checkpoint::rejects([&] { const DspMachine rejected(incompatible); },
                              "Unqualified interlock IRQ contract accepted")
              .contains("interrupt-vector timing"),
          "Selected dependency timing cannot silently claim IRQ pipeline coverage");
}

void data_bus_scope() {
  constexpr unsigned dma_counter = 0xffffed;
  DspMachine machine(contract());
  DspProgram code;
  code.immediate("x0", address);
  code.emit("move x0,x:>$ffffed"); // DMA0 counter is I/O, not a qualified RAM store.
  initialize(machine, code);
  machine.cpu().step();
  const auto before = machine.cpu().debug_state();
  const auto error = checkpoint::rejects([&] { machine.cpu().step(); },
                                         "Peripheral timing was silently treated as internal RAM");
  require(machine.timing_faulted() &&
              machine.cpu().debug_state().native_instructions == before.native_instructions &&
              error.contains("before write at X:0xffffed") &&
              error.contains("Unqualified DSP peripheral memory resource"),
          "The selected dependency profile rejects actual peripheral accesses before I/O effects");
  require(checkpoint::rejects(
              [&] {
                static_cast<void>(
                    machine.bus().read({.space = DspSpace::x, .address = dma_counter}));
              },
              "Rejected I/O initialized the DMA counter")
              .contains("Unknown DSP DMA address/counter"),
          "No DMA register value is fabricated on the unsupported data-bus path");
}
}

void state_checks() {
  register_sets();
  producer_forms();
  elapsed_and_postupdate();
  checkpoint_schedule_and_reset();
  visible_failure_and_restore();
  status_checkpoint();
  status_agu_dominance();
  fetched_identity();
  history_reset_and_unknown_modes();
  data_bus_scope();
}
}
