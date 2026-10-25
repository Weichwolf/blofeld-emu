// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp56kEmu/assembler.h"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using blofeld::test::DspProgram;
using blofeld::test::number;
using blofeld::test::require;
constexpr unsigned source = 0x660;
constexpr unsigned data = 0x123456;
constexpr unsigned next_data = 0x234567;
constexpr unsigned register_count = 8;
constexpr unsigned ready_clocks = 3;
constexpr unsigned maximum_setup = 128;
constexpr unsigned ram_base = 0x100;
constexpr unsigned timer_control = 0xffff8f;
constexpr unsigned timer_load = timer_control - 1;
constexpr unsigned timer_compare = timer_control - 2;
constexpr unsigned timer_count = timer_control - 3;
constexpr std::uint64_t nominal_period = 45;
constexpr unsigned extal_hz = 12000000;
constexpr unsigned timebase_hz = 540000000;

DspRunContract contract() {
  return {.clock = {.extal_hz = extal_hz,
                    .timebase_hz = timebase_hz,
                    .switch_timing = DspClockSwitch::reference_instruction_boundary},
          .timing = DspRunTiming::native_internal_agu_interlocks,
          .interrupts = DspInterruptTiming::native_vector_estimates,
          .resets = DspResetTiming::reference_instruction_boundary};
}

class Rig {
public:
  explicit Rig(const std::vector<std::string> &body, const bool ram = false) : machine(contract()) {
    script = body.front();
    DspProgram program;
    program.immediate("sr", blofeld::test::masked_status);
    for (unsigned index = 0; index < register_count; ++index) {
      program.immediate("r" + std::to_string(index), source);
      program.immediate("n" + std::to_string(index), 0);
      program.immediate("m" + std::to_string(index), blofeld::test::word_mask);
    }
    for (unsigned index = 0; index < ready_clocks; ++index) { program.emit("nop"); }
    work = program.pc();
    for (const auto &line : body) { program.emit(line); }
    program.emit("nop");
    program.install(machine.bus());
    if (ram) {
      for (unsigned pc = blofeld::test::boot; pc < program.pc(); ++pc) {
        machine.bus().write({.address = ram_base + pc - blofeld::test::boot,
                             .value = machine.bus().read({.address = pc})});
      }
      work = ram_base + work - blofeld::test::boot;
      DspProgram entry;
      entry.emit("jmp " + number(ram_base));
      entry.install(machine.bus());
    }
    for (const auto space : {DspSpace::x, DspSpace::y}) {
      machine.bus().write({.space = space, .address = source, .value = data});
      machine.bus().write({.space = space, .address = source + 1, .value = next_data});
    }
    machine.bus().pll().observe_pinit(false);
    machine.cpu().reset_spi_boot();
    for (unsigned count = 0; machine.cpu().state().pc != work && count < maximum_setup; ++count) {
      machine.cpu().step();
    }
    require(machine.cpu().state().pc == work && machine.run_clock().interlock_cycle_units() == 0,
            "Own setup reaches an independently drained AGU history");
  }

  void step(const unsigned stalls) {
    const auto plan = machine.plan_step();
    const auto before = machine.cpu().debug_state();
    const auto ticks = machine.run_clock().elapsed_ticks();
    const auto elapsed = machine.run_clock().elapsed_cycle_units();
    const auto interlocks = machine.run_clock().interlock_cycle_units();
    require(plan.interlock_cycle_units == stalls && machine.cpu().debug_state() == before,
            "AGU preview actual=" + std::to_string(plan.interlock_cycle_units) + " expected=" +
                std::to_string(stalls) + " pc=" + hex(before.cpu.pc, 6) + " script=" + script);
    machine.cpu().step();
    require(machine.run_clock().epoch_native_cycle_units() ==
                    machine.cpu().debug_state().native_cycles &&
                machine.run_clock().elapsed_ticks() == ticks + plan.duration_ticks &&
                machine.run_clock().elapsed_cycle_units() ==
                    elapsed + plan.native_cycle_units + stalls &&
                machine.run_clock().interlock_cycle_units() == interlocks + stalls,
            "Native instruction units, elapsed units and actual CLK2 accounting remain distinct");
  }

  DspMachine machine;
  unsigned work = 0;
  std::string script;
};

template <typename Operation> std::string failure(Operation operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected an explicit nominal AGU rejection");
}

void literal_pair_matrix() {
  for (unsigned index = 0; index < register_count; ++index) {
    for (const auto &family : std::array<std::string, 3>{"r", "n", "m"}) {
      auto value = family == "r" ? source : 0;
      if (family == "m") { value = blofeld::test::word_mask; }
      for (unsigned gap = 0; gap <= ready_clocks; ++gap) {
        std::vector<std::string> body{"move #>" + number(value) + ',' + family +
                                      std::to_string(index)};
        for (unsigned count = 0; count < gap; ++count) { body.emplace_back("nop"); }
        body.push_back("move x:(r" + std::to_string(index) + "),x0");
        Rig rig(body);
        rig.step(0);
        for (unsigned count = 0; count < gap; ++count) { rig.step(0); }
        rig.step(ready_clocks - gap);
        require(
            rig.machine.cpu().state().x == data,
            "Every R/N/M member forwards to its own actual address pair, even unchanged writes");
      }
    }
  }
}

void clock_distance_and_forwarding() {
  for (const auto &gap :
       std::array<std::string, 2>{"move #>$123456,x1", "movep x:<<$fffffd,p:(r7)"}) {
    // A long immediate occupies two nominal clocks; P-space MOVEP occupies six.
    const bool immediate = gap.starts_with("move ");
    Rig rig({"move #>$660,r0", gap, "move x:(r0),x0"});
    rig.step(0);
    constexpr unsigned program_peripheral_units = 6;
    require(rig.machine.plan_step().native_cycle_units ==
                (immediate ? 2 : program_peripheral_units),
            "The gap must encode its actual long-immediate or P-space peripheral form");
    rig.step(0);
    rig.step(immediate ? 1 : 0);
  }
  Rig manual({"move #>$660,r0", "clr a", "move x:(r0)+,y1", "move x:(r0)+,y0"});
  for (const auto stalls : std::array{0U, 0U, 2U, 0U}) { manual.step(stalls); }
  require(manual.machine.cpu().state().r[0] == source + 2 &&
              manual.machine.cpu().state().y ==
                  (static_cast<std::uint64_t>(data) << blofeld::test::word_bits) + next_data,
          "FM A-2: inserted clocks satisfy the next consumer; pointer updates do not rearm MOVE "
          "stalls");
  Rig separate({"move #>$660,r1", "move x:(r0),x0"});
  separate.step(0);
  separate.step(0);
  require(separate.machine.cpu().state().x == data, "A distinct physical AGU pair does not stall");
}

void parallel_consumers_and_explicit_destinations() {
  Rig parallel({"move #>$660,r0", "move #>$660,r4", "move x:(r0)+,x0 y:(r4)+,y0"});
  parallel.step(0);
  parallel.step(0);
  parallel.step(3);
  require(parallel.machine.cpu().state().x == data && parallel.machine.cpu().state().y == data &&
              parallel.machine.cpu().state().r[0] == source + 1 &&
              parallel.machine.cpu().state().r[4] == source + 1,
          "Simultaneous X/Y uses take the longest dependency, not two serial stall sums");
  Rig loaded({"move x:(r0)+,r1", "move x:(r1),x0"});
  loaded.machine.bus().write({.space = DspSpace::x, .address = source, .value = source + 1});
  loaded.step(0);
  loaded.step(3);
  require(
      loaded.machine.cpu().state().x == next_data,
      "A real memory-to-R MOVE is an explicit writer; unrelated source postupdate stays distinct");
}

void other_move_destinations_and_sources() {
  constexpr unsigned program_data = source;
  constexpr unsigned short_data = 0x20;
  constexpr unsigned dma_done = 0x3f;
  for (const auto &line : std::array<std::string, 6>{"move r0,r1",
                                                     "move x:>$660,r1",
                                                     "move x:<$20,r1",
                                                     "move p:(r0),r1",
                                                     "movep x:<<$ffff8c,r1",
                                                     "movep x:<<$fffff4,r1"}) {
    Rig rig({line, "move x:(r1),x0"});
    rig.machine.bus().write({.address = program_data, .value = source + 1});
    rig.machine.bus().write({.space = DspSpace::x, .address = short_data, .value = source + 1});
    rig.machine.bus().write({.space = DspSpace::x, .address = source, .value = source + 1});
    rig.machine.bus().write({.space = DspSpace::x, .value = next_data});
    rig.machine.bus().write({.space = DspSpace::x, .address = dma_done, .value = next_data});
    rig.step(0);
    rig.step(3);
    const auto expected = line == "move r0,r1" ? source + 1 : next_data;
    require(rig.machine.cpu().state().x == expected,
            "Actual register, memory, P-memory and peripheral MOVE destinations retain readiness");
  }
  Rig control({"move #>$660,m0", "move m0,r1", "move x:(r1),x0"});
  control.step(0);
  control.step(0);
  control.step(3);
  require(control.machine.cpu().state().x == data,
          "MOVEC reverse direction writes its actual R destination");
  Rig short_modifier({"move #<$ff,m1", "move x:(r1),x0"});
  short_modifier.step(0);
  short_modifier.step(3);
  Rig reverse({"move #>$660,r4", "move #>$660,r0", "move x:(r4)+,x0 y:(r0)+,y0"});
  reverse.step(0);
  reverse.step(0);
  reverse.step(3);
  require(reverse.machine.cpu().state().x == data && reverse.machine.cpu().state().y == data,
          "Parallel Y address selection follows the opposite register bank in either orientation");
  for (const auto &line :
       std::array<std::string, 3>{"move x:(r0+n0),x0", "move x:(r0)+n0,x0", "move l:(r0),a"}) {
    Rig rig({"move #>$660,r0", line});
    rig.step(0);
    rig.step(3);
    require(rig.machine.cpu().state().r[0] == source,
            "Offset, zero postincrement and long-memory uses preserve their actual address effect");
  }
}

void timer_and_checkpoint() {
  Rig rig({"move #>$660,r0", "nop", "move x:(r0),x0", "nop"});
  rig.step(0);
  rig.step(0);
  auto &timer = rig.machine.bus().timers();
  timer.write({.address = timer_load, .value = 7});
  timer.write({.address = timer_compare, .value = 8});
  timer.write({.address = timer_control, .value = 7});
  const auto saved = rig.machine.checkpoint();
  const auto before = rig.machine.run_clock().elapsed_cycle_units();
  const auto first_edges = ((before & 1U) + 3) / 2;
  rig.step(2);
  require(rig.machine.bus().timers().read(timer_count) == 7 + first_edges - 1,
          "Actual timer edges include the two inserted clocks and the retained odd phase");
  rig.step(0);
  const auto expected = rig.machine.cpu().debug_state();
  const auto ticks = rig.machine.run_clock().elapsed_ticks();
  const auto elapsed = rig.machine.run_clock().elapsed_cycle_units();
  const auto counter = rig.machine.bus().timers().read(timer_count);
  for (unsigned replay = 0; replay < 2; ++replay) {
    rig.machine.restore(saved);
    rig.step(2);
    rig.step(0);
    require(rig.machine.cpu().debug_state() == expected &&
                rig.machine.run_clock().elapsed_ticks() == ticks &&
                rig.machine.run_clock().elapsed_cycle_units() == elapsed &&
                rig.machine.bus().timers().read(timer_count) == counter,
            "Coordinated restore retains pending AGU readiness, elapsed phase and native state");
  }
}

void latched_repeat_and_reset() {
  Rig rig({"rep #3", "move x:(r0),r1", "move x:(r1),x0"}, true);
  rig.machine.bus().write({.space = DspSpace::x, .address = source, .value = source});
  rig.step(0);
  const auto body = rig.machine.cpu().state().pc;
  rig.step(0);
  const dsp56k::Assembler assembler;
  const auto different = assembler.assemble("move x:(r1),x0");
  require(different.success() && different.wordCount == 1, "Own replacement must be one word");
  rig.machine.bus().write({.address = body, .value = different.word[0]});
  const auto saved = rig.machine.checkpoint();
  rig.step(0);
  rig.step(0);
  rig.step(3);
  const auto expected = rig.machine.cpu().debug_state();
  const auto elapsed = rig.machine.run_clock().elapsed_cycle_units();
  rig.machine.restore(saved);
  rig.step(0);
  rig.step(0);
  rig.step(3);
  require(rig.machine.cpu().debug_state() == expected &&
              rig.machine.run_clock().elapsed_cycle_units() == elapsed,
          "A loaded REP previews its actual captured opcode rather than mutated P memory");
  const auto ticks = rig.machine.run_clock().elapsed_ticks();
  rig.machine.cpu().reset_spi_boot();
  require(rig.machine.run_clock().elapsed_ticks() == ticks &&
              rig.machine.run_clock().epoch_native_cycle_units() == 0,
          "Explicit reset preserves lifetime elapsed time and clears the nominal epoch history");
  DspProgram after_reset;
  after_reset.emit("move x:(r1),x0");
  after_reset.install(rig.machine.bus());
  rig.machine.bus().write({.space = DspSpace::x, .value = data});
  rig.step(0);
  require(rig.machine.cpu().state().x == data,
          "A reset-cleared R1 has no retained pre-reset MOVE dependency at the new epoch entry");
}

void scheduler_uses_elapsed_deadline() {
  Rig rig({"move #>$660,r0", "move x:(r0),x0"});
  rig.step(0);
  const auto start = rig.machine.run_clock().elapsed_ticks();
  const auto plan = rig.machine.plan_step();
  require(plan.native_cycle_units == 1 && plan.interlock_cycle_units == 3 &&
              plan.duration_ticks == 4 * nominal_period,
          "The scheduler previews a real load with one instruction unit and three inserted units");
  auto &timer = rig.machine.bus().timers();
  timer.set_tio_drive(0, false);
  timer.observe_synchronized_tio(0, false);
  timer.write({.address = timer_load, .value = 7});
  timer.write({.address = timer_compare, .value = 8});
  constexpr unsigned event_control = 0x31;
  timer.write({.address = timer_control, .value = event_control});
  bool delivered = false;
  const auto edge = start + (2 * nominal_period);
  const DspExternalEvents external{
      .next_ticks = [&]() { return delivered ? std::nullopt : std::optional(edge); },
      .observe =
          [&](const auto at) {
            require(
                at == edge && rig.machine.run_clock().elapsed_ticks() == start,
                "A fixed external edge keeps its timestamp inside the longer nominal instruction");
            rig.machine.bus().timers().set_tio_drive(0, true);
            rig.machine.bus().timers().observe_synchronized_tio(0, true);
            delivered = true;
          },
      .order = DspExternalOrder::external_before_dsp,
      .maximum_events = 1};
  DspScheduledRun schedule({.observation = DspStepObservation::reference_completion_boundary},
                           rig.machine);
  require(schedule.advance_to(rig.machine,
                              start + plan.duration_ticks - 1,
                              DspScheduleBoundary::through,
                              1,
                              {},
                              {},
                              external) == 0 &&
              rig.machine.cpu().state().x == 0 && delivered &&
              rig.machine.bus().timers().read(timer_count) == 7 &&
              schedule.completed_external_events() == 1,
          "The actual external event proceeds before the stalled native load can retire");
  const auto saved_owner = rig.machine.checkpoint();
  const auto saved_schedule = schedule.checkpoint(rig.machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    if (replay != 0) {
      rig.machine.restore(saved_owner);
      schedule.restore(saved_schedule, rig.machine);
    }
    require(schedule.advance_to(rig.machine,
                                start + plan.duration_ticks,
                                DspScheduleBoundary::through,
                                1,
                                {},
                                {},
                                external) == 1 &&
                rig.machine.cpu().state().x == data &&
                rig.machine.bus().timers().read(timer_count) == 7 &&
                schedule.completed_external_events() == 1,
            "The elapsed native deadline and consumed external edge survive coordinated restore");
  }
}

void interrupt_entry_stays_unqualified() {
  Rig rig({"move #>$c00000,sr", "nop"});
  rig.step(0);
  constexpr unsigned priority = 0xfffffe;
  constexpr unsigned timer_priority = 0x100;
  constexpr unsigned vector = 0x54;
  constexpr unsigned compare_flag = 0x200000;
  rig.machine.bus().write({.space = DspSpace::x, .address = priority, .value = timer_priority});
  rig.machine.bus().write({.address = vector});
  rig.machine.bus().write({.address = vector + 1});
  auto &timer = rig.machine.bus().timers();
  timer.write({.address = timer_load});
  timer.write({.address = timer_compare});
  timer.write({.address = timer_control, .value = 5});
  timer.advance_clk2(1);
  const auto saved = rig.machine.checkpoint();
  const auto before = rig.machine.cpu().debug_state();
  const auto ticks = rig.machine.run_clock().elapsed_ticks();
  require(failure([&] {
            static_cast<void>(rig.machine.plan_step());
          }).contains("Unqualified interrupt"),
          "Nominal AGU history cannot supply interrupt-entry pipeline placement");
  require(!rig.machine.timing_faulted(), "Rejected IRQ preview leaves its actual request pending");
  require(failure([&] { rig.machine.cpu().step(); }).contains("Unqualified interrupt") &&
              rig.machine.timing_faulted() && rig.machine.cpu().debug_state() == before &&
              rig.machine.run_clock().elapsed_ticks() == ticks &&
              rig.machine.bus().timers().read(timer_control, compare_flag) == compare_flag,
          "Unknown IRQ placement rejects before vector execution, acknowledgement or clocks");
  rig.machine.restore(saved);
  require(!rig.machine.timing_faulted(),
          "Trusted recovery retains the unacknowledged IRQ boundary");
}

void unsupported_sequences_reject_before_execution() {
  for (const auto &word : std::array<std::string, 3>{"lua (r0)+,r1", "tne r0,r1", "bset #0,r0"}) {
    Rig rig({word});
    const auto before = rig.machine.cpu().debug_state();
    const auto ticks = rig.machine.run_clock().elapsed_ticks();
    require(failure([&] { static_cast<void>(rig.machine.plan_step()); }).contains("Unqualified"),
            "Unqualified sequence preview must reject");
    require(rig.machine.cpu().debug_state() == before && !rig.machine.timing_faulted(),
            "A rejected pure preview cannot fault or alter the actual CPU");
    const auto saved = rig.machine.checkpoint();
    const auto text = failure([&] { rig.machine.cpu().step(); });
    require(
        text.contains("pc=") && text.contains("omr=") && rig.machine.timing_faulted() &&
            rig.machine.run_clock().elapsed_ticks() == ticks &&
            rig.machine.cpu().state().r == before.cpu.r,
        "Actual rejection precedes native effects and time; recovery needs a trusted checkpoint");
    rig.machine.restore(saved);
    require(!rig.machine.timing_faulted(),
            "Trusted restoration recovers the pre-execution history");
  }
}
}

int main() {
  try {
    literal_pair_matrix();
    clock_distance_and_forwarding();
    parallel_consumers_and_explicit_destinations();
    other_move_destinations_and_sources();
    timer_and_checkpoint();
    latched_repeat_and_reset();
    scheduler_uses_elapsed_deadline();
    unsupported_sequences_reject_before_execution();
    interrupt_entry_stays_unqualified();
    std::cout << "Nominal AGU clocks, actual native execution, timers and restores agree.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
