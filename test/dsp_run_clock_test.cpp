// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_pll.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/dsp_timers.hpp"
#include "dsp56kEmu/opcodecycles.h"
#include "dsp56kEmu/opcodeinfo.h"
#include "dsp56kEmu/opcodes.h"
#include "dsp56kEmu/opcodetypes.h"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
using namespace blofeld::test;
namespace fixture = blofeld::test::checkpoint;
constexpr unsigned extal_hz = 12000000;
constexpr unsigned timebase_hz = 540000000;
constexpr unsigned first_stage_pctl = 0x02e112;
constexpr unsigned csr0 = 0xffff8f;
constexpr unsigned count0 = 0xffff8c;
constexpr unsigned compare0 = 0xffff8d;
constexpr unsigned load0 = 0xffff8e;
constexpr unsigned tcf = 0x200000;
constexpr unsigned switched = 0x600085;
constexpr unsigned long_move_ticks = 90;

blofeld::DspRunContract contract() {
  return {.clock = {.extal_hz = extal_hz,
                    .timebase_hz = timebase_hz,
                    .switch_timing = blofeld::DspClockSwitch::reference_instruction_boundary},
          .timing = blofeld::DspRunTiming::native_internal_estimates};
}

void initialize(blofeld::DspMachine &machine, const DspProgram &program) {
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
}

void configure_timer(blofeld::DspBus &bus) {
  bus.timers().write({.address = csr0, .value = 0});
  bus.timers().write({.address = load0, .value = 7});
  bus.timers().write({.address = compare0, .value = 9});
  bus.timers().write({.address = csr0, .value = 7});
}

void counts(const blofeld::DspMachine &machine,
            const std::uint64_t cycles,
            const std::uint64_t ticks) {
  require(machine.clocked() && !machine.timing_faulted() &&
              machine.run_clock().native_cycle_units() == cycles &&
              machine.cpu().debug_state().native_cycles == cycles &&
              machine.run_clock().elapsed_ticks() == ticks &&
              machine.run_clock().timer_clk2_edges() == cycles / 2,
          "Native SDK units/explicit ticks mismatch: actual=" +
              std::to_string(machine.run_clock().native_cycle_units()) + "/" +
              std::to_string(machine.run_clock().elapsed_ticks()) +
              " expected=" + std::to_string(cycles) + "/" + std::to_string(ticks));
}

void test_literal_ratio_latch_and_fractional_timer_phase() {
  blofeld::DspBus bus;
  bus.pll().observe_pinit(false);
  bus.reset();
  configure_timer(bus);
  blofeld::DspRunClock clock(contract().clock);
  const blofeld::DspRunContext context{.pc = boot, .omr = 5};
  clock.begin_step(bus, context);
  bus.pll().write(first_stage_pctl);
  clock.finish_step(bus, 3);
  constexpr unsigned old_rate_ticks = 135;
  require(clock.elapsed_ticks() == old_rate_ticks && clock.native_cycle_units() == 3 &&
              bus.timers().read(count0) == 7,
          "The committing command uses the prior 12MHz reference rate and only one CLK/2 edge");
  const auto frozen_clock = clock;
  const auto frozen_bus = bus;
  for (unsigned replay = 0; replay < 2; ++replay) {
    clock = frozen_clock;
    bus = frozen_bus;
    clock.begin_step(bus, context);
    clock.finish_step(bus, 1);
    constexpr unsigned next_ticks = 145;
    require(clock.elapsed_ticks() == next_ticks && clock.timer_clk2_edges() == 2 &&
                bus.timers().read(count0) == 8,
            "The next reference unit uses ratio9/2 and retains the odd divider phase on copy");
    clock.begin_step(bus, context);
    clock.finish_step(bus, 2);
    constexpr unsigned final_ticks = 165;
    require(clock.elapsed_ticks() == final_ticks && clock.native_cycle_units() == 6 &&
                bus.timers().read(count0) == 9 && (bus.timers().read(csr0, tcf) & tcf) != 0,
            "The third actual CLK/2 edge reaches the independent timer compare exactly");
  }
}

void test_independent_nominal_addressing_costs() {
  struct Case {
    const char *source;
    dsp56k::Instruction instruction;
    unsigned units;
  };

  // FM A-1/A-6/A-7: post-update by N has no pre-update penalty;
  // long absolute/immediate select separate columns; P-space MOVEP starts at six.
  constexpr std::array cases{
      Case{.source = "move #>$123456,x0", .instruction = dsp56k::Movex_ea, .units = 2},
      Case{.source = "move x:>$100,x0", .instruction = dsp56k::Movex_ea, .units = 2},
      Case{.source = "move x0,x:>$100", .instruction = dsp56k::Movex_ea, .units = 2},
      Case{.source = "move x:(r0),x0", .instruction = dsp56k::Movex_ea, .units = 1},
      Case{.source = "move x:(r0)+,x0", .instruction = dsp56k::Movex_ea, .units = 1},
      Case{.source = "move x:(r0)-,x0", .instruction = dsp56k::Movex_ea, .units = 1},
      Case{.source = "move x:(r0)+n0,x0", .instruction = dsp56k::Movex_ea, .units = 1},
      Case{.source = "move x:(r0)-n0,x0", .instruction = dsp56k::Movex_ea, .units = 1},
      Case{.source = "move x:(r0+n0),x0", .instruction = dsp56k::Movex_ea, .units = 2},
      Case{.source = "move x:-(r0),x0", .instruction = dsp56k::Movex_ea, .units = 2},
      Case{.source = "move #>$123456,lc", .instruction = dsp56k::Movec_ea, .units = 2},
      Case{.source = "movep x0,x:<<$fffffd", .instruction = dsp56k::Movep_Spp, .units = 1},
      Case{.source = "movep #>$02e112,x:<<$fffffd", .instruction = dsp56k::Movep_ppea, .units = 2},
      Case{.source = "movep x:>$100,x:<<$fffffd", .instruction = dsp56k::Movep_ppea, .units = 3},
      Case{.source = "movep x:<<$ffffd4,p:(r0)+", .instruction = dsp56k::Movep_eapp, .units = 6},
      Case{.source = "movep x:<<$ffffd4,p:>$100", .instruction = dsp56k::Movep_eapp, .units = 7}};
  const dsp56k::Opcodes opcodes;
  for (const auto &test : cases) {
    const auto words = fixture::assemble(test.source);
    const auto *info = dsp56k::Opcodes::isParallelOpcode(words.front())
                           ? opcodes.findParallelMoveOpcodeInfo(words.front())
                           : opcodes.findNonParallelOpcodeInfo(words.front());
    require(info != nullptr && info->getInstruction() == test.instruction,
            "Own nominal-cost diagnostic must encode its declared instruction form: " +
                std::string(test.source));
    const auto units = dsp56k::calcCycles(test.instruction, boot, words.front(), 0, 0);
    require(units == test.units,
            "Independent nominal addressing cost: " + std::string(test.source) +
                " actual=" + std::to_string(units) + " expected=" + std::to_string(test.units));
  }
  // FM 13-136 low-I/O P: encodings; the SDK assembler lacks this distinct form.
  constexpr unsigned low_io_post = 0x009800;
  constexpr unsigned low_io_absolute = 0x00b000;
  for (const auto opcode : std::array{low_io_post, low_io_absolute}) {
    const auto *info = opcodes.findNonParallelOpcodeInfo(opcode);
    require(info != nullptr && info->getInstruction() == dsp56k::Movep_eaqq,
            "Independently encoded low-I/O P: MOVEP must retain its actual instruction class");
    const unsigned expected = opcode == low_io_post ? 6 : 7;
    require(dsp56k::calcCycles(dsp56k::Movep_eaqq, boot, opcode, 0, 0) == expected,
            "Low-I/O P: MOVEP nominal base and long-absolute penalty follow FM A-7");
  }
}

void test_native_commands_and_trusted_replay() {
  DspProgram program;
  program.immediate("x0", first_stage_pctl); // FM A-6 long-immediate MOVE: two units.
  program.emit("movep x0,x:<<$fffffd");      // FM A-7 register MOVEP: one unit.
  for (unsigned index = 0; index < 3; ++index) { program.emit("nop"); }
  blofeld::DspMachine machine(contract());
  initialize(machine, program);
  configure_timer(machine.bus());
  machine.cpu().step();
  constexpr unsigned move_ticks = 90;
  counts(machine, 2, move_ticks);
  machine.cpu().step();
  constexpr unsigned command_ticks = 135;
  counts(machine, 3, command_ticks);
  require(machine.bus().pll().read() == first_stage_pctl,
          "The native opcode supplies its PCTL operand without a host command injection");
  const auto frozen = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(frozen);
    machine.cpu().step();
    constexpr unsigned first_nop_ticks = 145;
    counts(machine, 4, first_nop_ticks);
    static_cast<void>(fixture::rejects([&] { machine.cpu().reset_spi_boot(); },
                                       "A live clocked owner cannot silently reset its epoch"));
    machine.cpu().step();
    machine.cpu().step();
    constexpr unsigned final_ticks = 165;
    counts(machine, 6, final_ticks);
    require(machine.bus().timers().read(count0) == 9,
            "Native instructions and restored odd CLK/2 phase advance the timer exactly once");
  }
}

void test_native_memory_switch_deadline() {
  DspProgram program;
  program.immediate("omr", switched);
  for (unsigned index = 0; index < 3; ++index) { program.emit("nop"); }
  blofeld::DspMachine machine(contract());
  initialize(machine, program);
  constexpr unsigned physical_x = 0x10000;
  constexpr unsigned alias_p = 0x1000;
  constexpr unsigned cookie = 0x123456;
  machine.bus().write({.space = blofeld::DspSpace::x, .address = physical_x, .value = cookie});
  machine.cpu().step();
  constexpr unsigned command_ticks = 90;
  counts(machine, 2, command_ticks);
  const auto frozen = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(frozen);
    for (unsigned cycle = 0; cycle < 3; ++cycle) {
      static_cast<void>(fixture::rejects(
          [&] { static_cast<void>(machine.bus().read({.address = alias_p})); },
          "The native OMR write cannot complete its switch before three subsequent units"));
      machine.cpu().step();
    }
    constexpr unsigned switched_ticks = 225;
    counts(machine, 5, switched_ticks);
    require(machine.bus().read({.address = alias_p}) == cookie,
            "Native elapsed units settle the delayed map while retaining physical X/P contents");
  }
}

void test_bounded_rep_clock_replay() {
  DspProgram program;
  program.emit("rep #3"); // FM A-8: five units for REP, one for each NOP body.
  program.emit("nop");
  program.emit("nop");
  blofeld::DspMachine machine(contract());
  initialize(machine, program);
  machine.cpu().step();
  constexpr unsigned setup_ticks = 225;
  counts(machine, 5, setup_ticks);
  machine.cpu().step();
  const auto frozen = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(frozen);
    machine.cpu().step();
    machine.cpu().step();
    constexpr unsigned repeated_ticks = 360;
    counts(machine, 8, repeated_ticks);
    machine.cpu().step();
    constexpr unsigned next_ticks = 405;
    counts(machine, 9, next_ticks);
  }
}

void test_unknown_inputs_failure_prefix_and_restore() {
  DspProgram program;
  for (unsigned index = 0; index < 3; ++index) { program.emit("nop"); }
  blofeld::DspMachine machine(contract());
  program.install(machine.bus());
  const auto initial = machine.checkpoint();
  const auto error = fixture::rejects([&] { machine.cpu().step(); }, "Unobserved PINIT must fail");
  require(machine.timing_faulted() && error.contains("pc=0xff0000") &&
              error.contains("omr=0x000305") && error.contains("PINIT") &&
              machine.run_clock().elapsed_ticks() == 0,
          "Unknown reset clock input is not fabricated and preserves full CPU/clock context");
  machine.restore(initial);
  initialize(machine, program);
  machine.bus().timers().write({.address = csr0, .value = 1}); // Load/compare remain unknown.
  const auto before = machine.checkpoint();
  machine.cpu().step(); // An odd cycle has no timer edge yet.
  static_cast<void>(fixture::rejects([&] { machine.cpu().step(); }, "Unknown timer must fail"));
  require(
      machine.timing_faulted() && machine.run_clock().native_cycle_units() == 1 &&
          machine.cpu().debug_state().native_cycles == 2,
      "A rejected peripheral advance retains only its completed time prefix and quarantines CPU");
  static_cast<void>(
      fixture::rejects([&] { static_cast<void>(machine.checkpoint()); },
                       "An incomplete timed step cannot become a trusted checkpoint"));
  machine.restore(before);
  configure_timer(machine.bus());
  machine.cpu().step();
  machine.cpu().step();
  constexpr unsigned recovered_ticks = 90;
  counts(machine, 2, recovered_ticks);
}

void test_contract_compatibility_and_clock_bounds() {
  static_cast<void>(
      fixture::rejects([] { const blofeld::DspMachine invalid(blofeld::DspRunContract{}); },
                       "Timing selection is explicit"));
  auto bad = contract().clock;
  bad.extal_hz = 0;
  static_cast<void>(fixture::rejects([&] { const blofeld::DspRunClock invalid(bad); },
                                     "Zero EXTAL cannot define reference time"));
  blofeld::DspBus bus;
  bus.pll().observe_pinit(false);
  bus.reset();
  constexpr unsigned incompatible_pctl = 0x03e10b;
  bus.pll().write(incompatible_pctl);
  blofeld::DspRunClock clock(contract().clock);
  static_cast<void>(fixture::rejects([&] { clock.begin_step(bus, {}); },
                                     "An unrepresentable PLL period must fail without rounding"));
  require(clock.faulted(), "Rejected clock ratios require coordinated recovery");
  const blofeld::DspMachine timed(contract());
  const auto frozen = timed.checkpoint();
  blofeld::DspMachine untimed;
  static_cast<void>(fixture::rejects([&] { untimed.restore(frozen); },
                                     "Untimed owners cannot silently import a timing contract"));
}

void test_unqualified_interrupt_timing() {
  DspProgram program;
  program.immediate("sr", fixture::ordinary_status);
  program.emit("nop");
  blofeld::DspMachine machine(contract());
  initialize(machine, program);
  machine.bus().write({.space = blofeld::DspSpace::x, .address = fixture::iprp, .value = 0x100});
  machine.bus().timers().write({.address = load0, .value = 1});
  machine.bus().timers().write({.address = compare0, .value = 1});
  machine.bus().timers().write({.address = csr0, .value = 7});
  machine.cpu().step();
  counts(machine, 2, long_move_ticks);
  require(machine.bus().interrupt_request(0).has_value(),
          "A native unmasking instruction and its clock produce an actual pending timer IRQ");
  const auto before = machine.cpu().debug_state();
  const auto frozen = machine.checkpoint();
  const auto error = fixture::rejects([&] { machine.cpu().step(); },
                                      "Unqualified IRQ-entry timing must reject before dispatch");
  require(machine.timing_faulted() && machine.cpu().debug_state() == before &&
              machine.bus().interrupt_request(0).has_value() && error.contains("irq=0x54") &&
              error.contains("Unqualified DSP interrupt"),
          "Timing rejection precedes acknowledgement, unknown vector fetch and native entry");
  machine.restore(frozen);
  counts(machine, 2, long_move_ticks);
}

void test_unqualified_compatibility_timing() {
  DspProgram program;
  program.immediate("sr", compatibility);
  program.emit("nop");
  blofeld::DspMachine machine(contract());
  initialize(machine, program);
  const auto initial = machine.checkpoint();
  machine.cpu().step();
  counts(machine, 2, long_move_ticks);
  const auto before = machine.cpu().debug_state();
  const auto error = fixture::rejects([&] { machine.cpu().step(); },
                                      "Compatibility-mode timing requires a separate contract");
  require(machine.timing_faulted() && machine.cpu().debug_state() == before &&
              error.contains("compatibility-mode timing"),
          "Native SR.SC selection remains visible and stops before unqualified execution");
  machine.restore(initial);
  counts(machine, 0, 0);
}
}

int main() {
  try {
    test_literal_ratio_latch_and_fractional_timer_phase();
    test_independent_nominal_addressing_costs();
    test_native_commands_and_trusted_replay();
    test_native_memory_switch_deadline();
    test_bounded_rep_clock_replay();
    test_unknown_inputs_failure_prefix_and_restore();
    test_contract_compatibility_and_clock_bounds();
    test_unqualified_interrupt_timing();
    test_unqualified_compatibility_timing();
    std::cout
        << "Declared DSP native-cycle reference time, PLL/timer/map phase and restore passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
