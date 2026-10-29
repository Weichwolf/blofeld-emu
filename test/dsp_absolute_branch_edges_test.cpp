// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_reference_execution.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
using namespace blofeld;
constexpr unsigned extal_hz = 12000000;
constexpr unsigned timebase_hz = 540000000;
constexpr unsigned boot = 0xff0000;
constexpr unsigned ram = 0x100;
constexpr unsigned ordinary_target = 0x180;
constexpr unsigned maximum_short_target = 0xfff;
constexpr unsigned status = 0xc00300;
constexpr unsigned switched_omr = 0x600385; // DSP56371 MS=1, MSW=11; P extends beyond $fff.
constexpr unsigned omr_immediate = 0x05f43a;
constexpr unsigned high_branch_offset = 7;
constexpr unsigned status_immediate = 0x05f439;
constexpr unsigned jump_extension = 0x0af080;
constexpr unsigned short_absolute = 0x0e0000; // FM13-80:00001110 CCCC aaaaaaaaaaaa.
constexpr unsigned condition_shift = 12;
constexpr unsigned conditions = 16;
constexpr unsigned flag_states = 256;
constexpr unsigned native_cost = 4; // SDK diagnostic units; not physical clocks.
constexpr unsigned carry = 1, overflow = 2, zero = 4, negative = 8;
constexpr unsigned unnormalized = 0x10, extension = 0x20, limit = 0x40;
unsigned cases = 0;

struct Input {
  unsigned condition;
  unsigned ccr;
  unsigned target;
  bool high_source = false;
};

void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

// FM12-17/18: each row is independent of the interpreter's condition decoder.
bool taken(const Input input) {
  const auto ccr = input.ccr;
  const bool c = (ccr & carry) != 0;
  const bool v = (ccr & overflow) != 0;
  const bool z = (ccr & zero) != 0;
  const bool n = (ccr & negative) != 0;
  const bool u = (ccr & unnormalized) != 0;
  const bool e = (ccr & extension) != 0;
  const bool l = (ccr & limit) != 0;
  const std::array rows{!c,
                        n == v,
                        !z,
                        !n,
                        !z && (u || e),
                        !e,
                        !l,
                        !z && n == v,
                        c,
                        n != v,
                        z,
                        n,
                        z || (!u && !e),
                        e,
                        l,
                        z || n != v};
  return rows.at(input.condition);
}

DspDebugState prepare(DspMachine &machine, const Input input) {
  const auto opcode = short_absolute | (input.condition << condition_shift) | input.target;
  if (input.high_source) {
    const std::array code{omr_immediate,
                          switched_omr,
                          status_immediate,
                          status | input.ccr,
                          0U,
                          0U,
                          0U,
                          opcode,
                          0U,
                          0U,
                          0U,
                          0U,
                          0U};
    machine.bus().install_rom({.start = boot, .words = code});
  } else {
    const std::array code{omr_immediate,
                          switched_omr,
                          status_immediate,
                          status | input.ccr,
                          0U,
                          0U,
                          0U,
                          jump_extension,
                          ram};
    machine.bus().install_rom({.start = boot, .words = code});
    machine.bus().write({.address = ram, .value = opcode});
    for (unsigned i = 1; i < 16; ++i) { machine.bus().write({.address = ram + i}); }
  }
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  // Native instructions establish OMR; this untimed setup does not spend bus clocks.
  for (unsigned i = 0; i < 5; ++i) { machine.cpu().step(); }
  if (!input.high_source) { machine.cpu().step(); }
  // Explicit standalone fixture precondition, not timed-switch validation.
  machine.bus().advance_instruction_cycles(3);
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == (input.high_source ? boot + high_branch_offset : ram) &&
              before.cpu.sr == (status | input.ccr) && before.cpu.omr == switched_omr,
          "Actual own setup establishes literal source PC and complete CCR");
  return before;
}

DspFetchRunInputs contract() {
  return {.extal_hz = extal_hz,
          .timebase_hz = timebase_hz,
          .timing = DspFetchTiming::reference_cycle_order,
          .execution = DspReferenceExecution::staged_register_boundaries,
          .pipeline = DspFetchPipeline::automatic_memory};
}

void suffix(DspMachine &machine, DspFetchRun &run, const Input input, const DspDebugState &before) {
  auto expected = before;
  expected.cpu.pc = taken(input) ? input.target : before.cpu.pc + 1;
  ++expected.native_instructions;
  expected.native_cycles += native_cost;
  constexpr unsigned cycle_bound = 16;
  while (run.completed_cycles() < cycle_bound) {
    const auto point =
        run.advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available});
    if (!point.pipeline || !point.pipeline->retirement_published) { continue; }
    require(point.pipeline->execute2 && point.pipeline->execute2->address == before.cpu.pc &&
                point.completed_cycles == 7 && machine.cpu().debug_state() == expected,
            "Short absolute edge E2 must match independent full state, cost and period");
    return;
  }
  throw Fault("Own short absolute edge failed to retire within its independent bound");
}

void one(const Input input) {
  DspMachine machine;
  const auto before = prepare(machine, input);
  // Retain the branch itself for self targets; all other own paths are NOPs.
  for (unsigned i = 0; i < 16; ++i) {
    const auto address = input.target + i;
    if (address != before.cpu.pc) { machine.bus().write({.address = address}); }
  }
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  DspFetchRun run(contract(), machine);
  for (unsigned cycle = 0; cycle < 5; ++cycle) {
    static_cast<void>(
        run.advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available}));
  }
  const auto saved = run.checkpoint(machine);
  suffix(machine, run, input, before);
  run.restore(saved, machine);
  suffix(machine, run, input, before);
  ++cases;
}

unsigned seed(const unsigned condition, const bool result) {
  for (unsigned flags = 0; flags < flag_states; ++flags) {
    if (taken({.condition = condition, .ccr = flags, .target = ordinary_target}) == result) {
      return flags;
    }
  }
  throw Fault("Independent condition table must have both true and false inputs");
}
}

int main() {
  try {
    for (unsigned cc = 0; cc < conditions; ++cc) {
      for (const bool result : {false, true}) {
        for (const unsigned target : {0U, ram, ram + 1, maximum_short_target}) {
          for (const bool high : {false, true}) {
            one({.condition = cc, .ccr = seed(cc, result), .target = target, .high_source = high});
          }
        }
      }
    }
    std::cout << "PASS own timed short absolute Jcc edges:cases=" << cases
              << " restored-suffixes=" << cases
              << " conditions=16 predicates=true,false "
                 "absolute12,self,sequential,high-ROM-source;full-state,cost4\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
