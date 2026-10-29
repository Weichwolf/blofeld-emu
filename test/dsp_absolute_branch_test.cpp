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
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned extal = 12000000;
constexpr unsigned timebase = 540000000;
constexpr unsigned budget = 96;
constexpr unsigned condition_count = 16;
constexpr unsigned status_count = 256;
constexpr unsigned short_branch = 0x0e0000; // FM13-80: short absolute Jcc.
constexpr unsigned condition_shift = 12;
constexpr unsigned absolute_target = 0x180;
constexpr unsigned native_branch_cost = 4; // SDK diagnostic units, not silicon clocks.
constexpr unsigned gdb = 0xfffffc;
constexpr unsigned false_marker = 0x123456;
constexpr unsigned true_marker = 0x654321;
unsigned cases = 0;
unsigned assertions = 0;
unsigned restored = 0;

struct Input {
  unsigned condition = 0;
  unsigned ccr = 0;
  unsigned producer = 0;
  bool checkpoints = false;
};

void check(const bool value, const std::string &message) {
  ++assertions;
  if (!value) { throw Fault(message); }
}

// Independent existing FM12-17/18 truth-matrix representation: all16 rows
// are declared directly, rather than using the implementation's complement bit.
bool expected(const Input input, const unsigned ccr) {
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

struct Setup {
  unsigned branch;
  DspDebugState before;
};

Setup setup(DspMachine &machine, const Input input) {
  DspProgram program;
  program.immediate("b", 0);
  if (input.producer >= 4) {
    program.immediate("a", input.producer == 4 ? 0U : input.producer - 3);
  }
  program.immediate("sr", masked_status | input.ccr);
  const auto entry = program.pc();
  if (input.producer >= 4) {
    program.emit("cmp #2,a");
  } else if (input.producer == 3) {
    program.emit("clr b");
  } else if (input.producer != 0) {
    program.emit(input.producer == 1 ? "cmp #>0,b" : "cmp #>1,b");
  }
  const auto branch = program.pc();
  // The target is low RAM even though the branch is in high ROM.
  program.word(short_branch | (input.condition << condition_shift) | absolute_target);
  program.emit("movep #>" + number(false_marker) + ",x:<<" + number(gdb));
  for (unsigned i = 0; i < budget; ++i) { program.emit("nop"); }
  const auto target_store =
      checkpoint::assemble(("movep #>" + number(true_marker) + ",x:<<" + number(gdb)).c_str());
  checkpoint::ram(machine.bus(), absolute_target, target_store);
  for (auto i = static_cast<unsigned>(target_store.size()); i < budget; ++i) {
    machine.bus().write({.address = absolute_target + i});
  }
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  // Only the seed prologue is untimed. CMP/branch/output execute automatically.
  checkpoint::until(machine, entry);
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return {.branch = branch, .before = machine.cpu().debug_state()};
}

void suffix(DspMachine &machine, DspFetchRun &clock, const unsigned marker) {
  for (unsigned cycle = 0; cycle < budget; ++cycle) {
    const auto point =
        clock.advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available});
    if (!point.bus.memory || point.bus.memory->data.target.address != gdb) { continue; }
    check(point.bus.memory->data.value == marker && machine.bus().once_latch() == marker,
          "The genuine native target store must match the independent CCR decision");
    return;
  }
  throw Fault("Own conditional suffix did not reach its actual bounded native output");
}

unsigned expected_ccr(const Input input) {
  if (input.producer == 0) { return input.ccr; }
  // Independent own operands: B=0 CMP0/CMP1/CLR; A=0/2/3 CMP2.
  constexpr std::array outcomes{0U,
                                unnormalized | zero,
                                unnormalized | negative | carry,
                                unnormalized | zero,
                                unnormalized | negative | carry,
                                unnormalized | zero,
                                unnormalized};
  return (input.ccr & (sticky | limit)) | outcomes.at(input.producer);
}

void one(const Input input) {
  DspMachine machine;
  const auto setup_state = setup(machine, input);
  const auto branch = setup_state.branch;
  DspFetchRun clock({.extal_hz = extal,
                     .timebase_hz = timebase,
                     .timing = DspFetchTiming::reference_cycle_order,
                     .execution = DspReferenceExecution::staged_register_boundaries,
                     .pipeline = DspFetchPipeline::automatic_memory},
                    machine);
  const auto ccr = expected_ccr(input);
  const bool taken = expected(input, ccr);
  const unsigned target = taken ? absolute_target : branch + 1;
  auto expected_state = setup_state.before;
  expected_state.cpu.pc = target;
  expected_state.cpu.sr = masked_status | ccr;
  expected_state.native_instructions += input.producer == 0 ? 1U : 2U;
  expected_state.native_cycles += native_branch_cost;
  if (input.producer != 0) { expected_state.native_cycles += input.producer >= 3 ? 1U : 2U; }
  for (unsigned cycle = 0; cycle < budget; ++cycle) {
    const auto point =
        clock.advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available});
    if (!point.pipeline) { throw Fault("Own conditional branch lost its automatic stages"); }
    const auto pipeline = *point.pipeline;
    if (!pipeline.retirement_published || !pipeline.execute2 ||
        pipeline.execute2->address != branch) {
      continue;
    }
    check(machine.cpu().debug_state() == expected_state,
          "Native Jcc E2 differs from independent complete state/counters:condition=" +
              std::to_string(input.condition) + " seed=" + hex(input.ccr, 2) + " producer=" +
              std::to_string(input.producer) + " pc=" + hex(machine.cpu().state().pc, 6) +
              " expected-pc=" + hex(target, 6) + " sr=" + hex(machine.cpu().state().sr, 6) +
              " expected-sr=" + hex(masked_status | ccr, 6));
    const auto saved = clock.checkpoint(machine);
    const auto marker = taken ? true_marker : false_marker;
    suffix(machine, clock, marker);
    if (input.checkpoints) {
      clock.restore(saved, machine);
      suffix(machine, clock, marker);
      ++restored;
    }
    ++cases;
    return;
  }
  throw Fault("Own conditional branch did not reach its actual bounded native E2");
}
}

int main() {
  try {
    for (unsigned condition = 0; condition < condition_count; ++condition) {
      for (unsigned ccr = 0; ccr < status_count; ++ccr) {
        one({.condition = condition, .ccr = ccr});
      }
      for (const unsigned producer : {1U, 2U, 3U, 4U, 5U, 6U}) {
        one({.condition = condition,
             .ccr = extension | limit | sticky,
             .producer = producer,
             .checkpoints = true});
      }
    }
    std::cout << "absolute-conditional-cases=" << cases << " restored-suffixes=" << restored
              << " assertions=" << assertions << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
