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
constexpr unsigned short_branch = 0x050400;
constexpr unsigned condition_shift = 12;
constexpr unsigned target_displacement = 16;
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

unsigned setup(DspMachine &machine, const Input input) {
  DspProgram program;
  program.immediate("b", 0);
  program.immediate("sr", masked_status | input.ccr);
  const auto entry = program.pc();
  if (input.producer == 3) {
    program.emit("clr b");
  } else if (input.producer != 0) {
    program.emit(input.producer == 1 ? "cmp #>0,b" : "cmp #>1,b");
  }
  const auto branch = program.pc();
  // FM13-18 literal signed9 encoding. Positive16 occupies only its low field.
  program.word(short_branch | (input.condition << condition_shift) | target_displacement);
  program.emit("movep #>" + number(false_marker) + ",x:<<" + number(gdb));
  for (unsigned i = 3; i < target_displacement; ++i) { program.emit("nop"); }
  check(program.pc() == branch + target_displacement,
        "Own literal target must match instruction sizes");
  program.emit("movep #>" + number(true_marker) + ",x:<<" + number(gdb));
  for (unsigned i = 0; i < budget; ++i) { program.emit("nop"); }
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  // Only the seed prologue is untimed. CMP/branch/output execute automatically.
  checkpoint::until(machine, entry);
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return branch;
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

void one(const Input input) {
  DspMachine machine;
  const auto branch = setup(machine, input);
  DspFetchRun clock({.extal_hz = extal,
                     .timebase_hz = timebase,
                     .timing = DspFetchTiming::reference_cycle_order,
                     .execution = DspReferenceExecution::staged_register_boundaries,
                     .pipeline = DspFetchPipeline::automatic_memory},
                    machine);
  unsigned ccr = input.ccr;
  if (input.producer != 0) {
    ccr = input.ccr & (sticky | limit);
    ccr |= input.producer != 2 ? unnormalized | zero : unnormalized | negative | carry;
  }
  const bool taken = expected(input, ccr);
  const unsigned target = branch + (taken ? target_displacement : 1);
  for (unsigned cycle = 0; cycle < budget; ++cycle) {
    const auto point =
        clock.advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available});
    if (!point.pipeline) { throw Fault("Own conditional branch lost its automatic stages"); }
    const auto pipeline = *point.pipeline;
    if (!pipeline.retirement_published || !pipeline.execute2 ||
        pipeline.execute2->address != branch) {
      continue;
    }
    check(machine.cpu().state().pc == target && machine.cpu().state().sr == (masked_status | ccr),
          "Native Bcc E2 differs from independent PC/CCR:condition=" +
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
      for (const unsigned producer : {1U, 2U, 3U}) {
        one({.condition = condition,
             .ccr = extension | limit | sticky,
             .producer = producer,
             .checkpoints = true});
      }
    }
    std::cout << "conditional-cases=" << cases << " restored-suffixes=" << restored
              << " assertions=" << assertions << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
