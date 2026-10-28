// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_short_load_fixture.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
namespace short_load = blofeld::test::short_load;
constexpr unsigned entry = 0x100, guards = 4, setup = entry + guards, body = setup + 2;
constexpr unsigned loop_flag = 0x8000, do_immediate = 0x060080, source_value = 0x812345;
constexpr unsigned bound = 128, incoming_ccr = 0xff, repeat_three = 0x0603a0;

struct Case {
  unsigned destination = 0, count = 3, size = 1, position = 0;
  bool y = false;

  [[nodiscard]] unsigned move_pc() const { return body + position; }

  [[nodiscard]] unsigned after() const { return body + size; }

  [[nodiscard]] unsigned end_clock() const {
    // Existing DO Decode spacing: D=7, first D=13(single)/12(multi), E2=D+4.
    if (count == 0) { return 15; }
    const auto first_decode = size == 1 ? 13U : 12U;
    return first_decode + 4 + (count * size) - 1;
  }
};

short_load::Case operand(const Case input) {
  return {.destination = input.destination,
          .address = short_load::addresses[2],
          .value = source_value,
          .ccr = incoming_ccr,
          .y = input.y};
}

DspFetchRunInputs contract() {
  auto result = fetch_clock::contract();
  result.execution = DspReferenceExecution::staged_register_boundaries;
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

DspDebugState prepare(DspMachine &machine, const Case input) {
  const auto before = short_load::prepare(machine, operand(input), input.count != 0);
  for (unsigned index = 0; index < 32; ++index) { machine.bus().write({.address = entry + index}); }
  machine.bus().write({.address = setup, .value = do_immediate | (input.count << 8U)});
  machine.bus().write({.address = setup + 1, .value = input.after() - 1});
  machine.bus().write({.address = input.move_pc(), .value = short_load::opcode(operand(input))});
  return before;
}

struct Oracle {
  DspDebugState expected;
  DspDebugState before;
  unsigned addresses = 0, transfers = 0, iterations = 0;
};

void open(Oracle &oracle, const Case input) {
  auto &wanted = oracle.expected;
  const auto before = oracle.before;
  wanted.native_cycles += 5; // FM A1 immediate DO cost, not the pipeline's elapsed counter.
  wanted.stack_high.at(1) = before.cpu.la;
  wanted.stack_low.at(1) = before.cpu.lc;
  wanted.stack_high.at(2) = body;
  wanted.stack_low.at(2) = before.cpu.sr;
  wanted.cpu.pc = input.count == 0 ? input.after() : body;
  if (input.count == 0) { return; }
  wanted.cpu.la = input.after() - 1;
  wanted.cpu.lc = input.count;
  wanted.cpu.sp = 2;
  wanted.sc = 2;
  wanted.cpu.sr |= loop_flag;
}

void retire(Oracle &oracle, const Case input) {
  auto &wanted = oracle.expected;
  const auto pc = wanted.cpu.pc;
  ++wanted.native_instructions;
  if (pc == setup) {
    open(oracle, input);
    return;
  }
  ++wanted.cpu.pc;
  ++wanted.native_cycles;
  if (pc != input.after() - 1) { return; }
  if (++oracle.iterations < input.count) {
    wanted.cpu.lc = input.count - oracle.iterations;
    wanted.cpu.pc = body;
    return;
  }
  wanted.cpu.la = oracle.before.cpu.la;
  wanted.cpu.lc = oracle.before.cpu.lc;
  wanted.cpu.sp = oracle.before.cpu.sp;
  wanted.sc = oracle.before.sc;
  wanted.cpu.sr = oracle.before.cpu.sr;
}

void period(DspFetchRun &run, DspMachine &machine, const Case input, Oracle &oracle) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  if (!cycle.pipeline) { throw Fault("Counted memory loops lost actual pipeline evidence"); }
  const auto &phase = *cycle.pipeline;
  auto &wanted = oracle.expected;
  if (phase.ag2_address) {
    const auto &access = *phase.ag2_address;
    ++oracle.addresses;
    require(access.kind == DspAddressStageKind::absolute_short && access.published &&
                !access.pointer_update && !access.secondary && !access.extension &&
                access.word.address == input.move_pc() &&
                access.update.address == operand(input).address && access.data_access_count == 1,
            "Each loop occurrence publishes one fixed address without R/N/M arithmetic");
  }
  if (phase.memory) {
    ++oracle.transfers;
    const auto &effect = *phase.memory;
    require(phase.execute1 && phase.execute1->address == input.move_pc() &&
                effect.data.kind == DspNativeAccessKind::read &&
                effect.data.pc == input.move_pc() &&
                effect.data.target.address == operand(input).address &&
                effect.data.target.space == (input.y ? DspSpace::y : DspSpace::x) &&
                effect.data.value == source_value && !effect.additional_data,
            "Each real native loop E1 reads the exact literal source once");
    short_load::destination(wanted.cpu, operand(input));
  }
  require(phase.advanced && !phase.memory_source_stall && phase.agu_dependencies == 0,
          "Fixed standalone AA6 adds no source/AGU or indexed-memory reference hold");
  if (phase.retirement_published) {
    require(phase.execute2 && phase.execute2->address == wanted.cpu.pc,
            "Native E2 follows independent prefix,DO,body,backedge and final fallthrough order");
    retire(oracle, input);
  }
  wanted.cpu.stack_pc = wanted.stack_high.at(wanted.cpu.sp);
  wanted.cpu.stack_sr = wanted.stack_low.at(wanted.cpu.sp);
  const auto actual = machine.cpu().debug_state();
  if (actual != wanted) {
    std::cerr << "clock=" << cycle.completed_cycles << " destination=" << input.destination
              << " count=" << input.count << " size=" << input.size
              << " PC=" << hex(actual.cpu.pc, 6) << '/' << hex(wanted.cpu.pc, 6)
              << " cycles=" << actual.native_cycles << '/' << wanted.native_cycles << '\n';
  }
  require(actual == wanted, "Every period agrees with independent full CPU,CCR,stack and costs");
  short_load::memory(machine, operand(input), input.count != 0);
}

void finish(DspFetchRun &run, DspMachine &machine, const Case input, Oracle &oracle) {
  while (run.completed_cycles() < input.end_clock()) { period(run, machine, input, oracle); }
  const auto &wanted = oracle.expected;
  require(input.end_clock() < bound && wanted.cpu.pc == input.after() &&
              wanted.native_instructions == oracle.before.native_instructions + guards + 1 +
                                                (std::uint64_t{input.count} * input.size) &&
              oracle.addresses == input.count && oracle.transfers == input.count &&
              oracle.iterations == input.count,
          "Complete native DO entry and exit matches independent elapsed and iteration totals");
}

void one(const Case input, const bool snapshots) {
  DspMachine machine;
  const auto before = prepare(machine, input);
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  DspFetchRun run(contract(), machine);
  Oracle oracle{.expected = before, .before = before};
  if (!snapshots) {
    finish(run, machine, input, oracle);
    return;
  }
  for (unsigned cut = 0; cut < input.end_clock(); ++cut) {
    const auto state = run.checkpoint(machine);
    const auto saved_oracle = oracle;
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(state, machine);
      oracle = saved_oracle;
      finish(run, machine, input, oracle);
    }
    run.restore(state, machine);
    oracle = saved_oracle;
    period(run, machine, input, oracle);
  }
}

unsigned counts(Case input) {
  constexpr unsigned full_b = 7;
  constexpr unsigned last_modifier = short_load::data_begin - 1;
  constexpr unsigned last_data = short_load::destination_count - 1;
  unsigned total = 0;
  for (const unsigned iterations : {0U, 1U, 3U, 4U}) {
    for (const unsigned size : {1U, 3U}) {
      for (unsigned position = 0; position < size; position += 2) {
        input.count = iterations;
        input.size = size;
        input.position = position;
        one(input, false);
        if ((input.destination == full_b || input.destination == last_modifier ||
             input.destination == last_data) &&
            iterations == 3) {
          one(input, true);
        }
        ++total;
      }
    }
  }
  return total;
}

unsigned matrix() {
  unsigned total = 0;
  for (unsigned target = 0; target < short_load::destination_count; ++target) {
    for (const bool y : {false, true}) { total += counts({.destination = target, .y = y}); }
  }
  return total;
}

void repeat_preserved(DspFetchRun &run,
                      DspMachine &machine,
                      const DspDebugState &before,
                      const short_load::Case sample,
                      const std::string &error) {
  require(error.contains("Unqualified repeated Decode read P:0x000101") && error.contains("pc=") &&
              error.contains("omr=0x000300") && run.faulted() &&
              machine.cpu().debug_state() == before,
          "AA6 remains explicitly excluded from REP before any body or public CPU effect");
  short_load::memory(machine, sample);
}

void repeat_refusal(const short_load::Case sample) {
  DspMachine machine;
  static_cast<void>(short_load::prepare(machine, sample));
  for (unsigned index = 0; index < 16; ++index) { machine.bus().write({.address = entry + index}); }
  machine.bus().write({.address = entry, .value = repeat_three});
  machine.bus().write({.address = entry + 1, .value = short_load::opcode(sample)});
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  DspFetchRun run(contract(), machine);
  for (unsigned index = 0; index < 32; ++index) {
    const auto saved = run.checkpoint(machine);
    const auto before = machine.cpu().debug_state();
    const auto completed = run.completed_cycles();
    const auto dma = machine.bus().last_dma_cycle();
    std::optional<std::string> failure;
    try {
      const auto cycle = run.advance_cycle(machine, fetch_clock::input());
      require(cycle.pipeline && !cycle.pipeline->memory && !cycle.pipeline->ag1_address &&
                  !cycle.pipeline->ag2_address,
              "No excluded REP body reaches address/data stages");
    } catch (const Fault &error) { failure = error.what(); }
    if (!failure) { continue; }
    repeat_preserved(run, machine, before, sample, *failure);
    require(run.completed_cycles() == completed && machine.bus().last_dma_cycle() == dma,
            "Pure REP refusal consumes no additional period or DMA prefix");
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      const auto error = checkpoint::rejects(
          [&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); },
          "Restored excluded AA6 REP body must refuse again");
      repeat_preserved(run, machine, before, sample, error);
      require(run.completed_cycles() == completed && machine.bus().last_dma_cycle() == dma,
              "Both restored refusals preserve the exact admitted prefix");
    }
    return;
  }
  throw Fault("Excluded AA6 REP body unexpectedly executed");
}

void repeat_guards() {
  for (unsigned target = 0; target < short_load::destination_count; ++target) {
    for (const bool y : {false, true}) {
      repeat_refusal({.destination = target,
                      .address = short_load::addresses[2],
                      .value = source_value,
                      .ccr = incoming_ccr,
                      .y = y});
    }
  }
}
}

int main() {
  try {
    constexpr unsigned expected = 864;
    require(matrix() == expected,
            "All36 AA6 destinations execute genuine DO/zero-count/body positions");
    repeat_guards();
    std::cout
        << "864 genuine AA6 DO cases,402cuts/804suffixes,72 REP exclusions/144suffixes passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
