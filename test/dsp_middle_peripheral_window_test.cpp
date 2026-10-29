// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/detail/dsp_pipeline.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_memory_form.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"
#include "native_access_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned seed_count = 7;
constexpr unsigned body = boot + (2 * seed_count);
constexpr unsigned ordinary_sr = 0xc00300;
constexpr unsigned sa16 = 0x020000;
constexpr unsigned clear_flags = 0x14;
constexpr unsigned raw_middle = 0x800001;
constexpr unsigned middle_shift = 24;
constexpr unsigned extension_shift = 48;
constexpr unsigned extension_mask = 0xff;
constexpr unsigned unknown_y_middle_store = 0x09cc3c; // FM13-136: A1 to Y:fffffc.
constexpr std::uint64_t initial = 0x55800001abcdef;
constexpr std::array clear{0x200013U, 0x20001bU};
constexpr std::array store{0x08cc3cU, 0x08cd3cU};
constexpr std::array independent_opcodes{
    0x08cc3cU, 0x09cc35U, 0x04cc90U, 0x04cc3fU, 0x08cd3cU, 0x09cd35U, 0x04cd90U, 0x04cd3fU};
constexpr std::array independent_addresses{0xfffffcU, 0xfffff5U, 0xffff90U, 0xffff9fU};

struct Input {
  bool b = false;
  bool same = false;
  bool arithmetic16 = false;
  unsigned gap = 0;
};

unsigned stall(const Input input) {
  return input.same && input.gap == 0 ? 1U : 0U;
}

unsigned store_pc(const Input input) {
  return body + 2 + input.gap;
}

unsigned store_e1(const Input input) {
  return 8 + input.gap + stall(input);
}

unsigned last(const Input input) {
  return 12 + input.gap + stall(input);
}

DspFetchRunInputs contract() {
  auto result = pipeline::contract();
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

DspDebugState prepare(DspMachine &machine, const Input input) {
  DspProgram code;
  for (const auto *prefix : {"a", "b"}) {
    code.immediate(std::string(prefix) + '0', initial & word_mask);
    code.immediate(std::string(prefix) + '1', (initial >> middle_shift) & word_mask);
    code.immediate(std::string(prefix) + '2', (initial >> extension_shift) & extension_mask);
  }
  code.immediate("sr", ordinary_sr | (input.arithmetic16 ? sa16 : 0U));
  code.word(0);
  code.word(clear.at(static_cast<unsigned>(input.same ? input.b : !input.b)));
  for (unsigned gap = 0; gap < input.gap; ++gap) { code.word(0); }
  code.word(store.at(static_cast<unsigned>(input.b)));
  for (unsigned index = 0; index < 32; ++index) { code.word(0); }
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::run(machine, seed_count);
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == body && before.cpu.a == initial && before.cpu.b == initial &&
              before.cpu.sr == (ordinary_sr | (input.arithmetic16 ? sa16 : 0U)) &&
              !machine.bus().once_latch(),
          "Native seeds establish independent raw middle/extension bits before the real stream");
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  return before;
}

struct Oracle {
  DspDebugState expected;
  unsigned stalls = 0;
  unsigned stores = 0;
  unsigned retired = 0;
};

unsigned retired(const Input input, const unsigned period) {
  unsigned count = 0;
  for (unsigned index = 0; index < 6 + input.gap; ++index) {
    if (period >= 7 + index + (index >= 2 + input.gap ? stall(input) : 0U)) { ++count; }
  }
  return count;
}

void step(DspFetchRun &run, DspMachine &machine, const Input input, Oracle &oracle) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(cycle);
  const auto period = static_cast<unsigned>(cycle.completed_cycles);
  require(phase.retirement_published == (retired(input, period) != retired(input, period - 1)),
          "Literal one-word costs and the independent source interlock fix every retirement");
  if (phase.memory_source_stall) {
    ++oracle.stalls;
    require(stall(input) == 1 && period == 8 && phase.execute1 &&
                phase.execute1->address == store_pc(input) && !phase.memory &&
                !phase.execution_captured,
            "Only adjacent same-accumulator E1 waits for the older real E2 result");
  }
  if (period == 8) {
    const bool producer_b = input.same ? input.b : !input.b;
    auto &destination = producer_b ? oracle.expected.cpu.b : oracle.expected.cpu.a;
    destination = 0;
  }
  if (phase.memory) {
    ++oracle.stores;
    require(period == store_e1(input) && phase.execute1 &&
                phase.execute1->address == store_pc(input),
            "The sole actual register MOVEP write occurs at the independent E1 period");
  }
  const auto latch = machine.bus().once_latch();
  const unsigned raw = input.arithmetic16 ? raw_middle >> 8U : raw_middle;
  const unsigned value = input.same ? 0U : raw;
  require(period < store_e1(input) ? !latch : latch == value,
          "Actual OnCE observes raw A1/B1 without full-accumulator limiting or an early write");
  if (phase.retirement_published) {
    require(phase.execute2 && phase.execute2->address == oracle.expected.cpu.pc,
            "Real E2 retains the independently specified NOP/CLR/MOVEP instruction order");
    if (oracle.expected.cpu.pc == body + 1) { oracle.expected.cpu.sr |= clear_flags; }
    ++oracle.expected.cpu.pc;
    ++oracle.expected.native_instructions;
    ++oracle.expected.native_cycles;
    ++oracle.retired;
  }
  require(machine.cpu().debug_state() == oracle.expected,
          "Every period preserves the full literal CPU/flags/stack/counter expectation");
}

void finish(DspFetchRun &run, DspMachine &machine, const Input input, Oracle &oracle) {
  while (run.completed_cycles() < last(input)) { step(run, machine, input, oracle); }
  require(oracle.stalls == stall(input) && oracle.stores == 1 && oracle.retired == 6 + input.gap &&
              run.elapsed_ticks() == last(input) * pipeline::period,
          "The complete window has exactly its independent source stall and once-only port write");
}

void one(const Input input) {
  DspMachine machine;
  Oracle oracle{.expected = prepare(machine, input)};
  DspFetchRun run(contract(), machine);
  finish(run, machine, input, oracle);
}

void restores(const Input input) {
  for (unsigned cut = 6; cut <= 11; ++cut) {
    DspMachine machine;
    Oracle oracle{.expected = prepare(machine, input)};
    DspFetchRun run(contract(), machine);
    while (run.completed_cycles() < cut) { step(run, machine, input, oracle); }
    const auto saved = run.checkpoint(machine);
    const auto prefix = oracle;
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      oracle = prefix;
      finish(run, machine, input, oracle);
    }
  }
}

void decode_checks() {
  for (unsigned index = 0; index < independent_opcodes.size(); ++index) {
    const auto opcode = independent_opcodes[index];
    const auto form = decode_memory_stage(opcode);
    const auto port = index % 4;
    const auto space = port == 1 || port == 3 ? DspSpace::y : DspSpace::x;
    require(form && form->peripheral && form->peripheral_register &&
                form->peripheral->space == space &&
                form->peripheral->address == independent_addresses[port] &&
                middle_accumulator_peripheral_store(opcode),
            "All eight independent Freescale encodings retain their exact I/O map/address");
    for (unsigned producer = 0; producer < clear.size(); ++producer) {
      const auto mask = memory_source_dependency(
          {.producer = {.value = clear[producer]}, .consumer = {.value = opcode}});
      const unsigned source_mask = producer == 0 ? 1U : 2U;
      require(mask == (producer == index / 4 ? source_mask : 0U),
              "The independently encoded CLR source selects only its actual accumulator hazard");
    }
    require(!middle_accumulator_peripheral_store(opcode | 0x1000000U),
            "Out-of-word encodings cannot alias a qualified register source");
  }
  for (const auto *text : {"movep a,x:<<$fffffc",
                           "movep b,x:<<$fffffc",
                           "movep a0,x:<<$fffffc",
                           "movep b0,x:<<$fffffc",
                           "movep a2,x:<<$fffffc",
                           "movep b2,x:<<$fffffc",
                           "movep ssh,x:<<$fffffc",
                           "movep sr,x:<<$fffffc",
                           "movep x:<<$fffffc,a1"}) {
    const auto words = checkpoint::assemble(text);
    require(words.size() == 1 && !middle_accumulator_peripheral_store(words.front()) &&
                !decode_memory_stage(words.front()),
            "Other parts, full accumulators, stack/control sources and loads remain unqualified");
  }
}

void strict_failure() {
  DspMachine machine;
  DspProgram code;
  code.word(unknown_y_middle_store);
  for (unsigned index = 0; index < 16; ++index) { code.word(0); }
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  const auto before = machine.cpu().debug_state();
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  DspFetchRun run(contract(), machine);
  pipeline::clocks(run, machine, 5);
  const auto saved = run.checkpoint(machine);
  std::string first;
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    const auto error = native_access::fault(
        [&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
    require(error.contains("write Y:0xfffffc") && error.contains("pc=") && error.contains("omr=") &&
                run.faulted() && run.completed_cycles() == 5 &&
                machine.cpu().debug_state() == before && !machine.bus().once_latch(),
            "The genuine raw-middle peripheral write retains strict native context/quarantine");
    if (replay == 0) { first = error; }
    require(error == first, "Joint restoration reproduces the exact unknown-port failure");
  }
}

void arithmetic16_refused() {
  for (const bool b : {false, true}) {
    DspMachine machine;
    const auto before = prepare(machine, {.b = b, .arithmetic16 = true});
    DspFetchRun run(contract(), machine);
    const auto error = native_access::fault(
        [&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
    require(error.contains("Unqualified automatic DSP mode/interrupt") &&
                error.contains("sr=0xc20300") && error.contains("pc=0xff000e") && run.faulted() &&
                run.completed_cycles() == 0 && run.elapsed_ticks() == 0 &&
                machine.cpu().debug_state() == before && !machine.bus().once_latch(),
            "The automatic reference profile retains its explicit SA16 frontier; native controls "
            "qualify raw SA16 bus semantics separately");
  }
}
}

int main() {
  try {
    decode_checks();
    strict_failure();
    arithmetic16_refused();
    for (const bool b : {false, true}) {
      for (const bool same : {false, true}) {
        for (unsigned gap = 0; gap < 4; ++gap) { one({.b = b, .same = same, .gap = gap}); }
        restores({.b = b, .same = same});
      }
    }
    std::cout << "16 independent raw-middle MOVEP windows,24 cuts/48 suffixes,2 SA16 refusals "
                 "passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
