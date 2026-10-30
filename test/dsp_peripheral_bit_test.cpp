// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_cycle.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_peripheral_bit_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <exception>
#include <iostream>
#include <optional>
#include <string_view>

namespace blofeld::test::peripheral_bit {
namespace {
void native_one(const Input input) {
  DspMachine machine;
  const auto before = prepare(machine, input, false);
  auto expected = before;
  ++expected.cpu.pc;
  expected.cpu.sr = status | static_cast<unsigned>(input.old);
  ++expected.native_instructions;
  expected.native_cycles += 2;
  unsigned callbacks = 0;
  machine.cpu().step({.observe = [&](const DspNativeAccess &access) {
    if (access.kind != DspNativeAccessKind::bit) { return; }
    ++callbacks;
    require(access.pc == body && access.bit && access.bit->index == input.bit &&
                access.bit->operation == operations.at(input.operation).kind &&
                access.value == (access.stage == DspNativeAccessStage::after
                                     ? std::optional(static_cast<unsigned>(input.old))
                                     : std::nullopt),
            "Native bit callbacks carry the real selected operation and previous bit");
  }});
  const auto port = ports.at(input.port);
  require(machine.cpu().debug_state() == expected && callbacks == 2 &&
              machine.bus().read({.space = port.space, .address = port.address}) == value(input),
          "FM bit truth table preserves every other GPIO/CPU/stack/CCR bit and costs two clocks");
}

void footprint(const DspReferenceCycleResult &cycle, const DspMemoryOperation expected) {
  unsigned count = 0;
  for (unsigned index = 0; index < cycle.core_access_count; ++index) {
    const auto &access = cycle.core_accesses.at(index);
    if (access.phase != DspMemoryPhase::peripheral_bit) { continue; }
    require(access.operation == expected,
            "The phase reserves its independently expected bus operation");
    ++count;
  }
  require(count == 1, "Each RMW phase has exactly one actual peripheral operand");
}

struct Oracle {
  DspDebugState expected;
  unsigned effects = 0;
  unsigned retired = 0;
};

void step(DspFetchRun &run, DspMachine &machine, const Input input, Oracle &oracle) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(cycle);
  const auto period = cycle.completed_cycles;
  const auto port = ports.at(input.port);
  require(!phase.memory_source_stall,
          "A literal fixed-port bit instruction has no accumulator-source interlock");
  if (period == 5) {
    require(phase.ag1_address && phase.ag1_address->kind == DspAddressStageKind::peripheral_bit &&
                !phase.ag1_address->extension && !phase.ag1_address->pointer_update,
            "Actual AG1 records the fixed bit operand without a fictitious address register");
  }
  const bool mutating = input.operation != 3;
  if (phase.memory) {
    ++oracle.effects;
    const auto &data = phase.memory->data;
    require(period == 7 && data.kind == DspNativeAccessKind::bit && data.bit &&
                data.bit->index == input.bit &&
                data.bit->operation == operations.at(input.operation).kind && data.pc == body + 1 &&
                data.ordinal == 1 && phase.memory->bit_read == mutating &&
                (mutating ? !data.value : data.value == static_cast<unsigned>(input.old)) &&
                data.stage ==
                    (mutating ? DspNativeAccessStage::before : DspNativeAccessStage::after),
            "E1 records one actual native RMW read or completes the read-only BTST callback");
    footprint(cycle.bus, DspMemoryOperation::read);
  }
  if (cycle.bus.bit_write) {
    const auto &data = *cycle.bus.bit_write;
    require(mutating && period == 8 && data.pc == body + 1 && data.bit &&
                data.bit->index == input.bit &&
                data.bit->operation == operations.at(input.operation).kind &&
                data.value == static_cast<unsigned>(input.old) &&
                data.stage == DspNativeAccessStage::after,
            "E2 writes once and supplies the genuine captured E1 old bit to native Carry");
    footprint(cycle.bus, DspMemoryOperation::write);
  }
  const bool retires = period == 7 || period == 8 || (period >= 10 && period <= 15);
  require(phase.retirement_published == retires,
          "Literal NOP/bit/NOP costs place every retirement and the one real Decode hold");
  if (retires) {
    require(phase.execute2 && phase.execute2->address == oracle.expected.cpu.pc,
            "Only the actual own sequential program words retire");
    const bool bit = oracle.retired == 1;
    if (bit) { oracle.expected.cpu.sr = status | static_cast<unsigned>(input.old); }
    ++oracle.expected.cpu.pc;
    ++oracle.expected.native_instructions;
    oracle.expected.native_cycles += bit ? 2U : 1U;
    ++oracle.retired;
  }
  require(machine.cpu().debug_state() == oracle.expected &&
              machine.bus().read({.space = port.space, .address = port.address}) ==
                  (period >= 8 ? value(input) : seed(input)),
          "Each period matches the independent full CPU/stack/counter/GPIO expectation");
}

void finish(DspFetchRun &run, DspMachine &machine, const Input input, Oracle &oracle) {
  while (run.completed_cycles() < 15) { step(run, machine, input, oracle); }
  require(oracle.retired == 8 && oracle.effects == 1 &&
              run.elapsed_ticks() == 15 * pipeline::period,
          "The complete quiet interval is six NOP clocks plus the independent two-clock bit cost");
}

void window(const Input input) {
  DspMachine machine;
  Oracle oracle{.expected = prepare(machine, input, true)};
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  DspFetchRun run(contract(), machine);
  finish(run, machine, input, oracle);
}

void restores(const Input input) {
  for (unsigned cut = 4; cut <= 10; ++cut) {
    DspMachine machine;
    Oracle oracle{.expected = prepare(machine, input, true)};
    static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
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
}

void native_checks() {
  constexpr unsigned physical_bits = 37;
  constexpr unsigned truth_cases = 16;
  unsigned cases = 0;
  for (unsigned port = 0; port < ports.size(); ++port) {
    for (unsigned operation = 0; operation < operations.size(); ++operation) {
      for (unsigned bit = 0; bit < ports[port].bits; ++bit) {
        for (const bool old : {false, true}) {
          for (const bool carry_in : {false, true}) {
            native_one({.port = port,
                        .operation = operation,
                        .bit = bit,
                        .old = old,
                        .carry_in = carry_in});
            ++cases;
          }
        }
      }
    }
  }
  require(cases == physical_bits * truth_cases,
          "All37 physical GPIO bit positions have sixteen independent truth cases");
  std::cout << cases << " native GPIO bit/carry cases passed\n";
}

void pipeline_checks() {
  for (unsigned port = 0; port < ports.size(); ++port) {
    for (unsigned operation = 0; operation < operations.size(); ++operation) {
      for (const unsigned bit : {0U, ports[port].bits - 1}) {
        for (const bool old : {false, true}) {
          for (const bool carry_in : {false, true}) {
            window({.port = port,
                    .operation = operation,
                    .bit = bit,
                    .old = old,
                    .carry_in = carry_in});
          }
          restores(
              {.port = port, .operation = operation, .bit = bit, .old = old, .carry_in = !old});
        }
      }
    }
  }
  std::cout << "128 bit windows,448 cuts/896 independent suffixes passed\n";
}
}

int main(const int argc, char **argv) {
  try {
    if (argc != 2) { throw blofeld::Fault("Select native, pipeline or guards"); }
    const std::string_view mode(argv[1]);
    if (mode == "native") {
      blofeld::test::peripheral_bit::native_checks();
    } else if (mode == "pipeline") {
      blofeld::test::peripheral_bit::pipeline_checks();
    } else if (mode == "guards") {
      blofeld::test::peripheral_bit::guard_checks();
    } else {
      throw blofeld::Fault("Unknown bit test selection");
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
