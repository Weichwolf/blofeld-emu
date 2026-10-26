// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"
#include "dsp_spi_prefix_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>

namespace blofeld::test::program_clock {
namespace {
constexpr unsigned direct_program = 0x00a014;
constexpr unsigned increment_program = 0x009814;
constexpr unsigned long_r0 = 0x60f400;
constexpr unsigned producer_destination = 0x100;
constexpr unsigned sentinel = 0x654321;
constexpr unsigned following = 0xa13579;
constexpr unsigned source_control = 0xffff91;
constexpr unsigned source_clock = 0xffff90;
constexpr unsigned fifo_24 = 0x29;
constexpr unsigned hrx = 0xffff94;
constexpr unsigned not_empty = 0x20000;
constexpr unsigned first_retirement = 12;
constexpr unsigned producer_retirement = 16;
constexpr unsigned instruction_cost = 6;
constexpr unsigned source_hazard_spacing = 2;
constexpr unsigned padding = 32;

struct Case {
  unsigned count = 1;
  unsigned gap = 0;
  unsigned payload = 0x123450;
  bool increment = true;
  bool producer = false;
  unsigned producer_gap = 0;
};

unsigned destination(const Case input) {
  return input.producer ? producer_destination : 0U;
}

unsigned store_address(const Case input, const unsigned index) {
  const auto begin = boot + (input.producer ? 2U + input.producer_gap : 0U);
  return begin + (index * (1U + input.gap));
}

unsigned retirement(const Case input, const unsigned index) {
  unsigned initial = first_retirement;
  if (input.producer) {
    const auto extra = input.producer_gap > source_hazard_spacing
                           ? input.producer_gap - source_hazard_spacing
                           : 0U;
    initial = producer_retirement + extra;
  }
  return initial + (index * (instruction_cost + input.gap));
}

DspDebugState prepare(DspMachine &machine, const Case input) {
  DspProgram code;
  if (input.producer) {
    code.word(long_r0);
    code.word(producer_destination);
    for (unsigned index = 0; index < input.producer_gap; ++index) { code.word(0); }
  }
  for (unsigned index = 0; index < input.count; ++index) {
    code.word(input.increment ? increment_program : direct_program);
    if (index + 1 < input.count) {
      for (unsigned gap = 0; gap < input.gap; ++gap) { code.word(0); }
    }
  }
  for (unsigned index = 0; index < padding; ++index) { code.word(0); }
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  machine.bus().shi().observe_spi({.mosi = false});
  machine.bus().shi().write({.address = source_clock, .value = 0});
  machine.bus().shi().write({.address = source_control, .value = fifo_24});
  for (unsigned index = 0; index <= input.count; ++index) {
    spi_prefix::word(machine.bus(), index == input.count ? following : input.payload + index);
    machine.bus().shi().observe_spi({.mosi = false});
  }
  for (unsigned index = 0; index < input.count; ++index) {
    machine.bus().write({.address = destination(input) + index, .value = sentinel});
  }
  return machine.cpu().debug_state();
}

void final_state(DspMachine &machine, const DspDebugState &before, const Case input) {
  auto expected = before;
  expected.cpu.pc = store_address(input, input.count - 1) + 1;
  expected.cpu.r[0] = destination(input) + (input.increment ? input.count : 0U);
  expected.native_instructions += input.count + ((input.count - 1) * input.gap);
  if (input.producer) { expected.native_instructions += 1U + input.producer_gap; }
  const auto actual = machine.cpu().debug_state();
  require(actual.cpu == expected.cpu && actual.sc == expected.sc && actual.ep == expected.ep &&
              actual.sz == expected.sz && actual.stack_high == expected.stack_high &&
              actual.stack_low == expected.stack_low &&
              actual.native_instructions == expected.native_instructions,
          "Independent complete CPU/retained-frame result at actual last MOVEP retirement");
  for (unsigned index = 0; index < input.count; ++index) {
    require(machine.bus().inspect_program(destination(input) + index) == input.payload + index,
            "Only native MOVEP writes each real SPI word to the literal P address");
  }
  require(machine.bus().read({.space = DspSpace::x, .address = hrx}) == following &&
              (machine.bus().shi().read(source_control) & not_empty) == 0,
          "Exactly one known following wire word remains after all native transfers");
}

void successor(DspMachine &machine, DspFetchRun &run, const Case input) {
  const auto before = machine.cpu().debug_state();
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(cycle);
  auto expected = before.cpu;
  ++expected.pc;
  const auto actual = machine.cpu().debug_state();
  require(phase.retirement_published && phase.execute2 &&
              phase.execute2->address == store_address(input, input.count - 1) + 1 &&
              cycle.completed_cycles == retirement(input, input.count - 1) + 1 &&
              actual.cpu == expected && actual.sc == before.sc && actual.ep == before.ep &&
              actual.sz == before.sz && actual.stack_high == before.stack_high &&
              actual.stack_low == before.stack_low &&
              actual.native_instructions == before.native_instructions + 1,
          "The real successor retires one period after the last native P transfer");
}

unsigned observe_stores(const DspFetchCycleResult &cycle,
                        DspMachine &machine,
                        const Case input,
                        std::array<unsigned, 3> &held) {
  const auto &phase = pipeline::stages(cycle);
  unsigned writes = 0;
  if (phase.execute1) {
    for (unsigned index = 0; index < input.count; ++index) {
      if (phase.execute1->address != store_address(input, index)) { continue; }
      if (!phase.memory) {
        ++held.at(index);
        require(!phase.advanced && !phase.execution_captured && !cycle.issued_address &&
                    machine.bus().inspect_program(destination(input) + index) == sentinel,
                "A real five-period execution hold cannot dispatch native I/O or new F1");
        continue;
      }
      require(cycle.completed_cycles == retirement(input, index) - 1 &&
                  phase.memory->data.value == input.payload + index &&
                  phase.memory->additional_data &&
                  phase.memory->additional_data->kind == DspNativeAccessKind::write &&
                  phase.memory->additional_data->target.space == DspSpace::p &&
                  phase.memory->additional_data->target.address == destination(input) + index &&
                  phase.memory->additional_data->value == input.payload + index &&
                  !cycle.bus.fetch && cycle.bus.core_access_count == 2,
              "Actual native peripheral/P effects use the free P bus before the independent E2");
      ++writes;
    }
  }
  return writes;
}

void one_case(const Case input) {
  DspMachine machine;
  const auto before = prepare(machine, input);
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  auto contract = pipeline::contract();
  contract.pipeline = DspFetchPipeline::automatic_memory;
  DspFetchRun run(contract, machine);
  std::array<unsigned, 3> held{};
  unsigned writes = 0;
  unsigned retired = 0;
  const auto last = retirement(input, input.count - 1);
  while (run.completed_cycles() < last) {
    const auto cycle = run.advance_cycle(machine, fetch_clock::input());
    const auto &phase = pipeline::stages(cycle);
    writes += observe_stores(cycle, machine, input, held);
    if (phase.retirement_published && phase.execute2) {
      for (unsigned index = 0; index < input.count; ++index) {
        if (phase.execute2->address != store_address(input, index)) { continue; }
        require(cycle.completed_cycles == retirement(input, index),
                "Initial/composed/R-source retirement clocks match independent NXP windows");
        ++retired;
      }
    }
  }
  require(writes == input.count && retired == input.count &&
              run.elapsed_ticks() == last * pipeline::period,
          "Every actual transfer retires once on the declared nominal clock");
  for (unsigned index = 0; index < input.count; ++index) {
    require(held.at(index) == 5,
            "Each six-clock P transfer has exactly five extra execution periods");
  }
  final_state(machine, before, input);
  successor(machine, run, input);
}

void held_restores() {
  constexpr Case input{};
  constexpr unsigned first_hold = 6;
  constexpr unsigned holds = 5;
  for (unsigned pause = first_hold; pause < first_hold + holds; ++pause) {
    DspMachine machine;
    const auto before = prepare(machine, input);
    static_cast<void>(
        machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
    auto contract = pipeline::contract();
    contract.pipeline = DspFetchPipeline::automatic_memory;
    DspFetchRun run(contract, machine);
    while (run.completed_cycles() < pause) {
      static_cast<void>(run.advance_cycle(machine, fetch_clock::input()));
    }
    const auto saved = run.checkpoint(machine);
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      require(run.completed_cycles() == pause && !run.faulted() &&
                  machine.cpu().state().pc == boot && machine.cpu().state().r[0] == 1 &&
                  machine.bus().inspect_program(0) == sentinel,
              "Trusted held-period restore keeps real AG2 but no future FIFO/P/retirement effect");
      while (run.completed_cycles() < first_retirement) {
        static_cast<void>(run.advance_cycle(machine, fetch_clock::input()));
      }
      final_state(machine, before, input);
      successor(machine, run, input);
    }
  }
}

}
}

int main() {
  try {
    using namespace blofeld::test::program_clock;
    one_case({.increment = false});
    for (unsigned count = 1; count <= 3; ++count) { one_case({.count = count}); }
    one_case({.count = 2, .gap = 1});
    for (unsigned gap = 0; gap < 5; ++gap) { one_case({.producer = true, .producer_gap = gap}); }
    for (const auto value : {0U, 0x800000U, 0xffffffU}) { one_case({.payload = value}); }
    held_restores();
    std::cout << "13 actual automatic P-transfer clock/state/FIFO cases passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
