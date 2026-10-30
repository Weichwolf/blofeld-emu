// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_memory_pipeline_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"
#include "dsp_transfer_pipeline_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned pointer = 7, modifier = 7, payload = 0x812345;
constexpr unsigned mover = 0x200000, source_shift = 13, destination_shift = 8;
constexpr unsigned a1_code = 12, b1_code = 13, n0_code = 24;
constexpr unsigned producer_e1 = 6, producer_e2 = 7, cycles_without_hold = 8;
constexpr unsigned expected_cases = 384;
constexpr std::array holds{3U, 2U, 1U, 0U};

struct Case {
  bool b = false;
  DspSpace space = DspSpace::x;
  unsigned index = 0, gap = 0, offset = 0;

  [[nodiscard]] unsigned cycles() const { return cycles_without_hold + gap + holds.at(gap); }

  [[nodiscard]] unsigned consumer() const { return transfer_pipeline::body + 3 + gap; }
};

struct Oracle {
  DspDebugState state;
  unsigned observed_holds = 0;
};

unsigned opcode(const Case sample) {
  return mover | ((sample.b ? b1_code : a1_code) << source_shift) |
         ((n0_code + sample.index) << destination_shift);
}

Oracle prepare(DspMachine &machine, const Case sample) {
  const auto reg = std::to_string(sample.index);
  const auto accumulator = std::string(sample.b ? "b1" : "a1");
  const auto space = std::string(sample.space == DspSpace::x ? "x" : "y");
  std::vector<std::string> code{"move #>" + number(sample.offset) + ',' + accumulator,
                                "move " + accumulator + ",n" + reg};
  code.insert(code.end(), sample.gap, "nop");
  code.push_back("move " + space + ":(r" + reg + ")+n" + reg + ",y1");
  transfer_pipeline::prepare(machine, code, {.index = sample.index});
  static_cast<void>(reference::finish(machine, {.pending = machine.reference_pending()}));
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  machine.bus().write({.space = sample.space, .address = pointer, .value = payload});
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == transfer_pipeline::body + 2 &&
              before.cpu.r.at(sample.index) == pointer && before.cpu.n.at(sample.index) == 3 &&
              before.cpu.m.at(sample.index) == modifier &&
              machine.bus().inspect_program(before.cpu.pc) == opcode(sample),
          "Native preconditions and own independent A1/B1-to-N encoding match exactly");
  return {.state = before};
}

void step(DspFetchRun &run, DspMachine &machine, Oracle &oracle, const Case sample) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(cycle);
  const auto clock = cycle.completed_cycles;
  auto &expected = oracle.state;
  if (phase.agu_dependencies != 0) {
    ++oracle.observed_holds;
    require(phase.agu_dependencies == (1U << sample.index) && !phase.decode_accepted &&
                phase.advanced && !cycle.issued_address && !cycle.bus.fetch,
            "Only the explicit matching N register holds actual Decode/F2/F1");
  }
  if (clock == producer_e1) {
    const DspMoveFields fields{.n = static_cast<std::uint8_t>(1U << sample.index)};
    require(phase.transfer && phase.transfer->pc == transfer_pipeline::body + 2 &&
                phase.transfer->opcode == opcode(sample) && phase.transfer->fields == fields,
            "Actual E1 publishes exactly the raw selected N lane");
    expected.cpu.n.at(sample.index) = sample.offset;
  }
  const bool first_retirement = clock == producer_e2;
  const bool nop_retirement = clock > producer_e2 && clock <= producer_e2 + sample.gap;
  const bool final_retirement = clock == sample.cycles();
  require(phase.retirement_published == (first_retirement || nop_retirement || final_retirement),
          "Literal reference E2 periods retain exact producer/NOP/load retirements");
  if (first_retirement || nop_retirement || final_retirement) {
    ++expected.cpu.pc;
    ++expected.native_instructions;
    ++expected.native_cycles;
  }
  if (clock + 3 == sample.cycles()) {
    require(phase.ag1_address && phase.ag1_address->word.address == sample.consumer() &&
                phase.ag1_address->update.address == pointer &&
                phase.ag1_address->update.value == ((pointer + sample.offset) & modifier),
            "Actual consumer AG1 uses the ready raw N value and independent modulo8 result");
  }
  if (clock + 2 == sample.cycles()) {
    require(phase.ag2_address && phase.ag2_address->word.address == sample.consumer(),
            "The real consumer owns its exact AG2 reference period");
    expected.cpu.r.at(sample.index) = (pointer + sample.offset) & modifier;
  }
  if (clock + 1 == sample.cycles()) {
    expected.cpu.y = (std::uint64_t{payload} << word_bits) | memory_pipeline::half;
  }
  require(machine.cpu().debug_state() == expected,
          "Complete CPU/CCR/counters/stack match at reference period " + std::to_string(clock));
}

void one_case(const Case sample, const std::optional<unsigned> cut) {
  DspMachine machine;
  auto oracle = prepare(machine, sample);
  DspFetchRun run(memory_pipeline::contract(), machine);
  const auto finish = [&] {
    while (run.completed_cycles() < sample.cycles()) { step(run, machine, oracle, sample); }
    require(oracle.observed_holds == holds.at(sample.gap) &&
                machine.cpu().state().pc == sample.consumer() + 1 &&
                machine.bus().read({.space = sample.space, .address = pointer}) == payload,
            "Exact hold count and actual read-only cell survive the whole own program");
  };
  if (!cut) {
    finish();
    return;
  }
  while (run.completed_cycles() < *cut) { step(run, machine, oracle, sample); }
  const auto saved = run.checkpoint(machine);
  const auto before = oracle;
  for (unsigned suffix = 0; suffix < 2; ++suffix) {
    run.restore(saved, machine);
    oracle = before;
    require(machine.cpu().debug_state() == oracle.state,
            "Joint owner restore preserves native prefix and actual AGU context");
    finish();
  }
}

std::vector<Case> cases() {
  std::vector<Case> result;
  for (const bool b : {false, true}) {
    for (const auto space : {DspSpace::x, DspSpace::y}) {
      for (unsigned index = 0; index < 8; ++index) {
        for (unsigned gap = 0; gap < 4; ++gap) {
          for (const unsigned offset : {0U, 1U, 3U}) {
            result.push_back(
                {.b = b, .space = space, .index = index, .gap = gap, .offset = offset});
          }
        }
      }
    }
  }
  require(result.size() == expected_cases, "All384 independent raw-offset AGU windows exist");
  return result;
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 2, "Select windows or snapshots");
    const std::string_view mode(arguments[1]);
    require(mode == "windows" || mode == "snapshots", "Unknown own offset AGU scope");
    unsigned checked = 0;
    for (const auto sample : cases()) {
      if (mode == "windows") {
        one_case(sample, std::nullopt);
        ++checked;
      } else if (sample.index == 4 && sample.gap == 0 && sample.offset == 3) {
        for (unsigned cut = 0; cut <= sample.cycles(); ++cut) {
          one_case(sample, cut);
          ++checked;
        }
      }
    }
    std::cout << mode << " passed " << checked << " raw-offset AGU cases\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
