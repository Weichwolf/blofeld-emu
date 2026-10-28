// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
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

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test;
using namespace blofeld::test::short_load;
constexpr unsigned period_ticks = 45, b_bit = 0x10000, extension_sign = 0x80;
constexpr unsigned extension_fill = 0xffff00;
// FM13-118/122: standalone A0/A1/A2 stores to (R0)+;B and X/Y select independent lanes.
constexpr std::array stores{0x505800U, 0x545800U, 0x525800U};

struct Window {
  unsigned destination = 0, part = 0, gap = 0, ccr = 0, value = values[4];
  bool same = true, input_y = false, output_y = false;
};

Case operand(const Window sample) {
  return {.destination = sample.destination,
          .address = addresses[2],
          .value = sample.value,
          .ccr = sample.ccr,
          .y = sample.input_y};
}

bool producer_b(const Window sample) {
  return sample.destination == 7 || (sample.destination >= 3 && sample.destination < 6);
}

bool consumer_b(const Window sample) {
  return sample.same ? producer_b(sample) : !producer_b(sample);
}

unsigned delay(const Window sample) {
  // FM3.5.2.1/Figure3-13: includes a load of A1 followed by a store of A2.
  return sample.same && sample.gap == 0 ? 1U : 0U;
}

unsigned duration(const Window sample) {
  return 8 + sample.gap + delay(sample);
}

unsigned consumer_word(const Window sample) {
  return stores.at(sample.part) | (consumer_b(sample) ? b_bit : 0U) |
         (sample.output_y ? ordinary_y : 0U);
}

std::vector<std::uint32_t> stream(const Window sample) {
  std::vector<std::uint32_t> code{opcode(operand(sample))};
  code.insert(code.end(), sample.gap, 0);
  code.push_back(consumer_word(sample));
  code.insert(code.end(), 8, 0);
  return code;
}

unsigned stored(const DspDebugState &before, const Window sample) {
  auto inputs = before.cpu;
  destination(inputs, operand(sample));
  const auto source = consumer_b(sample) ? inputs.b : inputs.a;
  const auto mask = sample.part == 2 ? extension_mask : word_mask;
  auto value = static_cast<unsigned>((source >> (sample.part * word_bits)) & mask);
  if (sample.part == 2 && (value & extension_sign) != 0) { value |= extension_fill; }
  return value;
}

void extra_memory(DspMachine &machine,
                  const DspDebugState &before,
                  const Window sample,
                  const unsigned count) {
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto address : {r_seed_base - 1, r_seed_base, r_seed_base + 1}) {
      const bool selected = address == r_seed_base && (space == DspSpace::y) == sample.output_y &&
                            count >= 7 + sample.gap + delay(sample);
      require(machine.bus().read({.space = space, .address = address}) ==
                  (selected ? stored(before, sample) : canary),
              "Only actual consumer E1 changes the selected raw/sign-extended partial store cell");
    }
  }
}

DspDebugState prepare_window(DspMachine &machine, const Window sample) {
  const auto before = prepare(machine, operand(sample));
  checkpoint::ram(machine.bus(), entry, stream(sample));
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto address : {r_seed_base - 1, r_seed_base, r_seed_base + 1}) {
      machine.bus().write({.space = space, .address = address, .value = canary});
    }
  }
  require(!machine.bus().last_dma_cycle(), "Unclocked native prefix establishes DMA epoch zero");
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return before;
}

DspDebugState expected(const DspDebugState &before, const Window sample, const unsigned count) {
  auto result = before;
  if (count >= 6) { destination(result.cpu, operand(sample)); }
  if (count >= 6 + sample.gap) { result.cpu.r[0] = r_seed_base + 1; }
  if (count >= 7) {
    ++result.cpu.pc;
    ++result.native_instructions;
    ++result.native_cycles;
  }
  for (unsigned index = 0; index < sample.gap; ++index) {
    if (count >= 8 + index) {
      ++result.cpu.pc;
      ++result.native_instructions;
      ++result.native_cycles;
    }
  }
  if (count == duration(sample)) {
    ++result.cpu.pc;
    ++result.native_instructions;
    ++result.native_cycles;
  }
  return result;
}

void effects(const DspPipelineCycle &phase,
             const DspDebugState &before,
             const Window sample,
             const unsigned count) {
  const bool held = delay(sample) != 0 && count == 7;
  const auto consumer_e1 = 7 + sample.gap + delay(sample);
  require(phase.agu_dependencies == 0 && phase.memory_source_stall == held &&
              phase.memory.has_value() == (count == 6 || count == consumer_e1),
          "Only an adjacent same-accumulator source has the independent one-period E1 hold");
  if (!phase.memory) { return; }
  const auto &effect = *phase.memory;
  require(!effect.additional_data, "Each actual E1 owns one selected native memory effect");
  const auto &data = effect.data;
  if (count == 6) {
    address(effect.address, operand(sample), true);
    require(data.kind == DspNativeAccessKind::read && data.pc == entry &&
                data.target.address == operand(sample).address &&
                data.target.space == (sample.input_y ? DspSpace::y : DspSpace::x) &&
                data.value == sample.value,
            "Literal producer E1 reads the independently supplied accumulator part/full word");
    return;
  }
  auto loaded = before.cpu;
  destination(loaded, operand(sample));
  require(
      count == consumer_e1 && data.kind == DspNativeAccessKind::write &&
          data.pc == entry + 1 + sample.gap && data.target.address == r_seed_base &&
          data.target.space == (sample.output_y ? DspSpace::y : DspSpace::x) &&
          data.value == stored(before, sample) && effect.inputs.a == loaded.a &&
          effect.inputs.b == loaded.b && effect.inputs.r[0] == r_seed_base + 1,
      "Actual consumer E1 stores the latest independent partial/sign-extended accumulator value");
}

void clock(DspFetchRun &run,
           DspMachine &machine,
           const DspDebugState &before,
           const Window sample,
           const unsigned count) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  require(
      cycle.completed_cycles == count && run.completed_cycles() == count &&
          run.elapsed_ticks() == static_cast<std::uint64_t>(count) * period_ticks &&
          run.timer_clk2_edges() == count / 2 && machine.bus().last_dma_cycle() == count - 1 &&
          !cycle.bus.dma.core_stall,
      "Every accumulator consumer retains exact nominal/CLK2/DMA periods including source holds");
  const auto target = expected(before, sample, count);
  const auto actual = machine.cpu().debug_state();
  if (actual != target) {
    std::cerr << "source=" << name(sample.destination) << " same=" << sample.same
              << " part=" << sample.part << " gap=" << sample.gap << " period=" << count
              << " A=" << hex(actual.cpu.a, 14) << '/' << hex(target.cpu.a, 14)
              << " B=" << hex(actual.cpu.b, 14) << '/' << hex(target.cpu.b, 14)
              << " R0=" << hex(actual.cpu.r[0], 6) << '/' << hex(target.cpu.r[0], 6) << '\n';
  }
  require(
      actual == target,
      "Every reference period preserves complete independent CPU/CCR/AG2/native counter effects");
  if (!cycle.pipeline) { throw Fault("AA6/accumulator consumer lost its actual pipeline receipt"); }
  effects(*cycle.pipeline, before, sample, count);
  memory(machine, operand(sample));
  extra_memory(machine, before, sample, count);
}

void one(const Window sample) {
  DspMachine machine;
  const auto before = prepare_window(machine, sample);
  DspFetchRun run(contract(), machine);
  for (unsigned count = 1; count <= duration(sample); ++count) {
    clock(run, machine, before, sample, count);
  }
}

unsigned snapshots(const Window sample) {
  for (unsigned cut = 0; cut < duration(sample); ++cut) {
    DspMachine machine;
    const auto before = prepare_window(machine, sample);
    DspFetchRun run(contract(), machine);
    for (unsigned count = 1; count <= cut; ++count) { clock(run, machine, before, sample, count); }
    const auto saved = run.checkpoint(machine);
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      for (unsigned count = cut + 1; count <= duration(sample); ++count) {
        clock(run, machine, before, sample, count);
      }
    }
  }
  return duration(sample);
}

unsigned variants(const Window sample) {
  unsigned count = 0;
  for (const bool input_y : {false, true}) {
    for (const bool output_y : {false, true}) {
      for (const auto ccr : {flags.front(), flags.back()}) {
        for (const auto value : {values[4], values[5]}) {
          auto current = sample;
          current.input_y = input_y;
          current.output_y = output_y;
          current.ccr = ccr;
          current.value = value;
          one(current);
          ++count;
        }
      }
    }
  }
  return count;
}

unsigned matrix() {
  unsigned count = 0;
  for (unsigned source = 0; source < 8; ++source) {
    for (unsigned part = 0; part < 3; ++part) {
      for (const bool same : {false, true}) {
        for (unsigned gap = 0; gap < 4; ++gap) {
          count += variants({.destination = source, .part = part, .gap = gap, .same = same});
        }
      }
    }
  }
  return count;
}
}

int main() {
  try {
    const auto count = matrix();
    unsigned cuts = 0;
    for (unsigned source = 0; source < 8; ++source) {
      for (const bool same : {false, true}) {
        for (const auto gap : {0U, 1U}) {
          cuts += snapshots({.destination = source,
                             .part = 2,
                             .gap = gap,
                             .ccr = flags.back(),
                             .value = values.back(),
                             .same = same,
                             .input_y = true});
        }
      }
    }
    constexpr unsigned expected_cases = 3072;
    constexpr unsigned expected_cuts = 280;
    require(count == expected_cases && cuts == expected_cuts,
            "All8 AA6 accumulator targets feed all6 partial store fields and independent controls");
    std::cout << "3072 AA6-accumulator-to-store windows,280cuts/560suffixes,independent raw/sign/"
                 "full CPU/CCR/E1-source/AG2/nominal/CLK2/DMA and136 RAM cells passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
