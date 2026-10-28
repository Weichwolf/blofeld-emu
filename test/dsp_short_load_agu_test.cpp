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
constexpr unsigned period_ticks = 45, consumer_base = 0x44c800, group_shift = 8;
constexpr unsigned loaded_pointer_base = 0x507, loaded_pointer_stride = 0x10;
constexpr unsigned variant_stride = 0x100;
constexpr unsigned consumer_value = values[4];

struct Window {
  unsigned kind = 0, group = 0, gap = 0, ccr = 0, variant = 0;
  bool same = true, input_y = false, output_y = false;
};

unsigned group(const Window sample) {
  return sample.same ? sample.group : (sample.group + 1) % 8;
}

unsigned delay(const Window sample) {
  // FM3.5.1: a MOVE-written AGU operand needs three intervening periods.
  return sample.same && sample.gap < 3 ? 3 - sample.gap : 0U;
}

unsigned duration(const Window sample) {
  return 8 + sample.gap + delay(sample);
}

unsigned pointer_value(const Window sample) {
  return loaded_pointer_base + (sample.group * loaded_pointer_stride) +
         (sample.variant * variant_stride);
}

unsigned loaded(const Window sample) {
  if (sample.kind == 0) { return pointer_value(sample); }
  if (sample.kind == 1) { return 1 + sample.variant; }
  constexpr std::array modifiers{0x7f0003U, 0x800007U};
  return modifiers.at(sample.variant);
}

Case operand(const Window sample) {
  return {.destination = 8 + (sample.kind * 8) + sample.group,
          .address = addresses.at(sample.group % addresses.size()),
          .value = loaded(sample),
          .ccr = sample.ccr,
          .y = sample.input_y,
          .small_offsets = true};
}

unsigned pointer(const Window sample) {
  return sample.kind == 0 && sample.same ? loaded(sample) : r_seed(group(sample));
}

unsigned updated(const Window sample) {
  const auto initial = pointer(sample);
  const auto offset =
      sample.kind == 1 && sample.same ? loaded(sample) : n_seed(group(sample), true);
  const auto modifier = sample.kind == 2 && sample.same ? loaded(sample) : m_seed(group(sample));
  constexpr unsigned low_word_mask = 0xffff;
  const auto mask = modifier & low_word_mask;
  require(mask == 3 || mask == 7,
          "Only defined aligned four/eight-word modulo buffers enter this independent oracle");
  return (initial & ~mask) | ((initial + offset) & mask);
}

std::vector<std::uint32_t> stream(const Window sample) {
  std::vector<std::uint32_t> result{opcode(operand(sample))};
  result.insert(result.end(), sample.gap, 0);
  // FM13-118/122: X0 read,EA=1,MMM=001=(Rn)+Nn;S selects X/Y.
  result.push_back(consumer_base | (group(sample) << group_shift) |
                   (sample.output_y ? ordinary_y : 0U));
  result.insert(result.end(), 8, 0);
  return result;
}

std::array<unsigned, 6> extra_cells(const Window sample) {
  const auto old = r_seed(group(sample));
  const auto next = pointer_value(sample);
  return {old - 1, old, old + 1, next - 1, next, next + 1};
}

void extra_memory(DspMachine &machine, const Window sample) {
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto address : extra_cells(sample)) {
      const bool source = address == pointer(sample) && (space == DspSpace::y) == sample.output_y;
      require(
          machine.bus().read({.space = space, .address = address}) ==
              (source ? consumer_value : canary),
          "The downstream AGU read preserves every old/new pointer neighbor and opposite space");
    }
  }
}

DspDebugState prepare_window(DspMachine &machine, const Window sample) {
  const auto before = prepare(machine, operand(sample));
  checkpoint::ram(machine.bus(), entry, stream(sample));
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto address : extra_cells(sample)) {
      const bool selected = address == pointer(sample) && (space == DspSpace::y) == sample.output_y;
      machine.bus().write(
          {.space = space, .address = address, .value = selected ? consumer_value : canary});
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
  if (count >= 6 + sample.gap + delay(sample)) { result.cpu.r.at(group(sample)) = updated(sample); }
  if (count >= 7 + sample.gap + delay(sample)) {
    result.cpu.x = (result.cpu.x & ~static_cast<std::uint64_t>(word_mask)) | consumer_value;
  }
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

void consumer_address(const DspAddressStageView &view, const Window sample, const bool published) {
  const auto pc = entry + 1 + sample.gap;
  const auto word =
      consumer_base | (group(sample) << group_shift) | (sample.output_y ? ordinary_y : 0U);
  require(
      view.kind == DspAddressStageKind::register_agu && view.pointer_update &&
          view.published == published && view.word.address == pc && view.word.value == word &&
          !view.secondary && !view.extension && view.update.index == group(sample) &&
          view.update.address == pointer(sample) && view.update.value == updated(sample) &&
          view.update.space == (sample.output_y ? DspSpace::y : DspSpace::x) &&
          view.update.operation == DspMemoryOperation::read && view.update.pc == pc &&
          view.update.omr == ordinary_omr && view.data_access_count == 1 &&
          view.data_accesses[0].target.address == pointer(sample) &&
          view.data_accesses[0].target.space == view.update.space &&
          view.data_accesses[0].operation == DspMemoryOperation::read,
      "Separate actual AG1/AG2 receipts retain both the captured source and new modulo pointer");
}

void address_phases(const DspPipelineCycle &phase, const Window sample, const unsigned count) {
  if (count == 5 + sample.gap + delay(sample)) {
    if (!phase.ag1_address) { throw Fault("Downstream AGU lost its captured AG1 address"); }
    consumer_address(*phase.ag1_address, sample, false);
  }
  if (count == 6 + sample.gap + delay(sample)) {
    if (!phase.ag2_address) { throw Fault("Downstream AGU lost its published AG2 address"); }
    consumer_address(*phase.ag2_address, sample, true);
  }
}

void producer_effect(const DspMemoryStageResult &effect, const Window sample) {
  address(effect.address, operand(sample), true);
  const auto &data = effect.data;
  require(data.kind == DspNativeAccessKind::read && data.pc == entry &&
              data.target.address == operand(sample).address &&
              data.target.space == (sample.input_y ? DspSpace::y : DspSpace::x) &&
              data.value == loaded(sample) && !effect.additional_data,
          "Actual AA6 E1 publishes the exact raw R/N/M source word once");
}

void consumer_effect(const DspMemoryStageResult &effect, const Window sample) {
  consumer_address(effect.address, sample, true);
  const auto offset =
      sample.kind == 1 && sample.same ? loaded(sample) : n_seed(group(sample), true);
  const auto modifier = sample.kind == 2 && sample.same ? loaded(sample) : m_seed(group(sample));
  require(effect.data.kind == DspNativeAccessKind::read &&
              effect.data.pc == entry + 1 + sample.gap &&
              effect.data.target.address == pointer(sample) &&
              effect.data.target.space == (sample.output_y ? DspSpace::y : DspSpace::x) &&
              effect.data.value == consumer_value && !effect.additional_data &&
              effect.inputs.r.at(group(sample)) == updated(sample) &&
              effect.inputs.n.at(group(sample)) == offset &&
              effect.inputs.m.at(group(sample)) == modifier,
          "Visible E1 inputs retain published AG2 and latest N/raw-M;the native read uses captured "
          "AG1");
}

void effects(const DspPipelineCycle &phase, const Window sample, const unsigned count) {
  const bool agu_stall = sample.same && count >= 4 + sample.gap && count < 7;
  const auto consumer_e1 = 7 + sample.gap + delay(sample);
  address_phases(phase, sample, count);
  require(phase.agu_dependencies == (agu_stall ? 1U << group(sample) : 0U) &&
              !phase.memory_source_stall &&
              phase.memory.has_value() == (count == 6 || count == consumer_e1),
          "Only the matching AGU field introduces its independent three-minus-gap hold");
  if (count == 6) {
    if (!phase.memory) { throw Fault("AA6 AGU producer lost its native read"); }
    producer_effect(*phase.memory, sample);
  }
  if (count == consumer_e1) {
    if (!phase.memory) { throw Fault("Downstream AGU consumer lost its native read"); }
    consumer_effect(*phase.memory, sample);
  }
}

void clock(DspFetchRun &run,
           DspMachine &machine,
           const DspDebugState &before,
           const Window sample,
           const unsigned count) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  require(cycle.completed_cycles == count && run.completed_cycles() == count &&
              run.elapsed_ticks() == static_cast<std::uint64_t>(count) * period_ticks &&
              run.timer_clk2_edges() == count / 2 && machine.bus().last_dma_cycle() == count - 1 &&
              !cycle.bus.dma.core_stall,
          "Every dependent/independent window preserves declared nominal/CLK2/DMA periods");
  const auto target = expected(before, sample, count);
  const auto actual = machine.cpu().debug_state();
  if (actual != target) {
    std::cerr << "kind=" << sample.kind << " group=" << sample.group << " same=" << sample.same
              << " gap=" << sample.gap << " variant=" << sample.variant << " period=" << count
              << " R=" << hex(actual.cpu.r.at(group(sample)), 6) << '/'
              << hex(target.cpu.r.at(group(sample)), 6) << " X=" << hex(actual.cpu.x, 12) << '/'
              << hex(target.cpu.x, 12) << " pc=" << hex(actual.cpu.pc, 6) << '/'
              << hex(target.cpu.pc, 6) << '\n';
  }
  require(
      actual == target,
      "All CPU/CCR/raw register fields match independent AA6/AG2/E1/E2 expectations each period");
  if (!cycle.pipeline) {
    throw Fault("AA6/downstream-AGU window lost its actual pipeline receipt");
  }
  effects(*cycle.pipeline, sample, count);
  memory(machine, operand(sample));
  extra_memory(machine, sample);
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
        auto current = sample;
        current.input_y = input_y;
        current.output_y = output_y;
        current.ccr = ccr;
        one(current);
        ++count;
      }
    }
  }
  return count;
}

unsigned matrix() {
  unsigned count = 0;
  for (unsigned kind = 0; kind < 3; ++kind) {
    for (unsigned group = 0; group < 8; ++group) {
      for (unsigned variant = 0; variant < 2; ++variant) {
        for (const bool same : {false, true}) {
          for (unsigned gap = 0; gap < 4; ++gap) {
            count += variants(
                {.kind = kind, .group = group, .gap = gap, .variant = variant, .same = same});
          }
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
    for (unsigned kind = 0; kind < 3; ++kind) {
      for (const auto group : {0U, 7U}) {
        for (const bool same : {false, true}) {
          for (const auto gap : {0U, 3U}) {
            cuts += snapshots({.kind = kind,
                               .group = group,
                               .gap = gap,
                               .ccr = flags.back(),
                               .same = same,
                               .input_y = true});
          }
        }
      }
    }
    constexpr unsigned expected_cases = 3072;
    constexpr unsigned expected_cuts = 246;
    require(count == expected_cases && cuts == expected_cuts,
            "All24 AA6 AGU destinations,independent controls and selected snapshot cuts execute");
    std::cout
        << "3072 AA6-to-AGU windows,246cuts/492suffixes,independent full CPU/CCR/raw-M/modulo/"
           "stalls/E1/E2/nominal/CLK2/DMA and RAM guards passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
