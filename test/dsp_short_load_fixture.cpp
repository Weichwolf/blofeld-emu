// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_short_load_fixture.hpp"

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

#include <array>
#include <cstdint>
#include <iostream>
#include <string>

namespace blofeld::test::short_load {
unsigned r_seed(const unsigned index) {
  return r_seed_base + (index * r_seed_stride);
}

unsigned n_seed(const unsigned index, const bool small) {
  if (small) { return 2U + (index % 2); }
  return n_seed_base + (index * 2);
}

unsigned m_seed(const unsigned index) {
  return m_seed_base ^ (index << register_shift);
}

std::string name(const unsigned destination) {
  if (destination >= data_begin) { return data_names.at(destination - data_begin); }
  if (destination < 8) { return accumulator_names.at(destination); }
  if (destination < 16) { return "r" + std::to_string(destination - 8); }
  if (destination < m_begin) { return "n" + std::to_string(destination - 16); }
  return "m" + std::to_string(destination - m_begin);
}

unsigned opcode(const Case sample) {
  unsigned base = 0;
  if (sample.destination >= data_begin) {
    constexpr std::array data_words{0x448000U, 0x458000U, 0x468000U, 0x478000U};
    return data_words.at(sample.destination - data_begin) | (sample.address << address_shift) |
           (sample.y ? ordinary_y : 0U);
  }
  if (sample.destination < 8) {
    base = accumulator_words.at(sample.destination);
  } else if (sample.destination < 16) {
    base = r_base | ((sample.destination - 8) << register_shift);
  } else if (sample.destination < m_begin) {
    base = n_base | ((sample.destination - 16) << register_shift);
  } else {
    base = m_base | (sample.destination - m_begin);
  }
  const auto y = sample.destination < m_begin ? ordinary_y : control_y;
  return base | (sample.address << address_shift) | (sample.y ? y : 0U);
}

namespace {
void initialize_cells(DspMachine &machine, const Case sample, const bool known_source) {
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (unsigned address = 0; address <= last_cell; ++address) {
      const bool selected = address == sample.address && (space == DspSpace::y) == sample.y;
      if (selected && !known_source) { continue; }
      machine.bus().write(
          {.space = space, .address = address, .value = selected ? sample.value : canary});
    }
  }
}

void seeds(const DspDebugState &state, const Case sample) {
  require(
      state.cpu.pc == entry && state.cpu.omr == ordinary_omr &&
          state.cpu.sr == (masked_status | sample.ccr) && state.cpu.a == a_seed &&
          state.cpu.b == b_seed &&
          state.cpu.x ==
              ((static_cast<std::uint64_t>(data_seeds[1]) << word_bits) | data_seeds[0]) &&
          state.cpu.y == ((static_cast<std::uint64_t>(data_seeds[3]) << word_bits) | data_seeds[2]),
      "Real own native instructions establish both full accumulators,data pairs and exact modes");
  for (unsigned index = 0; index < 8; ++index) {
    require(state.cpu.r[index] == r_seed(index) &&
                state.cpu.n[index] == n_seed(index, sample.small_offsets) &&
                state.cpu.m[index] == m_seed(index),
            "All24 AGU registers have independent nontrivial native seeds");
  }
}

}

DspDebugState prepare(DspMachine &machine, const Case sample, const bool known_source) {
  DspProgram program;
  program.immediate("omr", ordinary_omr);
  for (unsigned index = 0; index < data_names.size(); ++index) {
    program.immediate(data_names[index], data_seeds[index]);
  }
  for (unsigned index = 0; index < 8; ++index) {
    program.immediate("r" + std::to_string(index), r_seed(index));
    program.immediate("n" + std::to_string(index), n_seed(index, sample.small_offsets));
    program.immediate("m" + std::to_string(index), m_seed(index));
  }
  for (const bool b : {false, true}) {
    const auto value = b ? b_seed : a_seed;
    for (unsigned part = 0; part < 3; ++part) {
      program.immediate(std::string(b ? "b" : "a") + std::to_string(part),
                        (value >> (part * word_bits)) & (part == 2 ? extension_mask : word_mask));
    }
  }
  program.immediate("sr", masked_status | sample.ccr);
  for (unsigned index = 0; index < 5; ++index) { program.word(0); }
  program.word(absolute_jump);
  program.word(entry);
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::until(machine, entry);
  machine.bus().write({.address = entry, .value = opcode(sample)});
  initialize_cells(machine, sample, known_source);
  const auto before = machine.cpu().debug_state();
  seeds(before, sample);
  return before;
}

void destination(DspState &state, const Case sample) {
  if (sample.destination >= data_begin) {
    const auto index = sample.destination - data_begin;
    auto &target = index < 2 ? state.x : state.y;
    const auto shift = (index % 2) * word_bits;
    const auto mask = static_cast<std::uint64_t>(word_mask) << shift;
    target = (target & ~mask) | (static_cast<std::uint64_t>(sample.value) << shift);
    return;
  }

  if (sample.destination >= m_begin) {
    state.m.at(sample.destination - m_begin) = sample.value;
    return;
  }
  if (sample.destination >= 16) {
    state.n.at(sample.destination - 16) = sample.value;
    return;
  }
  if (sample.destination >= 8) {
    state.r.at(sample.destination - 8) = sample.value;
    return;
  }
  const auto index = sample.destination;
  const bool b = index == 7 || (index >= 3 && index < 6);
  auto &target = b ? state.b : state.a;
  if (index >= 6) {
    const auto extension = (sample.value & sign_mask) != 0
                               ? static_cast<std::uint64_t>(extension_mask) << data_bits
                               : 0;
    target = extension | (static_cast<std::uint64_t>(sample.value) << word_bits);
  } else {
    const auto part = index % 3;
    const auto mask = part == 2 ? extension_mask : word_mask;
    const auto shift = part * word_bits;
    const auto shifted = static_cast<std::uint64_t>(mask) << shift;
    target = (target & ~shifted) | (static_cast<std::uint64_t>(sample.value & mask) << shift);
  }
}

void memory(DspMachine &machine, const Case sample, const bool known_source) {
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (unsigned address = 0; address <= last_cell; ++address) {
      const bool selected = address == sample.address && (space == DspSpace::y) == sample.y;
      if (selected && !known_source) { continue; }
      require(machine.bus().read({.space = space, .address = address}) ==
                  (selected ? sample.value : canary),
              "The actual native AA6 load preserves all130 source/neighbor/opposite-space cells");
    }
  }
}

DspFetchRunInputs contract() {
  auto result = fetch_clock::contract();
  result.execution = DspReferenceExecution::staged_register_boundaries;
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

void address(const DspAddressStageView &view, const Case sample, const bool published) {
  require(view.kind == DspAddressStageKind::absolute_short && view.published == published &&
              !view.pointer_update && !view.secondary && !view.extension &&
              view.word.address == entry && view.word.value == opcode(sample) &&
              view.update.address == sample.address &&
              view.update.space == (sample.y ? DspSpace::y : DspSpace::x) &&
              view.update.operation == DspMemoryOperation::read && view.update.pc == entry &&
              view.update.omr == ordinary_omr && view.data_access_count == 1,
          "All36 literal-address loads have one actual AG1 owner,no fake pointer or F2 extension");
}

void period(DspFetchRun &run,
            DspMachine &machine,
            const DspDebugState &before,
            const Case sample,
            const unsigned count,
            const bool known_source) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  constexpr unsigned period_ticks = 45;
  require(cycle.completed_cycles == count && run.completed_cycles() == count &&
              run.elapsed_ticks() == static_cast<std::uint64_t>(count) * period_ticks &&
              run.timer_clk2_edges() == count / 2 && machine.bus().last_dma_cycle() == count - 1 &&
              !cycle.bus.dma.core_stall,
          "Every new destination retains independent nominal/CLK2/DMA timing");
  auto expected = before;
  if (count >= 6) { destination(expected.cpu, sample); }
  if (count == 7) {
    ++expected.cpu.pc;
    ++expected.native_instructions;
    ++expected.native_cycles;
  }
  const auto actual = machine.cpu().debug_state();
  if (actual != expected) {
    std::cerr << name(sample.destination) << " word=" << hex(opcode(sample), 6)
              << " period=" << count << " value=" << hex(sample.value, 6)
              << " SR=" << hex(actual.cpu.sr, 6) << '/' << hex(expected.cpu.sr, 6)
              << " A=" << hex(actual.cpu.a, 14) << '/' << hex(expected.cpu.a, 14)
              << " B=" << hex(actual.cpu.b, 14) << '/' << hex(expected.cpu.b, 14) << '\n';
  }
  require(actual == expected,
          "Independent full CPU/CCR/part/AGU expectation applies at every E1/E2 boundary");
  if (!cycle.pipeline) { throw Fault("New AA6 load destination lost its actual pipeline receipt"); }
  const auto &phase = *cycle.pipeline;
  require(phase.agu_dependencies == 0 && !phase.memory_source_stall &&
              phase.memory.has_value() == (count == 6),
          "A direct load has no phantom address dependency and exactly one E1 read");
  if (count == 4) {
    if (!phase.ag1_address) { throw Fault("New AA6 AG1 lost its literal address"); }
    address(*phase.ag1_address, sample, false);
  }
  if (count == 5) {
    if (!phase.ag2_address) { throw Fault("New AA6 AG2 lost its literal address"); }
    address(*phase.ag2_address, sample, true);
  }
  if (count == 6) {
    if (!phase.memory) { throw Fault("New AA6 E1 lost its actual data effect"); }
    const auto &effect = *phase.memory;
    address(effect.address, sample, true);
    require(effect.inputs == before.cpu && effect.data.kind == DspNativeAccessKind::read &&
                effect.data.pc == entry && effect.data.omr == ordinary_omr &&
                effect.data.target.address == sample.address &&
                effect.data.target.space == (sample.y ? DspSpace::y : DspSpace::x) &&
                effect.data.value == sample.value && effect.move.has_value() &&
                !effect.additional_data,
            "Genuine native E1 consumes the original input state and exact source word before "
            "field writeback");
  }
  memory(machine, sample, known_source);
}

void begin(DspMachine &machine) {
  for (unsigned index = 1; index < 16; ++index) { machine.bus().write({.address = entry + index}); }
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
}

}
