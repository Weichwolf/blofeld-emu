// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_modifier_window_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <cstdint>
#include <iostream>
#include <string>

namespace blofeld::test::modifier_window {
namespace {
// FM13-131/12-20: raw M1->M0/M7 or full A/B; low ALU byte is zero.
constexpr unsigned m1_to_m0 = 0x04e1a0;
constexpr unsigned m0_to_m2 = 0x04e0a2;
constexpr unsigned m0_to_a = 0x044ea0;
constexpr unsigned m1_to_a = 0x044ea1;
constexpr unsigned register_shift = 8;
constexpr unsigned clear_a = 0x200013;
// FM13-118/122: full A/B->X/Y:(R4)+ and X/Y:(R0)+->X0.
constexpr unsigned a_store = 0x565c00;
constexpr unsigned x_load = 0x44d800;

unsigned producer(const Case sample) {
  switch (sample.family) {
    case Family::chain: return m1_to_m0;
    case Family::clear: return clear_a | (sample.producer_b ? 8U : 0U);
    case Family::store:
      return m1_to_a | (static_cast<unsigned>(sample.producer_b) << register_shift);
    case Family::agu: return m1_to_m0 | sample.destination;
    default: throw Fault("Invalid own modifier producer family");
  }
}

unsigned consumer(const Case sample) {
  switch (sample.family) {
    case Family::chain:
      return sample.destination == 2 ? m0_to_m2
                                     : m0_to_a | ((sample.destination - 8) << register_shift);
    case Family::clear:
      return m1_to_a | (static_cast<unsigned>(sample.consumer_b) << register_shift);
    case Family::store: return a_store | (sample.consumer_b ? b_bit : 0U) | (sample.y ? y_bit : 0U);
    case Family::agu: return x_load | (sample.y ? y_bit : 0U);
    default: throw Fault("Invalid own modifier consumer family");
  }
}

std::uint64_t &accumulator(DspState &state, const bool b) {
  return b ? state.b : state.a;
}

void memory(DspMachine &machine, const Case sample, const unsigned period) {
  const bool active = sample.family == Family::store && period >= consumer_e1(sample);
  const auto original = sample.consumer_b ? initial_b : initial_a;
  const auto source = sample.producer_b == sample.consumer_b ? expanded(sample.value) : original;
  const auto stored = static_cast<unsigned>((source >> word_bits) & word_mask);
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const unsigned address : {data - 1, data, data + 1}) {
      const bool selected = active && address == data && (space == DspSpace::y) == sample.y;
      require(machine.bus().read({.space = space, .address = address}) ==
                  (selected ? stored : canary),
              "Literal native source store preserves other cells and the other memory space");
    }
    require(machine.bus().read({.space = space, .address = pointer}) == loaded,
            "The genuine source cell is never changed by native loads or other stores");
  }
}

void phases(const DspFetchCycleResult &cycle, const Case sample) {
  const auto &phase = pipeline::stages(cycle);
  const auto period = static_cast<unsigned>(cycle.completed_cycles);
  const bool agu_stall = sample.family == Family::agu && sample.destination == 0 &&
                         period >= 5 + sample.gap && period < 8;
  const bool source_stall = sample.family == Family::store && delay(sample) != 0 && period == 8;
  const bool data = sample.family == Family::store || sample.family == Family::agu;
  const bool producer_move = sample.family != Family::clear && period == 7;
  const bool consumer_move = !data && period == consumer_e1(sample);
  require(phase.agu_dependencies == (agu_stall ? 1U : 0U) &&
              phase.memory_source_stall == source_stall,
          "Independent FM producer windows admit only the declared AGU/source holds");
  require(phase.memory.has_value() == (data && period == consumer_e1(sample)) &&
              phase.transfer.has_value() == (producer_move || consumer_move),
          "The genuine E1 operation and register transfer occur at their independent periods");
  const auto count = retired(sample, period);
  const auto previous = retired(sample, period - 1);
  require(phase.retirement_published == (count != previous),
          "Independent single-word costs/order determine actual E2 retirement, including holes");
  if (phase.retirement_published) {
    require(phase.execute2 && phase.execute2->address == entry + previous,
            "Native retirement preserves the exact anchor/producer/gap/consumer/NOP word order");
  }
}
}

DspFetchRunInputs contract() {
  auto result = fetch_clock::contract();
  result.execution = DspReferenceExecution::staged_register_boundaries;
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

DspDebugState prepare(DspMachine &machine, const Case sample) {
  DspProgram program;
  program.immediate("omr", ordinary_omr);
  for (unsigned index = 0; index < 8; ++index) {
    auto value = index == 0 ? sample.initial_modifier : linear_modifier;
    if (index == 1) { value = sample.value; }
    program.immediate("m" + std::to_string(index), value);
  }
  program.immediate("r0", pointer);
  program.immediate("r4", data);
  for (const bool b : {false, true}) {
    const auto value = b ? initial_b : initial_a;
    const std::string name = b ? "b" : "a";
    for (unsigned part = 0; part < 3; ++part) {
      constexpr unsigned extension_mask = 0xff;
      program.immediate(name + std::to_string(part),
                        (value >> (part * word_bits)) & (part == 2 ? extension_mask : word_mask));
    }
  }
  program.immediate("sr", masked_status);
  for (unsigned index = 0; index < 5; ++index) { program.word(0); }
  program.word(absolute_jump);
  program.word(entry);
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::until(machine, entry);
  for (unsigned address = entry; address < entry + 32; ++address) {
    machine.bus().write({.address = address});
  }
  machine.bus().write({.address = entry + 1, .value = producer(sample)});
  machine.bus().write({.address = entry + consumer_index(sample), .value = consumer(sample)});
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const unsigned address : {data - 1, data, data + 1, pointer}) {
      machine.bus().write(
          {.space = space, .address = address, .value = address == pointer ? loaded : canary});
    }
  }
  const auto before = machine.cpu().debug_state();
  require(
      before.cpu.pc == entry && before.cpu.a == initial_a && before.cpu.b == initial_b &&
          before.cpu.m[0] == sample.initial_modifier && before.cpu.m[1] == sample.value &&
          before.cpu.r[0] == pointer && before.cpu.r[4] == data && before.cpu.sr == masked_status,
      "Literal native setup establishes independent raw M, accumulator, address and flag seeds");
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return before;
}

DspDebugState expected(const DspDebugState &before, const Case sample, const unsigned period) {
  auto wanted = before;
  if (period >= 7) {
    if (sample.family == Family::chain || sample.family == Family::agu) {
      wanted.cpu.m.at(sample.family == Family::chain ? 0U : sample.destination) = sample.value;
    } else if (sample.family == Family::store) {
      accumulator(wanted.cpu, sample.producer_b) = expanded(sample.value);
    }
  }
  if (sample.family == Family::clear && period >= 8) {
    accumulator(wanted.cpu, sample.producer_b) = 0;
    wanted.cpu.sr = masked_status | clear_flags;
  }
  if (period >= consumer_e1(sample)) {
    if (sample.family == Family::chain && sample.destination == 2) {
      wanted.cpu.m[2] = sample.value;
    } else if (sample.family == Family::chain || sample.family == Family::clear) {
      accumulator(wanted.cpu,
                  sample.family == Family::clear ? sample.consumer_b : sample.destination == 9) =
          expanded(sample.value);
    } else if (sample.family == Family::agu) {
      wanted.cpu.x = (wanted.cpu.x & ~std::uint64_t{word_mask}) | loaded;
    }
  }
  if (period >= consumer_ag2(sample)) {
    if (sample.family == Family::store) { wanted.cpu.r[4] = data + 1; }
    if (sample.family == Family::agu) { wanted.cpu.r[0] = data; }
  }
  const auto count = retired(sample, period);
  wanted.cpu.pc += count;
  wanted.native_instructions += count;
  wanted.native_cycles += count;
  return wanted;
}

void step(DspFetchRun &run, DspMachine &machine, const DspDebugState &before, const Case sample) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto period = static_cast<unsigned>(cycle.completed_cycles);
  phases(cycle, sample);
  memory(machine, sample, period);
  const auto wanted = expected(before, sample, period);
  const auto actual = machine.cpu().debug_state();
  if (actual != wanted) {
    std::cerr << "family=" << static_cast<unsigned>(sample.family) << " period=" << period
              << " gap=" << sample.gap << " value=" << hex(sample.value, 6)
              << " a=" << hex(actual.cpu.a, 14) << '/' << hex(wanted.cpu.a, 14)
              << " b=" << hex(actual.cpu.b, 14) << '/' << hex(wanted.cpu.b, 14)
              << " pc=" << hex(actual.cpu.pc, 6) << '/' << hex(wanted.cpu.pc, 6)
              << " sr=" << hex(actual.cpu.sr, 6) << '/' << hex(wanted.cpu.sr, 6) << '\n';
  }
  require(actual == wanted,
          "Every period matches independent complete native CPU/flag/cache-visible effects");
}

void one(const Case sample) {
  DspMachine machine;
  const auto before = prepare(machine, sample);
  DspFetchRun run(contract(), machine);
  while (run.completed_cycles() < last(sample)) { step(run, machine, before, sample); }
  require(run.elapsed_ticks() == last(sample) * pipeline::period,
          "Reference elapsed time includes only independently required holes");
}

void restores(const Case sample) {
  for (unsigned cut = 6; cut <= 11; ++cut) {
    DspMachine machine;
    const auto before = prepare(machine, sample);
    DspFetchRun run(contract(), machine);
    while (run.completed_cycles() < cut) { step(run, machine, before, sample); }
    const auto saved = run.checkpoint(machine);
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      while (run.completed_cycles() < last(sample)) { step(run, machine, before, sample); }
    }
  }
}
}
