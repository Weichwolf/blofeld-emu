// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"
#include "dsp_scalar_store_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using test::require;
constexpr unsigned staging = 0x300, low_entry = 0x800, wide_entry = 0x2400;
constexpr unsigned reset_program_end = 0x1000, high_ram = 0xc000;
constexpr unsigned normal_mode = 0x300, mode_mask = 0x600080;
constexpr unsigned x_low_end = 0x3000, x_high_end = 0x10000;
constexpr unsigned y_high_end = 0x16000, y_small_end = 0xe000, y_middle_end = 0x14000;
constexpr unsigned small_y_mode = 0x80, medium_y_mode = 0x200080, large_y_mode = 0x400080;
constexpr unsigned cookie = 0xa55a33, canary = 0x123456, first_value = 0x800001;
constexpr unsigned second_value = 0xffffff, flags_pattern = 0xaa;
constexpr unsigned seed_count = 9, seed_words = 18, waits = 6;
constexpr unsigned jump = 0x0af080, immediate_omr = 0x05f43a;
constexpr unsigned absolute_r6 = 0x667000, read_bit = 0x8000, y_bit = 0x80000;
constexpr unsigned padding = 16, bound = 128, stop = 12, illegal = 7;
constexpr unsigned first_memory = 8, second_memory = 10;
constexpr std::array omrs{0x300U, 0x380U, 0x200380U, 0x400380U, 0x600380U, 0x600389U};
constexpr std::array x_addresses{
    0x3feU, 0x400U, 0x2c34U, 0x2ffcU, 0xc234U, 0xdffeU, 0xe000U, 0xfffcU};
constexpr std::array y_addresses{
    0x3feU, 0x400U, 0x1c34U, 0x1ffcU, 0xc234U, 0xdffeU, 0xe000U, 0xfffcU};

struct Input {
  unsigned omr, entry, address, flags;
  bool load, y;
};

unsigned opcode(const Input sample) {
  return absolute_r6 | (sample.load ? read_bit : 0U) | (sample.y ? y_bit : 0U);
}

unsigned y_end(const unsigned omr) {
  // UM tables3-1..3-6, indexed by MS then MSW; no model layout query.
  if ((omr & mode_mask) == small_y_mode) { return y_small_end; }
  if ((omr & mode_mask) == medium_y_mode) { return x_high_end; }
  if ((omr & mode_mask) == large_y_mode) { return y_middle_end; }
  return y_high_end;
}

bool mapped(const Input sample) {
  const auto lower = sample.y ? 0x2000U : x_low_end;
  const auto upper = sample.y ? y_end(sample.omr) : x_high_end;
  return sample.address + 2 < lower || (sample.address >= high_ram && sample.address + 2 < upper);
}

void frame(DspMachine &machine, const Input sample) {
  for (unsigned index = 0; index < padding; ++index) {
    machine.bus().write({.address = sample.entry + index, .value = 0});
  }
  for (unsigned index = 0; index < 2; ++index) {
    machine.bus().write({.address = sample.entry + 1 + (index * 2), .value = opcode(sample)});
    machine.bus().write(
        {.address = sample.entry + 2 + (index * 2), .value = sample.address + index});
  }
}

void install(DspMachine &machine, const Input sample) {
  test::DspProgram code;
  for (unsigned index = 0; index < 8; ++index) {
    code.immediate("r" + std::to_string(index), index == 6 ? cookie : reset_program_end + index);
  }
  code.immediate("sr", test::masked_status | sample.flags);
  code.word(immediate_omr);
  code.word(sample.omr);
  for (unsigned index = 0; index < waits; ++index) { code.word(0); }
  code.word(jump);
  code.word(staging);
  code.install(machine.bus());
  machine.bus().write({.address = staging, .value = jump});
  machine.bus().write({.address = staging + 1, .value = sample.entry});
  for (unsigned index = 2; index < padding; ++index) {
    machine.bus().write({.address = staging + index, .value = 0});
  }
  if (sample.entry < reset_program_end) { frame(machine, sample); }
  const auto space = sample.y ? DspSpace::y : DspSpace::x;
  for (const auto address :
       {sample.address - 1, sample.address, sample.address + 1, sample.address + 2}) {
    machine.bus().write({.space = space, .address = address, .value = canary});
  }
  if (sample.load) {
    machine.bus().write({.space = space, .address = sample.address, .value = first_value});
    machine.bus().write({.space = space, .address = sample.address + 1, .value = second_value});
  }
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  test::checkpoint::run(machine, seed_count);
  require(machine.cpu().state().pc == test::boot + seed_words,
          "Only actual own data/SR instructions precede the continuous clocked OMR path");
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
}

DspDebugState enter(DspFetchRun &run, DspMachine &machine, const Input sample) {
  constexpr unsigned branch_address = test::boot + seed_words + 2 + waits;
  bool installed = sample.entry < reset_program_end;
  for (unsigned index = 0; index < bound; ++index) {
    const auto cycle = run.advance_cycle(machine, test::fetch_clock::input());
    if (!installed && machine.bus().stable_program_context(sample.entry, sample.omr)) {
      require(
          machine.cpu().state().pc >= test::boot,
          "Wide code is installed after actual OMR switching and before its first target fetch");
      frame(machine, sample);
      installed = true;
    }
    if (cycle.pipeline && cycle.pipeline->retirement_published && cycle.pipeline->execute2 &&
        cycle.pipeline->execute2->address == branch_address) {
      const auto before = machine.cpu().debug_state();
      require(
          installed && before.cpu.pc == staging && before.cpu.omr == sample.omr &&
              before.native_instructions == seed_count + waits + 2 && before.cpu.r[6] == cookie &&
              before.cpu.sr == (test::masked_status | sample.flags),
          "Genuine OMR/NOP/JMP retirement establishes the independent main-program preconditions");
      return before;
    }
  }
  throw Fault("Own clocked memory-mode prelude exceeded its finite bound");
}

DspDebugState expected(DspDebugState state, const Input sample, const unsigned period) {
  if (period >= 4) {
    state.cpu.pc = sample.entry;
    ++state.native_instructions;
    state.native_cycles += 4;
  }
  if (sample.load && period >= first_memory) { state.cpu.r[6] = first_value; }
  if (sample.load && period >= second_memory) { state.cpu.r[6] = second_value; }
  if (period >= first_memory) {
    ++state.cpu.pc;
    ++state.native_instructions;
    ++state.native_cycles;
  }
  for (const auto due : {first_memory + 1, second_memory + 1}) {
    if (period >= due) {
      state.cpu.pc += 2;
      ++state.native_instructions;
      state.native_cycles += 2;
    }
  }
  return state;
}

void cells(DspMachine &machine, const Input sample, const unsigned period) {
  const auto space = sample.y ? DspSpace::y : DspSpace::x;
  for (const auto address :
       {sample.address - 1, sample.address, sample.address + 1, sample.address + 2}) {
    unsigned value = canary;
    if (address == sample.address) {
      if (sample.load) {
        value = first_value;
      } else if (period >= first_memory) {
        value = cookie;
      }
    } else if (address == sample.address + 1) {
      if (sample.load) {
        value = second_value;
      } else if (period >= second_memory) {
        value = cookie;
      }
    }
    require(machine.bus().read({.space = space, .address = address}) == value,
            "Literal memory words and neighboring guards follow the two genuine E1 accesses");
  }
}

void footprint(DspMachine &machine,
               const DspFetchCycleResult &cycle,
               const Input sample,
               const unsigned period) {
  if (!cycle.pipeline || !cycle.pipeline->memory) { throw Fault("Missing genuine memory E1"); }
  const auto which = period == first_memory ? 0U : 1U;
  const DspProgramWord word{.address = sample.entry + 1 + (which * 2), .value = opcode(sample)};
  const DspProgramWord extension{.address = word.address + 1, .value = sample.address + which};
  const auto &memory = *cycle.pipeline->memory;
  require(cycle.bus.core_access_count == 2 &&
              memory.address.kind == DspAddressStageKind::absolute_long &&
              memory.address.word == word && memory.address.extension == extension &&
              cycle.pipeline->execute1 == word && cycle.pipeline->e1_extension == extension,
          "Native E1 and the real simultaneous F2 own exactly two core bus accesses");
  bool saw_data = false;
  bool saw_program = false;
  for (unsigned index = 0; index < cycle.bus.core_access_count; ++index) {
    const auto access = cycle.bus.core_accesses.at(index);
    const auto resource = machine.bus().memory_resource(access);
    if (access.target.space == DspSpace::p) {
      saw_program = true;
      const bool switched = sample.entry == wide_entry;
      const unsigned physical_word =
          switched ? 0x10000U + access.target.address - reset_program_end : access.target.address;
      require(
          resource == DspMemoryResource{.bank = switched ? DspSpace::x : DspSpace::p,
                                        .word = physical_word,
                                        .block_words = switched ? 8192U : 1024U},
          "Actual F2 retains the documented block size and existing explicit X-to-P relocation");
    } else {
      saw_data = true;
      require(resource ==
                  DspMemoryResource{.bank = sample.y ? DspSpace::y : DspSpace::x,
                                    .word = access.target.address,
                                    .block_words = sample.address < high_ram ? 1024U : 8192U},
              "The independent RAM operand retains its literal low1K or upper8K module");
    }
  }
  require(saw_data && saw_program,
          "No invented memory transaction replaces either native bus owner");
}

void step(DspFetchRun &run,
          DspMachine &machine,
          const DspDebugState &before,
          const Input sample,
          const std::uint64_t base) {
  const auto cycle = run.advance_cycle(machine, test::fetch_clock::input());
  if (!cycle.pipeline) { throw Fault("Native block observation lost its actual pipeline"); }
  const auto &phase = *cycle.pipeline;
  const auto period = static_cast<unsigned>(run.completed_cycles() - base);
  const bool retired = period == 4 || period == first_memory || period == first_memory + 1 ||
                       period == second_memory + 1;
  require(phase.retirement_published == retired && phase.agu_dependencies == 0 &&
              !phase.memory_source_stall,
          "Independent JMP4/NOP1/MOVE2 timing has no bank-derived or invented AGU stalls");
  const bool memory = period == first_memory || period == second_memory;
  require(phase.memory.has_value() == memory, "Only the two declared E1 periods access data RAM");
  if (memory) { footprint(machine, cycle, sample, period); }
  const auto wanted = expected(before, sample, period);
  if (machine.cpu().debug_state() != wanted) {
    const auto actual = machine.cpu().debug_state();
    std::cerr << "period=" << period << " omr=" << hex(sample.omr, 6)
              << " entry=" << hex(sample.entry, 6) << " PC=" << hex(actual.cpu.pc, 6) << '/'
              << hex(wanted.cpu.pc, 6) << " cycles=" << actual.native_cycles << '/'
              << wanted.native_cycles << '\n';
  }
  require(
      machine.cpu().debug_state() == wanted,
      "Every period retains independently predicted complete CPU/flags/counters/all16 stack slots");
  cells(machine, sample, period);
  require(run.elapsed_ticks() == run.completed_cycles() * test::pipeline::period,
          "Concurrent accesses consume no extra nominal clock or altered ledger");
  if (phase.decode_accepted && phase.decode &&
      (phase.decode->address == sample.entry + 1 || phase.decode->address == sample.entry + 3)) {
    machine.bus().write({.address = phase.decode->address, .value = illegal});
    machine.bus().write({.address = phase.decode->address + 1, .value = illegal});
  }
}

unsigned streams = 0, restores = 0, clones = 0;

void one(const Input sample) {
  DspMachine machine;
  install(machine, sample);
  DspFetchRun run(test::scalar_store::contract(), machine);
  const auto before = enter(run, machine, sample);
  const auto base = run.completed_cycles();
  const bool snapshots = sample.entry == wide_entry && sample.flags == flags_pattern &&
                         sample.address == (sample.y ? y_addresses.at(2) : x_addresses.at(2));
  std::vector<DspFetchCheckpoint> cuts;
  while (run.completed_cycles() < base + stop) {
    if (snapshots) { cuts.push_back(run.checkpoint(machine)); }
    step(run, machine, before, sample, base);
  }
  for (const auto &cut : cuts) {
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(cut, machine);
      while (run.completed_cycles() < base + stop) { step(run, machine, before, sample, base); }
      ++restores;
    }
    DspMachine target;
    DspFetchRun clone(cut, target);
    while (clone.completed_cycles() < base + stop) { step(clone, target, before, sample, base); }
    ++clones;
  }
  ++streams;
}

void cases(const unsigned omr, const unsigned entry, const bool y) {
  const auto addresses = y ? y_addresses : x_addresses;
  for (const unsigned address : addresses) {
    for (const bool load : {false, true}) {
      for (const unsigned flags : {0U, flags_pattern}) {
        const Input sample{
            .omr = omr, .entry = entry, .address = address, .flags = flags, .load = load, .y = y};
        if (mapped(sample)) { one(sample); }
      }
    }
  }
}
}

int main() {
  try {
    for (const unsigned omr : omrs) {
      for (const unsigned entry : {low_entry, wide_entry}) {
        if (omr == normal_mode && entry == wide_entry) { continue; }
        for (const bool y : {false, true}) { cases(omr, entry, y); }
      }
    }
    std::cout << "PASS CORE_BLOCKS streams=" << streams << " restores=" << restores
              << " clones=" << clones << '\n';
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
