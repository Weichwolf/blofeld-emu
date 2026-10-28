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
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned entry = 0x100, first_cell = 0x3ff, last_cell = 0x408;
constexpr unsigned stored_value = 0x123456, source_value = 0xabcdef, sentinel = 0x654321;
constexpr unsigned plain_omr = 0x300;
constexpr unsigned x0_load = 0x44c000, x0_store = 0x444000, y_bit = 0x80000;
constexpr unsigned absolute_jump = 0x0af080;
constexpr unsigned full_flags = 0xff;

struct Vector {
  unsigned mode, pointer, offset, modifier, address, after;
};

struct Case {
  Vector vector;
  unsigned reg, flags;
  bool y, load;
};

unsigned opcode(const Case sample) {
  return (sample.load ? x0_load : x0_store) | (sample.y ? y_bit : 0U) |
         (sample.vector.mode << 11U) | (sample.reg << 8U);
}

DspDebugState prepare(DspMachine &machine, const Case sample) {
  const auto reg = std::to_string(sample.reg);
  DspProgram program;
  program.immediate("omr", plain_omr);
  program.immediate("m" + reg, sample.vector.modifier);
  program.immediate("r" + reg, sample.vector.pointer);
  program.immediate("n" + reg, sample.vector.offset);
  program.immediate("x0", stored_value);
  program.immediate("x1", sentinel);
  program.immediate("y0", sentinel);
  program.immediate("y1", stored_value);
  program.immediate("sr", masked_status | sample.flags);
  for (unsigned index = 0; index < 5; ++index) { program.word(0); }
  program.word(absolute_jump);
  program.word(entry);
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::until(machine, entry);
  for (unsigned index = 0; index < 16; ++index) { machine.bus().write({.address = entry + index}); }
  machine.bus().write({.address = entry, .value = opcode(sample)});
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (unsigned address = first_cell; address <= last_cell; ++address) {
      const bool selected = (space == DspSpace::y) == sample.y && address == sample.vector.address;
      machine.bus().write({.space = space,
                           .address = address,
                           .value = selected && sample.load ? source_value : sentinel});
    }
  }
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == entry && before.cpu.omr == plain_omr &&
              before.cpu.r.at(sample.reg) == sample.vector.pointer &&
              before.cpu.n.at(sample.reg) == sample.vector.offset &&
              before.cpu.m.at(sample.reg) == sample.vector.modifier &&
              (before.cpu.x & word_mask) == stored_value &&
              before.cpu.sr == (masked_status | sample.flags),
          "Real native setup establishes the exact independent address and flag inputs");
  return before;
}

constexpr unsigned initial_pointer = 0x403;
constexpr std::array reserved_modifiers{0x8000U, 0x8002U, 0xc000U, 0xff80U, 0xfffeU};

Case sample(
    const unsigned raw, const unsigned mode, const unsigned reg, const bool y, const bool load) {
  return {.vector = {.mode = mode,
                     .pointer = initial_pointer,
                     .offset = 3,
                     .modifier = raw,
                     .address = initial_pointer,
                     .after = initial_pointer},
          .reg = reg,
          .flags = full_flags,
          .y = y,
          .load = load};
}

DspFetchRunInputs contract() {
  auto result = fetch_clock::contract();
  result.execution = DspReferenceExecution::staged_register_boundaries;
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

void reserved(const Case input) {
  DspMachine machine;
  const auto before = prepare(machine, input);
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  DspFetchRun run(contract(), machine);
  for (unsigned index = 0; index < 3; ++index) {
    static_cast<void>(run.advance_cycle(machine, fetch_clock::input()));
  }
  const auto saved = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    try {
      const auto cycle = run.advance_cycle(machine, fetch_clock::input());
      if (!cycle.pipeline || !cycle.pipeline->ag1_address) {
        throw Fault("Reserved modifier control lost AG1");
      }
      std::cout << "UNEXPECTED_RESERVED_ADMISSION mode=" << input.vector.mode
                << " M=" << hex(input.vector.modifier, 6)
                << " EA=" << hex(cycle.pipeline->ag1_address->update.address, 6)
                << " pc=" << hex(entry, 6) << '\n';
    } catch (const Fault &error) {
      const std::string message = error.what();
      require(
          message.contains("reserved modifier") && message.contains("pc=0x000100") &&
              message.contains("omr=0x000300") &&
              message.contains("m=" + hex(input.vector.modifier, 6)) && run.faulted() &&
              run.completed_cycles() == 3 && machine.cpu().debug_state() == before,
          "Reserved Mn arithmetic must retain actual modifier/PC/OMR and no pointer/data result");
      continue;
    }
    throw Fault("Reserved Mn address arithmetic must fail before native pointer/data effects");
  }
}

void unused(const Case input) {
  DspMachine machine;
  auto expected = prepare(machine, input);
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  DspFetchRun run(contract(), machine);
  for (unsigned index = 0; index < 7; ++index) {
    static_cast<void>(run.advance_cycle(machine, fetch_clock::input()));
  }
  if (input.load) {
    constexpr std::uint64_t high_half = 0xffffff000000;
    expected.cpu.x = (expected.cpu.x & high_half) | source_value;
  }
  ++expected.cpu.pc;
  ++expected.native_instructions;
  ++expected.native_cycles;
  require(machine.cpu().debug_state() == expected && !run.faulted(),
          "FM4.3.4/Table4-1 permits arbitrary Mn storage when the no-update access ignores Mn");
  const auto wanted = input.load ? source_value : stored_value;
  require(machine.bus().read(
              {.space = input.y ? DspSpace::y : DspSpace::x, .address = initial_pointer}) == wanted,
          "The no-update operand is the actual Rn cell,independent of unused modifier bits");
}

unsigned forms(Case input, const bool no_update) {
  if (no_update) {
    unused(input);
    return 1;
  }
  constexpr std::array modes{0U, 1U, 2U, 3U, 5U, 7U};
  for (const auto mode : modes) {
    input.vector.mode = mode;
    reserved(input);
  }
  return static_cast<unsigned>(modes.size());
}

unsigned matrix(const bool no_update) {
  unsigned count = 0;
  for (const auto low : reserved_modifiers) {
    for (const unsigned upper : {0U, 0x7fU, 0xffU}) {
      const auto raw = (upper << 16U) | low;
      for (const unsigned reg : {0U, 7U}) {
        for (const bool y : {false, true}) {
          for (const bool load : {false, true}) {
            count += forms(sample(raw, 4, reg, y, load), no_update);
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
    const auto rejected = matrix(false);
    const auto permitted = matrix(true);
    std::cout << rejected << " reserved-modifier refusals/two suffixes," << permitted
              << " arbitrary-Mn no-update controls passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
