// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_io.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp56300/core.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {
using namespace blofeld;
using namespace dsp56300;
constexpr DspRunClockInputs clocks{.extal_hz = 12000000,
                                   .timebase_hz = 540000000,
                                   .switch_timing = DspClockSwitch::reference_cycle_boundary};
constexpr Word boot = 0xff0000, entry = 0x100, normal_omr = 0x300;
constexpr Word masked_status = 0xc00300, canary = 0x654321, pattern = 0xa55aa5;
constexpr Word movec_omr = 0x05f43a, movec_sr = 0x05f439, immediate_x0 = 0x44f400;
constexpr Word long_jump = 0x0af080, positive = 32, negative = word_mask - 31;
constexpr unsigned words = 64, bits = 24, byte_mask = 0xff, base_cost = 5, effect_bound = 16;
constexpr std::uint64_t hold_ticks = 90, recovery_ticks = 135;
constexpr std::array deltas{positive, negative};

struct Sample {
  unsigned address = 0, bit = 0, flags = 0;
  Word displacement = positive;
  bool y = false, set = false, level = false;
};

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

Word opcode(const Sample sample) {
  // DSP56300FM13-27/30:AA6 BRCLR/BRSET,not register,EA or subroutine forms.
  constexpr Word family = 0x0c8080;
  constexpr Word y_bit = 0x40;
  constexpr Word set_bit = 0x20;
  constexpr unsigned address_shift = 8;
  return family | (sample.address << address_shift) | (sample.y ? y_bit : 0U) |
         (sample.set ? set_bit : 0U) | sample.bit;
}

Word value(const Sample sample) {
  const Word mask = 1U << sample.bit;
  return ((pattern ^ (sample.address << 8U)) & ~mask) | (sample.level ? mask : 0U);
}

DspAddress operand(const Sample sample) {
  return {.space = sample.y ? DspSpace::y : DspSpace::x, .address = sample.address};
}

PendingInstruction pending(const DspMachine &machine) {
  const auto result = machine.cpu().pending_instruction();
  if (!result) { throw std::runtime_error("Missing actual staged instruction effect"); }
  return *result;
}

void complete(DspMachine &machine) {
  auto step = machine.begin_fetch();
  for (unsigned index = 0; index < effect_bound && step.pending; ++index) {
    step = machine.advance(step.pending->ticket);
  }
  require(step.retired && step.fault.kind == FaultKind::none,
          "Own literal setup failed to retire through the actual core and board adapter");
}

void prepare(DspMachine &machine, const Sample sample) {
  const std::array<Word, 8> setup{movec_omr,
                                  normal_omr,
                                  movec_sr,
                                  masked_status | sample.flags,
                                  immediate_x0,
                                  canary,
                                  long_jump,
                                  entry};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot, .words = setup});
  machine.begin_hardware_reset(
      {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 3});
  require(machine.advance_hardware_reset_ticks(hold_ticks) == hold_ticks, "Lost real reset hold");
  machine.release_hardware_reset();
  require(machine.advance_hardware_reset_ticks(recovery_ticks) == recovery_ticks,
          "Lost real reset recovery");
  for (unsigned instruction = 0; instruction < 4; ++instruction) { complete(machine); }
  const auto registers = machine.cpu().registers();
  require(registers.pc == entry && registers.omr == normal_omr &&
              registers.sr == (masked_status | sample.flags) && registers.x == canary,
          "Real reset and native setup must establish independently encoded PC,OMR,CCR and X0");
  machine.bus().write({.address = entry, .value = opcode(sample)});
  machine.bus().write({.address = entry + 1, .value = sample.displacement});
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (unsigned address = 0; address < words; ++address) {
      machine.bus().write({.space = space, .address = address, .value = canary});
    }
  }
  machine.bus().write(
      {.space = operand(sample).space, .address = sample.address, .value = value(sample)});
}

struct Oracle {
  Registers registers;
  Progress progress;
  unsigned opcodes = 0, extensions = 0, reads = 0;
};

void observe(const PendingInstruction &actual, const Sample sample, Oracle &oracle) {
  const auto &effect = actual.effect;
  require(effect.pc == entry && effect.omr == normal_omr,
          "Every effect retains the actual branch PC and OMR");
  if (effect.kind == EffectKind::fetch_opcode) {
    ++oracle.opcodes;
    require(effect.space == Space::program && effect.address == entry,
            "One actual opcode read owns the manually encoded AA6 instruction");
  } else if (effect.kind == EffectKind::fetch_extension) {
    ++oracle.extensions;
    require(effect.space == Space::program && effect.address == entry + 1,
            "One actual extension read owns the signed24 displacement");
  } else if (effect.kind == EffectKind::read) {
    ++oracle.reads;
    require(effect.space == (sample.y ? Space::y : Space::x) && effect.address == sample.address &&
                actual.opcode == opcode(sample),
            "RAM supplies one genuine whole-word read in the decoded source space");
  } else {
    require(effect.kind == EffectKind::pcu_update || effect.kind == EffectKind::retire,
            "An AA6 branch cannot manufacture bit-I/O,stores,AGU or ALU writes");
  }
}

void memory(DspMachine &machine, const Sample sample) {
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (unsigned address = 0; address < words; ++address) {
      const auto expected =
          space == operand(sample).space && address == sample.address ? value(sample) : canary;
      require(machine.bus().read({.space = space, .address = address}) == expected,
              "The original RAM word,all AA6 canaries and the opposite bank remain unchanged");
    }
  }
}

void verify(DspMachine &machine, const Sample sample, Oracle oracle) {
  oracle.registers.pc =
      sample.level == sample.set ? (entry + sample.displacement) & word_mask : entry + 2;
  ++oracle.progress.instructions;
  oracle.progress.estimated_cycles += base_cost;
  require(
      machine.cpu().registers() == oracle.registers && machine.cpu().progress() == oracle.progress,
      "Independent bit truth,signed PC,all CCR/register/stack fields and manual five-cycle cost");
  require(oracle.opcodes == 1 && oracle.extensions == 1 && oracle.reads == 1,
          "A branch must consume exactly one actual opcode,extension and RAM word");
  require(machine.clock().elapsed_ticks() == hold_ticks + recovery_ticks && !machine.io().failure(),
          "Untimed instruction qualification cannot invent board clocks or hide adapter failures");
  memory(machine, sample);
}

void finish(DspMachine &machine, const Sample sample, Oracle oracle) {
  for (unsigned index = 0; index < effect_bound; ++index) {
    const auto actual = pending(machine);
    observe(actual, sample, oracle);
    const auto step = machine.advance(actual.ticket);
    require(step.fault.kind == FaultKind::none, "Actual AA6 execution produced a native fault");
    if (step.retired) {
      verify(machine, sample, oracle);
      return;
    }
  }
  throw std::runtime_error("AA6 branch exceeded its bounded actual effect sequence");
}

unsigned cases = 0, restores = 0, clones = 0, snapshot_cases = 0;

void one(const Sample sample) {
  DspMachine machine(clocks);
  prepare(machine, sample);
  Oracle oracle{.registers = machine.cpu().registers(), .progress = machine.cpu().progress()};
  static_cast<void>(machine.begin_fetch());
  const bool snapshots = sample.flags == byte_mask && sample.address == (sample.y ? 63U : 0U) &&
                         sample.bit == (sample.y ? 23U : 0U) &&
                         sample.displacement == (sample.y ? negative : positive);
  if (!snapshots) {
    finish(machine, sample, oracle);
  } else {
    ++snapshot_cases;
    for (unsigned index = 0; index < effect_bound; ++index) {
      const auto saved = machine.checkpoint();
      finish(machine, sample, oracle);
      machine.restore(saved);
      ++restores;
      DspMachine fresh(clocks);
      fresh.restore(saved);
      finish(fresh, sample, oracle);
      ++clones;
      const auto actual = pending(machine);
      observe(actual, sample, oracle);
      const auto step = machine.advance(actual.ticket);
      require(step.fault.kind == FaultKind::none, "Restored AA6 continuation failed");
      if (step.retired) {
        verify(machine, sample, oracle);
        break;
      }
      require(index + 1 < effect_bound, "All-effect checkpoint cuts did not terminate");
    }
  }
  ++cases;
}

void variants(Sample sample) {
  for (const bool set : {false, true}) {
    for (const bool level : {false, true}) {
      for (const unsigned flags : {0U, byte_mask}) {
        for (const auto delta : deltas) {
          sample.set = set;
          sample.level = level;
          sample.flags = flags;
          sample.displacement = delta;
          one(sample);
        }
      }
    }
  }
}

void all_flags(Sample sample) {
  for (const bool set : {false, true}) {
    for (const bool level : {false, true}) {
      for (unsigned flags = 1; flags < byte_mask; ++flags) {
        sample.set = set;
        sample.level = level;
        sample.flags = flags;
        one(sample);
      }
    }
  }
}

void run() {
  for (unsigned address = 0; address < words; ++address) {
    for (unsigned bit = 0; bit < bits; ++bit) {
      for (const bool y : {false, true}) { variants({.address = address, .bit = bit, .y = y}); }
    }
  }
  for (const unsigned address : {0U, 63U}) {
    for (const unsigned bit : {0U, 23U}) {
      for (const bool y : {false, true}) { all_flags({.address = address, .bit = bit, .y = y}); }
    }
  }
  constexpr unsigned expected_cases = 57280;
  constexpr unsigned snapshot_inputs = 8;
  constexpr unsigned required_read_cuts = 3;
  require(cases == expected_cases && snapshot_cases == snapshot_inputs && restores == clones &&
              restores >= snapshot_inputs * required_read_cuts,
          "The full literal encoding/CCR corpus and every selected actual effect cut must execute");
}
}

int main() {
  try {
    std::cout << "OWN_RAM_BIT begin\n" << std::flush;
    run();
    std::cout << "OWN_RAM_BIT cases=" << cases << " snapshot-cases=" << snapshot_cases
              << " restores=" << restores << " clones=" << clones << " passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "OWN_RAM_BIT cases=" << cases << ": " << error.what() << '\n';
    return 1;
  }
}
