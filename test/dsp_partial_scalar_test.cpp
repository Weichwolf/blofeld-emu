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
#include <string_view>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned entry = 0x100;
constexpr unsigned data = 0x407;
constexpr std::array pointer_results{0x404U, 0x402U, 0x406U, 0x400U, 0x407U};
constexpr unsigned canary = 0x654321;
constexpr unsigned ordinary_omr = 0x300;
constexpr unsigned modifier = 0xff0007;
constexpr unsigned absolute_jump = 0x0af080;
constexpr unsigned extension_mask = 0xff;
constexpr unsigned extension_sign = 0x80;
constexpr unsigned extension_fill = 0xffff00;
constexpr unsigned mode_shift = 11;
constexpr unsigned register_shift = 8;
constexpr unsigned y_bit = 0x80000;
constexpr std::uint64_t other_accumulator = 0x11223344556677;
// FM13-118/122 and Table12-14: A0/A1/A2/B0/B1/B2, W=0,EA=1,no ALU.
constexpr std::array source_words{0x504000U, 0x544000U, 0x524000U, 0x514000U, 0x554000U, 0x534000U};
constexpr std::array<std::uint64_t, 4> values{
    0, 0x7f7fffff123456, 0x80800000654321, 0xffffffffffabcd};
constexpr std::array flags{0U, 0x3fU, 0xc0U, 0xffU};

struct Case {
  unsigned source = 0;
  std::uint64_t value = 0;
  unsigned scale = 0;
  unsigned ccr = 0;
  bool y = false;
  unsigned mode = 3;
  unsigned reg = 0;
};

unsigned opcode(const Case sample) {
  return source_words.at(sample.source) | (sample.mode << mode_shift) |
         (sample.reg << register_shift) | (sample.y ? y_bit : 0U);
}

unsigned stored(const Case sample) {
  const auto part = sample.source % 3;
  auto value = static_cast<unsigned>((sample.value >> (part * word_bits)) &
                                     (part == 2 ? extension_mask : word_mask));
  // FM3-3: individual parts bypass the limiter; EXT sign-extends from bit7.
  if (part == 2 && (value & extension_sign) != 0) { value |= extension_fill; }
  return value;
}

void cells(DspMachine &machine, const Case sample, const bool committed) {
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const unsigned address : {data - 1, data, data + 1}) {
      const bool selected = committed && address == data && (space == DspSpace::y) == sample.y;
      require(machine.bus().read({.space = space, .address = address}) ==
                  (selected ? stored(sample) : canary),
              "Independent partial-word value preserves other space and neighboring cells");
    }
  }
}

DspDebugState prepare(DspMachine &machine, const Case sample) {
  DspProgram program;
  program.immediate("omr", ordinary_omr);
  const auto register_name = std::to_string(sample.reg);
  program.immediate("m" + register_name, modifier);
  program.immediate("r" + register_name, data);
  program.immediate("n" + register_name, 3);
  for (const bool b : {false, true}) {
    const auto value = b == (sample.source >= 3) ? sample.value : other_accumulator;
    const std::string name = b ? "b" : "a";
    for (unsigned part = 0; part < 3; ++part) {
      program.immediate(name + std::to_string(part),
                        (value >> (part * word_bits)) & (part == 2 ? extension_mask : word_mask));
    }
  }
  program.immediate("sr", masked_status | sample.scale | sample.ccr);
  for (unsigned index = 0; index < 5; ++index) { program.word(0); }
  program.word(absolute_jump);
  program.word(entry);
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::until(machine, entry);
  for (unsigned address = entry; address < entry + 16; ++address) {
    machine.bus().write({.address = address});
  }
  machine.bus().write({.address = entry, .value = opcode(sample)});
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const unsigned address : {data - 1, data, data + 1}) {
      machine.bus().write({.space = space, .address = address, .value = canary});
    }
  }
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == entry && before.cpu.r.at(sample.reg) == data &&
              before.cpu.m.at(sample.reg) == modifier && before.cpu.n.at(sample.reg) == 3 &&
              before.cpu.a == (sample.source < 3 ? sample.value : other_accumulator) &&
              before.cpu.b == (sample.source >= 3 ? sample.value : other_accumulator) &&
              before.cpu.sr == (masked_status | sample.scale | sample.ccr),
          "Native own-word seeds establish complete accumulator/address/CCR inputs");
  return before;
}

void native(const Case sample) {
  DspMachine machine;
  auto expected = prepare(machine, sample);
  expected.cpu.r.at(sample.reg) = pointer_results.at(sample.mode);
  ++expected.cpu.pc;
  ++expected.native_instructions;
  ++expected.native_cycles;
  machine.cpu().step();
  const auto actual = machine.cpu().debug_state();
  if (actual != expected) {
    std::cerr << "opcode=" << hex(opcode(sample), 6) << " value=" << hex(sample.value, 14)
              << " scale=" << hex(sample.scale, 3) << " flags=" << hex(sample.ccr, 2)
              << " sr=" << hex(actual.cpu.sr, 6) << '/' << hex(expected.cpu.sr, 6) << '\n';
  }
  require(actual == expected,
          "Independent partial stores retain all CPU/CCR fields apart from one wrapped "
          "R/PC/counter step");
  cells(machine, sample, true);
}

void refusal(const Case sample) {
  DspMachine machine;
  const auto before = prepare(machine, sample);
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  auto inputs = fetch_clock::contract();
  inputs.execution = DspReferenceExecution::staged_register_boundaries;
  inputs.pipeline = DspFetchPipeline::automatic_memory;
  DspFetchRun run(inputs, machine);
  for (unsigned period = 0; period < 2; ++period) {
    static_cast<void>(run.advance_cycle(machine, fetch_clock::input()));
  }
  const auto saved = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    const auto error = checkpoint::rejects(
        [&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); },
        "Baseline unexpectedly admitted a partial-accumulator store");
    require(error.contains("Unqualified") && error.contains("pc=0x000100") &&
                error.contains("omr=0x000300") && run.completed_cycles() == 2 && run.faulted() &&
                machine.cpu().debug_state() == before,
            "Automatic baseline must refuse with context before native AGU/store/CCR effects");
    cells(machine, sample, false);
  }
}

DspFetchRunInputs contract() {
  auto inputs = fetch_clock::contract();
  inputs.execution = DspReferenceExecution::staged_register_boundaries;
  inputs.pipeline = DspFetchPipeline::automatic_memory;
  return inputs;
}

void period(DspFetchRun &run, DspMachine &machine, const DspDebugState &before, const Case sample) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto count = cycle.completed_cycles;
  auto expected = before;
  if (count >= 5) { expected.cpu.r.at(sample.reg) = pointer_results.at(sample.mode); }
  if (count == 7) {
    ++expected.cpu.pc;
    ++expected.native_instructions;
    ++expected.native_cycles;
  }
  require(machine.cpu().debug_state() == expected,
          "Independent partial-store AG2/E1/E2 fields preserve source and every CCR bit");
  if (!cycle.pipeline) { throw Fault("Partial-store observation requires actual pipeline state"); }
  const auto &phase = *cycle.pipeline;
  require(phase.memory.has_value() == (count == 6) && !phase.memory_source_stall,
          "Standalone partial store has one real E1 and no fabricated source stall");
  if (count == 4) {
    const auto &address = phase.ag1_address;
    require(address && !address->published && address->pointer_update == (sample.mode != 4) &&
                address->update.index == sample.reg && address->update.before == data &&
                address->update.value == pointer_results.at(sample.mode) &&
                address->update.address == data,
            "Native AG1 samples the independently declared R/N/M result without early CPU effects");
  }
  if (count == 6) {
    if (!phase.memory) { throw Fault("Partial-store E1 must expose its real memory operation"); }
    const auto &effect = *phase.memory;
    require(
        effect.data.value == stored(sample) && effect.data.target.address == data &&
            effect.data.target.space == (sample.y ? DspSpace::y : DspSpace::x),
        "The native E1 event carries the literal part value, space and original effective address");
  }
  cells(machine, sample, count >= 6);
}

void automatic(const Case sample) {
  DspMachine machine;
  const auto before = prepare(machine, sample);
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  DspFetchRun run(contract(), machine);
  for (unsigned index = 0; index < 7; ++index) { period(run, machine, before, sample); }
}

void snapshots(const Case sample) {
  for (unsigned cut = 0; cut < 7; ++cut) {
    DspMachine machine;
    const auto before = prepare(machine, sample);
    static_cast<void>(
        machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
    DspFetchRun run(contract(), machine);
    for (unsigned index = 0; index < cut; ++index) { period(run, machine, before, sample); }
    const auto saved = run.checkpoint(machine);
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      for (unsigned index = cut; index < 7; ++index) { period(run, machine, before, sample); }
    }
  }
}

void guard(const Case sample, const unsigned word, const bool scaling) {
  DspMachine machine;
  const auto before = prepare(machine, sample);
  machine.bus().write({.address = entry, .value = word});
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  DspFetchRun run(contract(), machine);
  for (unsigned index = 0; index < 2; ++index) {
    static_cast<void>(run.advance_cycle(machine, fetch_clock::input()));
  }
  const auto saved = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    const auto error = checkpoint::rejects(
        [&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); },
        "Unsupported partial-store form/context must refuse before effects");
    require(error.contains("Unqualified") && error.contains("pc=0x000100") &&
                error.contains("omr=0x000300") && run.completed_cycles() == 2 && run.faulted() &&
                machine.cpu().debug_state() == before,
            "Guard recovery retains complete pre-AGU/memory/CCR state and failure context");
    if (scaling) {
      require(error.contains("scaling read P:") && error.contains("sr="),
              "Reserved scaling remains an explicit contextual refusal");
    }
    cells(machine, sample, false);
  }
}

unsigned guards() {
  constexpr unsigned reserved_scale = 0xc00;
  constexpr std::array suffixes{0x9800U, 0x1813U, 0x2800U, 0x3000U, 0x3800U};
  unsigned count = 0;
  for (unsigned source = 0; source < source_words.size(); ++source) {
    const Case sample{.source = source, .value = values[2], .ccr = flags[3]};
    for (const auto suffix : suffixes) {
      guard(sample, source_words[source] | suffix, false);
      ++count;
    }
    for (const bool y : {false, true}) {
      const Case reserved{
          .source = source, .value = values[2], .scale = reserved_scale, .ccr = flags[3], .y = y};
      guard(reserved, opcode(reserved), true);
      ++count;
    }
  }
  return count;
}

unsigned addresses(const unsigned source, const unsigned mode) {
  unsigned count = 0;
  for (unsigned reg = 0; reg < 8; ++reg) {
    for (const bool y : {false, true}) {
      for (unsigned vector = 2; vector < values.size(); ++vector) {
        automatic({.source = source,
                   .value = values[vector],
                   .scale = scale_up,
                   .ccr = flags[3],
                   .y = y,
                   .mode = mode,
                   .reg = reg});
        ++count;
      }
    }
  }
  return count;
}

unsigned expanded() {
  unsigned count = 0;
  for (unsigned source = 0; source < source_words.size(); ++source) {
    for (unsigned mode = 0; mode < 5; ++mode) {
      count += addresses(source, mode);
      for (const bool y : {false, true}) {
        snapshots({.source = source,
                   .value = values[2],
                   .scale = scale_down,
                   .ccr = flags[3],
                   .y = y,
                   .mode = mode,
                   .reg = y ? 7U : 0U});
      }
    }
  }
  return count;
}

unsigned matrix(const bool staged) {
  unsigned count = 0;
  for (unsigned source = 0; source < source_words.size(); ++source) {
    for (const auto value : values) {
      for (const unsigned scale : {0U, scale_down, scale_up}) {
        for (const auto ccr : flags) {
          for (const bool y : {false, true}) {
            const Case sample{.source = source, .value = value, .scale = scale, .ccr = ccr, .y = y};
            if (staged) {
              automatic(sample);
            } else {
              native(sample);
            }
            ++count;
          }
        }
      }
    }
  }
  return count;
}
}

int main(const int count, char **arguments) {
  try {
    const std::string_view mode = count == 2 ? arguments[1] : "--native";
    if (count > 2 || (mode != "--native" && mode != "--candidate" && mode != "--expanded" &&
                      mode != "--expect-refusal")) {
      throw Fault("Usage: partial-store-probe [--native|--candidate|--expanded|--expect-refusal]");
    }
    if (mode == "--expanded") {
      const auto cases = expanded();
      const auto rejected = guards();
      std::cout << cases << " addressing cases,420 cuts/840 suffixes," << rejected
                << " strict guards/two suffixes each passed\n";
    } else if (mode == "--native" || mode == "--candidate") {
      const auto cases = matrix(mode == "--candidate");
      std::cout << cases << " independent partial-store CPU/memory/CCR cases passed\n";
    } else {
      for (unsigned source = 0; source < source_words.size(); ++source) {
        refusal({.source = source, .value = values[2], .scale = scale_up, .ccr = flags[3]});
      }
      std::cout << "Six automatic partial-store baseline refusals/two replays passed\n";
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
