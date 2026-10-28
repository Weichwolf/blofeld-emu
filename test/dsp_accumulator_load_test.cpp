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

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned entry = 0x100, data = 0x407, canary = 0x654321;
constexpr unsigned ordinary_omr = 0x300, modifier = 0xff0007;
constexpr unsigned absolute_jump = 0x0af080, y_bit = 0x80000;
constexpr unsigned mode_shift = 11, register_shift = 8;
constexpr unsigned extension_mask = 0xff, word_sign = 0x800000;
constexpr std::uint64_t accumulator_mask = 0xffffffffffffff;
constexpr std::uint64_t a_seed = 0x12a1b2c3d4e5f6, b_seed = 0xe93f4a5b6c7d8e;
// FM13-118/122, Table12-14: standalone W=1,EA=1; A0/A1/A2/B0/B1/B2/A/B.
constexpr std::array words{
    0x50c000U, 0x54c000U, 0x52c000U, 0x51c000U, 0x55c000U, 0x53c000U, 0x56c000U, 0x57c000U};
constexpr std::array values{0U, 1U, 0x7fU, 0x80U, 0x7fffffU, 0x800000U, 0xffff80U, 0xffffffU};
constexpr std::array flags{0U, 0x3fU, 0xc0U, 0xffU};
constexpr std::array pointers{0x404U, 0x402U, 0x406U, 0x400U, 0x407U};

struct Case {
  unsigned destination = 0, value = 0, scale = 0, ccr = 0;
  bool y = false;
  unsigned mode = 3, reg = 0;
};

bool target_b(const Case sample) {
  return sample.destination == 7 || (sample.destination >= 3 && sample.destination < 6);
}

unsigned opcode(const Case sample) {
  return words.at(sample.destination) | (sample.y ? y_bit : 0U) | (sample.mode << mode_shift) |
         (sample.reg << register_shift);
}

std::uint64_t loaded(const Case sample) {
  const auto initial = target_b(sample) ? b_seed : a_seed;
  // FM3.1.3 and 12.2.1: full destination sign-extends bit23 and zero-fills low24.
  if (sample.destination >= 6) {
    constexpr std::uint64_t negative_extension = 0xff000000000000;
    return (static_cast<std::uint64_t>(sample.value) << word_bits) |
           ((sample.value & word_sign) != 0 ? negative_extension : 0);
  }
  // Individual destinations preserve every other part; A2/B2 accept only low8.
  const auto part = sample.destination % 3;
  const auto shift = part * word_bits;
  const auto mask = static_cast<std::uint64_t>(part == 2 ? extension_mask : word_mask) << shift;
  return (initial & (accumulator_mask ^ mask)) |
         ((static_cast<std::uint64_t>(sample.value) << shift) & mask);
}

void cells(DspMachine &machine, const Case sample) {
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto address : {data - 1, data, data + 1}) {
      const bool selected = address == data && (space == DspSpace::y) == sample.y;
      require(machine.bus().read({.space = space, .address = address}) ==
                  (selected ? sample.value : canary),
              "Native reads retain source, neighboring cells and opposite-space sentinels");
    }
  }
}

DspDebugState prepare(DspMachine &machine, const Case sample, const bool known_source = true) {
  DspProgram program;
  program.immediate("omr", ordinary_omr);
  const auto reg = std::to_string(sample.reg);
  program.immediate("m" + reg, modifier);
  program.immediate("r" + reg, data);
  program.immediate("n" + reg, 3);
  for (const bool b : {false, true}) {
    const auto value = b ? b_seed : a_seed;
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
  for (unsigned index = 0; index < 16; ++index) { machine.bus().write({.address = entry + index}); }
  machine.bus().write({.address = entry, .value = opcode(sample)});
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto address : {data - 1, data, data + 1}) {
      const bool selected = address == data && (space == DspSpace::y) == sample.y;
      if (!selected || known_source) {
        machine.bus().write(
            {.space = space, .address = address, .value = selected ? sample.value : canary});
      }
    }
  }
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == entry && before.cpu.a == a_seed && before.cpu.b == b_seed &&
              before.cpu.r.at(sample.reg) == data && before.cpu.n.at(sample.reg) == 3 &&
              before.cpu.m.at(sample.reg) == modifier && before.cpu.omr == ordinary_omr &&
              before.cpu.sr == (masked_status | sample.scale | sample.ccr),
          "Actual own native instructions establish both 56-bit destinations and exact modes");
  return before;
}

void native(const Case sample) {
  DspMachine machine;
  auto expected = prepare(machine, sample);
  expected.cpu.r.at(sample.reg) = pointers.at(sample.mode);
  auto &target = target_b(sample) ? expected.cpu.b : expected.cpu.a;
  target = loaded(sample);
  ++expected.cpu.pc;
  ++expected.native_instructions;
  ++expected.native_cycles;
  machine.cpu().step();
  const auto actual = machine.cpu().debug_state();
  if (actual != expected) {
    std::cerr << "word=" << hex(opcode(sample), 6) << " source=" << hex(sample.value, 6)
              << " A=" << hex(actual.cpu.a, 14) << '/' << hex(expected.cpu.a, 14)
              << " B=" << hex(actual.cpu.b, 14) << '/' << hex(expected.cpu.b, 14) << '\n';
  }
  require(actual == expected,
          "Independent native load expectation covers full CPU/CCR, part preservation and AGU");
  cells(machine, sample);
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
  if (count >= 5) { expected.cpu.r.at(sample.reg) = pointers.at(sample.mode); }
  if (count >= 6) {
    auto &target = target_b(sample) ? expected.cpu.b : expected.cpu.a;
    target = loaded(sample);
  }
  if (count == 7) {
    ++expected.cpu.pc;
    ++expected.native_instructions;
    ++expected.native_cycles;
  }
  require(machine.cpu().debug_state() == expected,
          "Independent AG2/E1/E2 load phases retain complete CPU/CCR state");
  if (!cycle.pipeline) { throw Fault("A scalar load must retain actual pipeline state"); }
  const auto &phase = *cycle.pipeline;
  require(phase.memory.has_value() == (count == 6) && !phase.memory_source_stall,
          "A standalone load has one native memory receipt and no source stall");
  if (count == 4) {
    const auto &address = phase.ag1_address;
    require(address && !address->published && address->pointer_update == (sample.mode != 4) &&
                address->update.index == sample.reg && address->update.before == data &&
                address->update.value == pointers.at(sample.mode) &&
                address->update.address == data,
            "Native AG1 carries independent N/M/EA and conditional pointer-update expectations");
  }
  if (count == 6) {
    if (!phase.memory) { throw Fault("Native load E1 lost its real data receipt"); }
    const auto &effect = *phase.memory;
    require(effect.data.kind == DspNativeAccessKind::read && effect.data.value == sample.value &&
                effect.data.target.address == data &&
                effect.data.target.space == (sample.y ? DspSpace::y : DspSpace::x),
            "Native E1 reads the independently supplied word from its exact source space/address");
  }
  cells(machine, sample);
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

void form_guard(const Case sample, const unsigned word) {
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
        "Unqualified accumulator-load form must refuse before AGU/data effects");
    require(error.contains("Unqualified") && error.contains("pc=0x000100") &&
                error.contains("omr=0x000300") && run.faulted() && run.completed_cycles() == 2 &&
                machine.cpu().debug_state() == before,
            "Decode refusal retains complete source/destination/CCR/address state and context");
    cells(machine, sample);
  }
}

void mode_guard(const Case sample) {
  DspMachine machine;
  const auto before = prepare(machine, sample);
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  const auto saved = machine.checkpoint();
  constexpr unsigned frontend_excluded = 0x12000; // SC/FV reject at frontend ownership admission.
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(saved);
    std::optional<DspFetchRun> run;
    const auto error = checkpoint::rejects(
        [&] {
          run.emplace(contract(), machine);
          static_cast<void>(run->advance_cycle(machine, fetch_clock::input()));
        },
        "Unqualified status modes must refuse before any automatic cycle effect");
    const bool early = (sample.scale & frontend_excluded) != 0;
    if (early) {
      require(!run && error.contains("Program-fetch start requires an ordinary pending opcode"),
              "SC/FV retain the existing strict frontend constructor refusal");
    } else {
      require(run && run->faulted() && run->completed_cycles() == 0 &&
                  error.contains("automatic DSP mode/interrupt") && error.contains("sr="),
              "CE/SA retain the existing strict pre-cycle pipeline refusal");
    }
    require(
        error.contains("pc=0x000100") && error.contains("omr=0x000300") &&
            machine.cpu().debug_state() == before,
        "Every restored unqualified-mode suffix preserves the complete CPU and contextual reason");
    cells(machine, sample);
  }
}

void unknown_source(const Case sample) {
  DspMachine machine;
  const auto before = prepare(machine, sample, false);
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  DspFetchRun run(contract(), machine);
  for (unsigned index = 0; index < 5; ++index) {
    static_cast<void>(run.advance_cycle(machine, fetch_clock::input()));
  }
  auto expected = before;
  expected.cpu.r.at(sample.reg) = pointers.at(sample.mode);
  require(machine.cpu().debug_state() == expected,
          "Unknown RAM does not prevent the independently expected earlier AG2 update");
  const auto saved = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    const auto error = checkpoint::rejects(
        [&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); },
        "An entered E1 read of unknown RAM must fail visibly");
    const std::string space = sample.y ? "Y:" : "X:";
    require(error.contains("read " + space + "0x000407") && error.contains("pc=0x000100") &&
                error.contains("omr=0x000300") && run.faulted() && run.completed_cycles() == 5 &&
                machine.cpu().debug_state() == expected,
            "An unknown native E1 source quarantines without invented destination/CCR/retirement");
    const auto quarantined =
        checkpoint::rejects([&] { machine.cpu().step(); },
                            "A quarantined core must reject subsequent native execution");
    require(quarantined.contains("requires reset after a rejected reference step") &&
                machine.cpu().debug_state() == expected,
            "The public native step proves quarantine without touching the opaque fault flag");
  }
  run.restore(saved, machine);
  machine.bus().write(
      {.space = sample.y ? DspSpace::y : DspSpace::x, .address = data, .value = sample.value});
  period(run, machine, before, sample);
  period(run, machine, before, sample);
}

unsigned expanded() {
  unsigned count = 0;
  for (unsigned destination = 0; destination < words.size(); ++destination) {
    for (unsigned mode = 0; mode < 5; ++mode) {
      for (unsigned reg = 0; reg < 8; ++reg) {
        for (const bool y : {false, true}) {
          for (const auto value : {values[4], values[5]}) {
            automatic({.destination = destination,
                       .value = value,
                       .scale = scale_up,
                       .ccr = flags[3],
                       .y = y,
                       .mode = mode,
                       .reg = reg});
            ++count;
          }
        }
      }
      for (const bool y : {false, true}) {
        snapshots({.destination = destination,
                   .value = values[6],
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

void guards() {
  constexpr unsigned parallel_clear = 0x13;
  constexpr unsigned effective_address_bit = 0x4000;
  constexpr std::array unqualified_modes{0x2000U, 0x10000U, 0x20000U, 0x80000U};
  for (unsigned destination = 0; destination < words.size(); ++destination) {
    for (const bool y : {false, true}) {
      const Case sample{.destination = destination, .value = values[6], .ccr = flags[3], .y = y};
      for (unsigned mode = 5; mode <= 7; ++mode) {
        form_guard(sample, words[destination] | (y ? y_bit : 0U) | (mode << mode_shift));
      }
      form_guard(sample, opcode(sample) | parallel_clear);
      form_guard(sample, opcode(sample) & ~effective_address_bit);
      for (const auto status : unqualified_modes) {
        auto mode_sample = sample;
        mode_sample.scale = status;
        mode_guard(mode_sample);
      }
      unknown_source(sample);
    }
  }
}

unsigned matrix(const bool staged) {
  unsigned count = 0;
  for (unsigned destination = 0; destination < words.size(); ++destination) {
    for (const auto value : values) {
      for (const auto scale : {0U, scale_down, scale_up}) {
        for (const auto ccr : flags) {
          for (const bool y : {false, true}) {
            const Case sample{
                .destination = destination, .value = value, .scale = scale, .ccr = ccr, .y = y};
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

int main(const int argc, char **argv) {
  try {
    const std::string_view mode = argc == 2 ? argv[1] : "--native";
    if (argc > 2 || (mode != "--native" && mode != "--candidate" && mode != "--expanded")) {
      throw Fault("Usage: accumulator-load-test [--native|--candidate|--expanded]");
    }
    if (mode == "--expanded") {
      const auto count = expanded();
      guards();
      std::cout << count
                << " addresses,560 cuts/1120 suffixes,80 form/64 mode guards/two suffixes,"
                   "16 unknown-source recoveries/two failures plus one repaired suffix passed\n";
    } else if (mode == "--native" || mode == "--candidate") {
      std::cout << matrix(mode == "--candidate")
                << " independent accumulator-load phase/value cases passed\n";
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
