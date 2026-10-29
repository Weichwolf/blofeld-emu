// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp_address_update_fixture.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <cstdint>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace blofeld::test::address_update {
namespace {
void append(std::vector<std::uint32_t> &words, const std::string &line) {
  const auto assembled = checkpoint::assemble(line.c_str());
  words.insert(words.end(), assembled.begin(), assembled.end());
}

struct Dependency {
  unsigned reg = 0;
  char family = 'r';
  unsigned gap = 0;
  bool extended = false;
};

unsigned final_pointer(const char family) {
  constexpr unsigned pointer_written_result = 18;
  constexpr unsigned offset_written_result = 19;
  if (family == 'r') { return pointer_written_result; }
  if (family == 'n') { return offset_written_result; }
  return 1;
}

void replay_dependency(const std::optional<DspFetchCheckpoint> &saved,
                       DspFetchRun &run,
                       DspMachine &machine,
                       const DspDebugState &expected,
                       const unsigned cycles) {
  if (!saved) { return; }
  for (unsigned repeat = 0; repeat < 2; ++repeat) {
    run.restore(*saved, machine);
    while (run.completed_cycles() < cycles) {
      static_cast<void>(run.advance_cycle(machine, fetch_clock::input()));
    }
    require(machine.cpu().debug_state() == expected,
            "Each restored dependency suffix reaches the independent full native expectation");
  }
}

void dependency(const Dependency sample) {
  const auto reg = sample.reg;
  const auto family = sample.family;
  const auto gap = sample.gap;
  const auto extended = sample.extended;
  constexpr Case input{.r = 14, .n = 3, .m = word_mask, .mode = 1};
  DspMachine machine;
  const auto before = prepare(machine, input, reg);
  const auto value = family == 'n' ? 5U : 15U;
  std::vector<std::uint32_t> words;
  append(words,
         "move #" + std::string(extended ? ">" : "<") + number(value) + ',' + family +
             std::to_string(reg));
  const auto length = static_cast<unsigned>(words.size());
  require(length == (extended ? 2U : 1U), "The producer retains its independent encoded length");
  words.insert(words.end(), gap, 0);
  const auto update_pc = entry + static_cast<unsigned>(words.size());
  words.push_back(opcode(input, reg));
  words.insert(words.end(), padding, 0);
  checkpoint::ram(machine.bus(), entry, words);
  const auto settling = extended ? 2U : 3U;
  const auto stalls = gap < settling ? settling - gap : 0U;
  const auto cycles = 13 + length + gap + stalls;
  const auto pointer = final_pointer(family);
  auto expected = before;
  expected.cpu.r.at(reg) = pointer;
  if (family == 'n') { expected.cpu.n.at(reg) = value; }
  if (family == 'm') { expected.cpu.m.at(reg) = value; }
  expected.cpu.pc += length + gap + 7;
  expected.native_instructions += gap + 8;
  expected.native_cycles += length + gap + 7;
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  DspFetchRun run(contract(), machine);
  unsigned holds = 0;
  bool captured = false;
  std::optional<DspFetchCheckpoint> saved;
  while (run.completed_cycles() < cycles) {
    const auto cycle = run.advance_cycle(machine, fetch_clock::input());
    const auto &phase = pipeline::stages(cycle);
    if (phase.agu_dependencies != 0) {
      ++holds;
      require(phase.decode && phase.decode->address == update_pc &&
                  phase.agu_dependencies == (1U << reg) && !phase.decode_accepted,
              "The real same-triplet producer holds U Decode by its exact register mask");
    }
    if (phase.ag1_address && phase.ag1_address->word.address == update_pc) {
      const auto &address = *phase.ag1_address;
      captured = true;
      require(address.update.before == (family == 'r' ? value : input.r) &&
                  address.update.value == pointer && address.data_access_count == 0,
              "U AG1 reads the now-ready native R/N/M cache without an invented data operand");
      if (gap == 0) { saved = run.checkpoint(machine); }
    }
  }
  require(holds == stalls && captured && machine.cpu().debug_state() == expected &&
              run.elapsed_ticks() == cycles * pipeline::period,
          "Independent short/long producer timing and complete final CPU state agree");
  replay_dependency(saved, run, machine, expected, cycles);
}

struct Chain {
  unsigned reg = 0, mode = 0;
};

void chain(const Chain sample) {
  const auto reg = sample.reg;
  const auto mode = sample.mode;
  constexpr unsigned base = 0x400;
  const Case input{.r = base + 15, .n = 3, .m = 15, .mode = mode};
  DspMachine machine;
  const auto before = prepare(machine, input, reg);
  constexpr std::array<int, 4> first_delta{-3, 3, -1, 1};
  const auto first = base + static_cast<unsigned>((15 + first_delta.at(mode)) % 16);
  const auto second = base + ((first - base + 3) % 16);
  const auto third = base + ((second - base + 3) % 16);
  const Case add_offset{.mode = 1};
  const std::array words{opcode(input, reg), opcode(add_offset, reg), opcode(add_offset, reg)};
  checkpoint::ram(machine.bus(), entry, words);
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  DspFetchRun run(contract(), machine);
  unsigned updates = 0;
  constexpr unsigned cycles = 15;
  const std::array targets{first, second, third};
  for (unsigned count = 0; count < cycles; ++count) {
    const auto cycle = run.advance_cycle(machine, fetch_clock::input());
    const auto &phase = pipeline::stages(cycle);
    require(phase.agu_dependencies == 0, "Implicit U updates do not rearm explicit MOVE holds");
    if (phase.ag1_address) {
      const auto &address = *phase.ag1_address;
      require(updates < targets.size() && address.update.value == targets.at(updates),
              "Each adjacent U reads its preceding actual AG2 result exactly once");
      ++updates;
    }
  }
  auto expected = before;
  expected.cpu.r.at(reg) = third;
  expected.cpu.pc += 9;
  expected.native_instructions += 9;
  expected.native_cycles += 9;
  require(updates == 3 && machine.cpu().debug_state() == expected,
          "Three adjacent native U updates preserve all other CPU fields and nominal T=1");
}
}

void dependency_checks() {
  for (unsigned reg = 0; reg < 8; ++reg) {
    for (const auto family : {'r', 'n', 'm'}) {
      for (const bool extended : {false, true}) {
        for (unsigned gap = 0; gap < 4; ++gap) {
          dependency({.reg = reg, .family = family, .gap = gap, .extended = extended});
        }
      }
    }
    for (unsigned mode = 0; mode < 4; ++mode) { chain({.reg = reg, .mode = mode}); }
  }
  std::cout << "192 U producer cases,48 cuts/96 suffixes and32 implicit-update chains passed\n";
}
}
