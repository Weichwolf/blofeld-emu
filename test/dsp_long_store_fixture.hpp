// SPDX-License-Identifier: GPL-3.0-only
#pragma once
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
#include <iostream>
#include <string>

namespace blofeld::test::long_saturation {
inline constexpr unsigned entry = 0x100, data = 0x400, old_lc = 0x33, canary = 0x654321;
inline constexpr unsigned ordinary_omr = 0x300, absolute_jump = 0x0af080, long_a = 0x484000;
inline constexpr unsigned long_b_bit = 0x10000, mode_shift = 11, register_shift = 8;
inline constexpr std::uint64_t sign56 = 0x80000000000000, span56 = 0x100000000000000;
inline constexpr std::uint64_t mask48 = 0xffffffffffff;
inline constexpr std::int64_t maximum48 = 140737488355327, minimum48 = -140737488355328;
inline constexpr auto execution = DspReferenceExecution::staged_register_boundaries;

struct Case {
  std::uint64_t value;
  unsigned scale = 0, flags = 0, mode = 0, reg = 0;
  bool b = false;
};

struct Value {
  unsigned x, y, flags;
};

inline Value expected(const Case sample) {
  // FM13-126/127: A/B are one signed 48-bit double-precision quantity.
  // Derive signed/scaled values using integer arithmetic and division, independently
  // of the interpreter's aligned register representation and shift implementation.
  auto value = sample.value < sign56 ? static_cast<std::int64_t>(sample.value)
                                     : -static_cast<std::int64_t>(span56 - sample.value);
  constexpr unsigned normal_growth_bit = 46;
  unsigned growth = normal_growth_bit;
  if (sample.scale == scale_up) {
    value *= 2;
    --growth;
  } else if (sample.scale == scale_down) {
    value = value >= 0 ? value / 2 : -((-value + 1) / 2);
    ++growth;
  }
  auto flags = sample.flags;
  if (((sample.value >> growth) & 1U) != ((sample.value >> (growth - 1)) & 1U)) { flags |= sticky; }
  if (value > maximum48) {
    value = maximum48;
    flags |= limit;
  } else if (value < minimum48) {
    value = minimum48;
    flags |= limit;
  }
  const auto bits = static_cast<std::uint64_t>(value) & mask48;
  return {.x = static_cast<unsigned>(bits >> word_bits),
          .y = static_cast<unsigned>(bits & word_mask),
          .flags = flags};
}

inline unsigned pointer(const unsigned mode) {
  switch (mode) {
    case 0: return data - 3;
    case 1: return data + 3;
    case 2: return data - 1;
    case 3: return data + 1;
    case 4: return data;
    default: throw Fault("Undefined own long-store address mode");
  }
}

inline void prepare(DspMachine &machine, const Case sample) {
  DspProgram program;
  program.immediate("omr", ordinary_omr);
  program.immediate("lc", old_lc);
  program.immediate("r" + std::to_string(sample.reg), data);
  program.immediate("n" + std::to_string(sample.reg), 3);
  const std::string reg = sample.b ? "b" : "a";
  for (unsigned field = 0; field < 3; ++field) {
    const auto value = (sample.value >> (field * word_bits)) & (field == 2 ? 0xffU : word_mask);
    program.immediate(reg + std::to_string(field), value);
  }
  program.immediate("sr", masked_status | sample.scale | sample.flags);
  for (unsigned index = 0; index < 5; ++index) { program.word(0); }
  program.word(absolute_jump);
  program.word(entry);
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::until(machine, entry);
  for (unsigned address = entry; address < entry + 16; ++address) {
    machine.bus().write({.address = address, .value = 0});
  }
  const auto opcode = long_a | (sample.b ? long_b_bit : 0U) | (sample.mode << mode_shift) |
                      (sample.reg << register_shift);
  machine.bus().write({.address = entry, .value = opcode});
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const unsigned address : {data - 1, data, data + 1}) {
      machine.bus().write({.space = space, .address = address, .value = canary});
    }
  }
  static_cast<void>(machine.begin_reference_instruction(execution));
}

inline void one(const Case sample) {
  DspMachine machine;
  prepare(machine, sample);
  const auto value = expected(sample);
  const auto before = machine.cpu().debug_state();
  auto contract = fetch_clock::contract();
  contract.execution = execution;
  contract.pipeline = DspFetchPipeline::automatic_memory;
  DspFetchRun run(contract, machine);
  for (unsigned period = 1; period <= 7; ++period) {
    const auto cycle = run.advance_cycle(machine, fetch_clock::input());
    auto wanted = before;
    if (period >= 5) { wanted.cpu.r.at(sample.reg) = pointer(sample.mode); }
    if (period == 7) {
      wanted.cpu.pc = entry + 1;
      wanted.cpu.sr = masked_status | sample.scale | value.flags;
      ++wanted.native_instructions;
      ++wanted.native_cycles;
    }
    const auto actual = machine.cpu().debug_state();
    if (actual != wanted) {
      std::cerr << "period=" << period << " source=" << sample.value << " scale=" << sample.scale
                << " B=" << sample.b << " mode=" << sample.mode << " r=" << sample.reg
                << " sr actual/expected=" << actual.cpu.sr << '/' << wanted.cpu.sr << '\n';
    }
    require(actual == wanted, "Full accumulator store must publish only genuine AG2/E2 effects");
    require(machine.bus().read({.space = DspSpace::x, .address = data}) ==
                    (period >= 6 ? value.x : canary) &&
                machine.bus().read({.space = DspSpace::y, .address = data}) ==
                    (period >= 6 ? value.y : canary),
            "Native X/Y data must match independent signed 48-bit scaling/clipping");
    require(cycle.pipeline && cycle.pipeline->memory.has_value() == (period == 6),
            "A full accumulator pair must execute once at actual native memory E1");
  }
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const unsigned address : {data - 1, data + 1}) {
      require(machine.bus().read({.space = space, .address = address}) == canary,
              "A native full accumulator transfer must preserve both neighboring cells");
    }
  }
}

inline constexpr std::array<std::uint64_t, 14> values{0,
                                                      1,
                                                      0x007fffffffffff,
                                                      0x00800000000000,
                                                      0x00ffffffffffff,
                                                      0xff800000000000,
                                                      0xff7fffffffffff,
                                                      0xffffffffffffff,
                                                      0x00200000000000,
                                                      0x00400000000000,
                                                      0x00600000000000,
                                                      0x00800000000001,
                                                      0x7fffffffffffff,
                                                      0x80000000000000};

}
