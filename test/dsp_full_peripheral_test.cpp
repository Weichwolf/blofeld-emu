// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_pipeline.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_full_peripheral_store.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_memory_form.hpp"
#include "dsp/dsp_pipeline.hpp"
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

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned seed_count = 7, body = boot + 14, padding = 24;
constexpr unsigned compare = 0xffff8d, canary = 0x654321;
constexpr unsigned b_field = 0x100, source_shift = 24, extension_shift = 48;
constexpr unsigned clear_flags = 0x14, byte_mask = 0xff;
constexpr unsigned once_a_store = 0x08ce3c, compare_a_store = 0x04ce8d;
constexpr std::uint64_t clipped = 0x00800000000001;
constexpr std::array clear{0x200013U, 0x20001bU};
constexpr std::array<std::uint64_t, 18> values{0,
                                               1,
                                               0x00000000ffffff,
                                               0x00123456789abc,
                                               0x003fffffabcdef,
                                               0x00400000000000,
                                               0x007fffff000000,
                                               0x00800000000001,
                                               0x00ffffffabcdef,
                                               0x55800001abcdef,
                                               0x7fffffffffffff,
                                               0x80000000000000,
                                               0xfe7fffff000000,
                                               0xff7fffffabcdef,
                                               0xff800000000000,
                                               0xffbfffffffffff,
                                               0xffc00000000000,
                                               0xffffffffffffff};

struct Input {
  std::uint64_t value = 0;
  unsigned scale = 0, flags = 0, gap = 0;
  bool b = false, timer = false, producer = false, same = false;
};

unsigned opcode(const Input input) {
  // FM13-136/Table12-13: full A/B fields14/15; pp=60 or split qq=13.
  return (input.timer ? compare_a_store : once_a_store) | (input.b ? b_field : 0U);
}

unsigned store_pc(const Input input) {
  return body + 1 + static_cast<unsigned>(input.producer) + input.gap;
}

unsigned stall(const Input input) {
  return input.producer && input.same && input.gap == 0 ? 1U : 0U;
}

unsigned e1(const Input input) {
  return 7 + static_cast<unsigned>(input.producer) + input.gap + stall(input);
}

DspFetchRunInputs contract() {
  auto result = pipeline::contract();
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

DspDebugState prepare(DspMachine &machine, const Input input) {
  DspProgram code;
  for (const auto *name : {"a", "b"}) {
    code.immediate(std::string(name) + '0', input.value & word_mask);
    code.immediate(std::string(name) + '1', (input.value >> source_shift) & word_mask);
    code.immediate(std::string(name) + '2', (input.value >> extension_shift) & byte_mask);
  }
  code.immediate("sr", masked_status | input.scale | input.flags);
  code.word(0);
  if (input.producer) {
    code.word(clear.at(static_cast<unsigned>(input.same ? input.b : !input.b)));
  }
  for (unsigned gap = 0; gap < input.gap; ++gap) { code.word(0); }
  code.word(opcode(input));
  for (unsigned i = 0; i < padding; ++i) { code.word(0); }
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  machine.bus().write({.space = DspSpace::x, .address = compare, .value = canary});
  checkpoint::run(machine, seed_count);
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == body && before.cpu.a == input.value && before.cpu.b == input.value &&
              before.cpu.sr == (masked_status | input.scale | input.flags),
          "Native immediate seeds establish the full independently selected accumulator state");
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  return before;
}

struct Oracle {
  DspDebugState state;
  unsigned stores = 0, stalls = 0;
};

void check_port(DspMachine &machine,
                const Input input,
                const unsigned period,
                const unsigned value) {
  const auto latch = machine.bus().once_latch();
  if (input.timer) {
    require(!latch && machine.bus().read({.space = DspSpace::x, .address = compare}) ==
                          (period >= e1(input) ? value : canary),
            "The real low-X timer compare retains the independently limited full-source value");
  } else {
    require(period >= e1(input) ? latch == value : !latch,
            "The real OnCE latch retains the independently limited full-source value at E1");
  }
}

void step(DspFetchRun &run, DspMachine &machine, const Input input, Oracle &oracle) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(cycle);
  const auto period = static_cast<unsigned>(cycle.completed_cycles);
  const auto source = input.producer && input.same ? 0U : input.value;
  const unsigned flags = input.producer ? clear_flags : input.flags;
  const auto stored =
      scalar_store::expected({.value = source, .scale = input.scale, .flags = flags});
  const unsigned index = period >= 7 ? period - 7 : 0;
  const unsigned store_index = store_pc(input) - body;
  const bool retired = period >= 7 && (stall(input) == 0 || index != store_index);
  require(phase.retirement_published == retired,
          "Literal one-word retirements include exactly one source interlock");
  if (input.producer && period == 8) {
    const bool producer_b = input.same ? input.b : !input.b;
    (producer_b ? oracle.state.cpu.b : oracle.state.cpu.a) = 0;
  }
  if (phase.memory_source_stall) {
    ++oracle.stalls;
    require(stall(input) == 1 && period == e1(input) - 1 && !phase.memory,
            "Only an adjacent same-accumulator producer stalls the actual E1 source");
  }
  require(phase.memory.has_value() == (period == e1(input)),
          "The native peripheral transaction happens once at the independently scheduled E1");
  if (phase.memory) { ++oracle.stores; }
  check_port(machine, input, period, stored.value);
  if (retired) {
    require(phase.execute2 && phase.execute2->address == oracle.state.cpu.pc,
            "Native E2 preserves exact instruction ownership and ordering");
    if (input.producer && oracle.state.cpu.pc == body + 1) { oracle.state.cpu.sr |= clear_flags; }
    if (oracle.state.cpu.pc == store_pc(input)) {
      oracle.state.cpu.sr = masked_status | input.scale | stored.flags;
    }
    ++oracle.state.cpu.pc;
    ++oracle.state.native_instructions;
    ++oracle.state.native_cycles;
  }
  if (machine.cpu().debug_state() != oracle.state) {
    std::cerr << "period=" << period << " opcode=" << hex(opcode(input), 6)
              << " value=" << hex(input.value, 14) << " scale=" << input.scale
              << " actualSR=" << hex(machine.cpu().state().sr, 6)
              << " expectedSR=" << hex(oracle.state.cpu.sr, 6) << '\n';
  }
  require(machine.cpu().debug_state() == oracle.state,
          "Every period matches complete independent CPU/flags/physical-stack/counter state");
}

void finish(DspFetchRun &run, DspMachine &machine, const Input input, Oracle &oracle) {
  while (run.completed_cycles() < e1(input) + 3) { step(run, machine, input, oracle); }
  require(oracle.stores == 1 && oracle.stalls == stall(input),
          "The complete stream has one native write and its exact source-stall count");
}

void one(const Input input) {
  DspMachine machine;
  Oracle oracle{.state = prepare(machine, input)};
  DspFetchRun run(contract(), machine);
  finish(run, machine, input, oracle);
}

void restores(const Input input) {
  for (unsigned cut = 5; cut <= e1(input) + 1; ++cut) {
    DspMachine machine;
    Oracle oracle{.state = prepare(machine, input)};
    DspFetchRun run(contract(), machine);
    while (run.completed_cycles() < cut) { step(run, machine, input, oracle); }
    const auto saved = run.checkpoint(machine);
    const auto prefix = oracle;
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      oracle = prefix;
      finish(run, machine, input, oracle);
    }
    DspMachine clone;
    DspFetchRun continuation(saved, clone);
    oracle = prefix;
    finish(continuation, clone, input, oracle);
  }
}

void decode_field(const bool b, const unsigned field) {
  const unsigned part = b ? b_field : 0U;
  constexpr unsigned low_mask = 31;
  constexpr unsigned split_bit = 32;
  constexpr unsigned high_x = 0x08ce00;
  constexpr unsigned high_y = 0x09ce00;
  constexpr unsigned low_x = 0x04ce80;
  constexpr unsigned low_y = 0x04ce20;
  constexpr unsigned high_base = 0xffffc0;
  constexpr unsigned low_base = 0xffff80;
  constexpr unsigned overwide = 0x1000000;
  const unsigned split = (field & low_mask) | ((field & split_bit) << 1U);
  const std::array words{
      high_x | part | field, high_y | part | field, low_x | part | split, low_y | part | split};
  for (unsigned port = 0; port < words.size(); ++port) {
    const auto word = words.at(port);
    const auto form = decode_memory_stage(word);
    const unsigned base = port < 2 ? high_base : low_base;
    require(full_accumulator_peripheral_store(word) && form && form->peripheral_register &&
                form->peripheral && form->peripheral->address == base + field &&
                form->peripheral->space == (port % 2 == 0 ? DspSpace::x : DspSpace::y),
            "Every literal full A/B pp/qq field decodes its independent space and address");
    for (unsigned producer = 0; producer < clear.size(); ++producer) {
      const unsigned mask = b ? 2U : 1U;
      const unsigned expected = producer == static_cast<unsigned>(b) ? mask : 0U;
      require(memory_source_dependency({.producer = {.value = clear.at(producer)},
                                        .consumer = {.value = word}}) == expected,
              "Full-source dependency covers the entire accumulator and no opposite source");
    }
    require(!full_accumulator_peripheral_store(word | overwide),
            "Overwide words cannot alias full sources");
  }
}

void decode_checks() {
  constexpr unsigned fields = 64;
  for (const bool b : {false, true}) {
    for (unsigned field = 0; field < fields; ++field) { decode_field(b, field); }
  }
}

}

int main() {
  try {
    decode_checks();
    unsigned streams = 0;
    for (const bool b : {false, true}) {
      for (const bool timer : {false, true}) {
        for (const auto value : values) {
          for (const unsigned scale : {0U, scale_down, scale_up}) {
            for (const unsigned flags : {0U, 0xffU}) {
              one({.value = value, .scale = scale, .flags = flags, .b = b, .timer = timer});
              ++streams;
            }
          }
        }
        restores({.value = clipped, .scale = scale_up, .b = b, .timer = timer});
        for (const bool same : {false, true}) {
          for (unsigned gap = 0; gap <= 2; ++gap) {
            const Input input{.value = 0x55800001abcdef,
                              .gap = gap,
                              .b = b,
                              .timer = timer,
                              .producer = true,
                              .same = same};
            one(input);
            restores(input);
            ++streams;
          }
        }
      }
    }
    std::cout << "Qualified full-source MOVEP streams=" << streams
              << ";512 decode fields;trusted suffixes and fresh clones\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
