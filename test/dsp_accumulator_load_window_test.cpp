// SPDX-License-Identifier: GPL-3.0-only
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
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned seed_instructions = 13, seed_cycles = 26, body = boot + seed_cycles;
constexpr unsigned producer_pc = body + 1, data = 0x400, source = 0x307;
constexpr unsigned canary = 0x654321, input_value = 0x812345, ordinary_omr = 0x300;
constexpr unsigned modulo = 0xff0007, wrapped_source = 0x300, flags = 0xff;
constexpr unsigned b_bit = 0x10000, y_bit = 0x80000, producer_address = 0x1c00;
constexpr unsigned extension_mask = 0xff, extension_sign = 0x80, extension_fill = 0xffff00;
constexpr std::uint64_t initial = 0x80800000abcdef, accumulator_mask = 0xffffffffffffff;
constexpr std::uint64_t negative_extension = 0xff000000000000;
constexpr std::array load_words{0x50c000U, 0x54c000U, 0x52c000U, 0x56c000U};
constexpr std::array store_words{0x505800U, 0x545800U, 0x525800U};

struct Input {
  bool consumer_b = false, input_y = false, output_y = false, same = true;
  unsigned gap = 0, load_part = 0, store_part = 0;
};

bool producer_b(const Input input) {
  return input.same ? input.consumer_b : !input.consumer_b;
}

unsigned stall(const Input input) {
  // FM3.5.2.1/Figure3-13 explicitly includes load A1 -> store A2.
  return input.same && input.gap == 0 ? 1U : 0U;
}

unsigned consumer_pc(const Input input) {
  return body + 2 + input.gap;
}

unsigned store_e1(const Input input) {
  return 8 + input.gap + stall(input);
}

unsigned last(const Input input) {
  return 12 + input.gap + stall(input);
}

std::uint64_t loaded(const Input input) {
  if (input.load_part == 3) {
    return negative_extension | (static_cast<std::uint64_t>(input_value) << word_bits);
  }
  const auto shift = input.load_part * word_bits;
  const auto mask = static_cast<std::uint64_t>(input.load_part == 2 ? extension_mask : word_mask)
                    << shift;
  return (initial & (accumulator_mask ^ mask)) |
         ((static_cast<std::uint64_t>(input_value) << shift) & mask);
}

unsigned stored(const Input input) {
  const auto value = input.same ? loaded(input) : initial;
  auto part = static_cast<unsigned>((value >> (input.store_part * word_bits)) &
                                    (input.store_part == 2 ? extension_mask : word_mask));
  if (input.store_part == 2 && (part & extension_sign) != 0) { part |= extension_fill; }
  return part;
}

DspFetchRunInputs contract() {
  auto result = pipeline::contract();
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

DspDebugState prepare(DspMachine &machine, const Input input) {
  DspProgram program;
  for (const bool b : {false, true}) {
    for (unsigned part = 0; part < 3; ++part) {
      program.immediate(std::string(b ? "b" : "a") + std::to_string(part),
                        (initial >> (part * word_bits)) & (part == 2 ? extension_mask : word_mask));
    }
  }
  program.immediate("omr", ordinary_omr);
  program.immediate("r0", data);
  program.immediate("n0", 3);
  program.immediate("r4", source);
  program.immediate("n4", 3);
  program.immediate("m4", modulo);
  program.immediate("sr", masked_status | flags);
  program.word(0);
  program.word(load_words.at(input.load_part) | producer_address |
               (producer_b(input) ? b_bit : 0U) | (input.input_y ? y_bit : 0U));
  for (unsigned gap = 0; gap < input.gap; ++gap) { program.word(0); }
  program.word(store_words.at(input.store_part) | (input.consumer_b ? b_bit : 0U) |
               (input.output_y ? y_bit : 0U));
  for (unsigned index = 0; index < 32; ++index) { program.word(0); }
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::run(machine, seed_instructions);
  const auto before = machine.cpu().debug_state();
  require(
      before.cpu.pc == body && before.cpu.a == initial && before.cpu.b == initial &&
          before.cpu.sr == (masked_status | flags) && before.cpu.omr == ordinary_omr &&
          before.cpu.r[0] == data && before.cpu.m[0] == word_mask && before.cpu.r[4] == source &&
          before.cpu.m[4] == modulo && before.cpu.n[4] == 3 &&
          before.native_instructions == seed_instructions && before.native_cycles == seed_cycles,
      "Actual own instructions establish exact accumulator/address/mode fields and native cost");
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    machine.bus().write({.space = space,
                         .address = source,
                         .value = (space == DspSpace::y) == input.input_y ? input_value : canary});
    for (const auto address : {data - 1, data, data + 1}) {
      machine.bus().write({.space = space, .address = address, .value = canary});
    }
  }
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return before;
}

struct Oracle {
  DspDebugState expected;
  unsigned reads = 0, stores = 0, stalls = 0, retired = 0;
};

unsigned retirement_count(const Input input, const unsigned period) {
  unsigned count = 0;
  for (unsigned index = 0; index < 6 + input.gap; ++index) {
    const auto ready = 7 + index + (index >= 2 + input.gap ? stall(input) : 0U);
    if (period >= ready) { ++count; }
  }
  return count;
}

void memory(DspMachine &machine, const Input input, const unsigned period) {
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    require(machine.bus().read({.space = space, .address = source}) ==
                ((space == DspSpace::y) == input.input_y ? input_value : canary),
            "Native producer reads preserve source and opposite-space sentinel");
    for (const auto address : {data - 1, data, data + 1}) {
      const bool selected =
          period >= store_e1(input) && address == data && (space == DspSpace::y) == input.output_y;
      require(machine.bus().read({.space = space, .address = address}) ==
                  (selected ? stored(input) : canary),
              "Actual memory retains independent raw/sign-extended parts and every neighbor");
    }
  }
}

void receipt(const DspPipelineCycle &phase,
             const Input input,
             const unsigned period,
             Oracle &oracle) {
  if (!phase.memory) { return; }
  const auto &access = phase.memory->data;
  require(!phase.memory->additional_data, "Standalone windows retain one data effect per E1");
  if (access.kind == DspNativeAccessKind::read) {
    ++oracle.reads;
    require(period == 7 && phase.execute1 && phase.execute1->address == producer_pc &&
                access.target.space == (input.input_y ? DspSpace::y : DspSpace::x) &&
                access.target.address == source && access.value == input_value,
            "The once-only native producer E1 reads its real independent source word");
  } else {
    ++oracle.stores;
    require(access.kind == DspNativeAccessKind::write && period == store_e1(input) &&
                phase.execute1 && phase.execute1->address == consumer_pc(input) &&
                access.target.space == (input.output_y ? DspSpace::y : DspSpace::x) &&
                access.target.address == data && access.value == stored(input),
            "The native consumer samples its independently expected post-load source once");
  }
}

void step(DspFetchRun &run, DspMachine &machine, const Input input, Oracle &oracle) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(cycle);
  const auto period = static_cast<unsigned>(cycle.completed_cycles);
  require(phase.retirement_published ==
              (retirement_count(input, period) != retirement_count(input, period - 1)),
          "Independent one-word costs and the manual transfer hold determine every E2");
  if (period == 8) {
    require(phase.memory_source_stall == (stall(input) != 0),
            "FM3.5.2.1 requires exactly one adjacent same-accumulator transfer hold");
  }
  if (phase.memory_source_stall) {
    ++oracle.stalls;
    require(period == 8 && phase.execute1 && phase.execute1->address == consumer_pc(input) &&
                !phase.memory && !phase.execution_captured,
            "The held actual consumer E1 waits while its preceding load retires");
  }
  if (period == 6) { oracle.expected.cpu.r[4] = wrapped_source; }
  if (period == 7 + input.gap) { oracle.expected.cpu.r[0] = data + 1; }
  if (period == 7) {
    auto &target = producer_b(input) ? oracle.expected.cpu.b : oracle.expected.cpu.a;
    target = loaded(input);
  }
  receipt(phase, input, period, oracle);
  memory(machine, input, period);
  if (phase.retirement_published) {
    require(phase.execute2 && phase.execute2->address == oracle.expected.cpu.pc,
            "Native retirement retains the actual anchor/producer/consumer/NOP sequence");
    ++oracle.expected.cpu.pc;
    ++oracle.expected.native_instructions;
    ++oracle.expected.native_cycles;
    ++oracle.retired;
  }
  require(machine.cpu().debug_state() == oracle.expected,
          "Every period retains complete independent CPU/CCR, younger AG2 and native counts");
}

void finish(DspFetchRun &run, DspMachine &machine, const Input input, Oracle &oracle) {
  while (run.completed_cycles() < last(input)) { step(run, machine, input, oracle); }
  require(oracle.retired == 6 + input.gap && oracle.reads == 1 && oracle.stores == 1 &&
              oracle.stalls == stall(input) &&
              run.elapsed_ticks() == last(input) * pipeline::period,
          "The complete manual window has exactly its declared load/store/retire/hold effects");
}

void one(const Input input) {
  DspMachine machine;
  Oracle oracle{.expected = prepare(machine, input)};
  DspFetchRun run(contract(), machine);
  finish(run, machine, input, oracle);
}

void snapshots(const Input input) {
  for (unsigned cut = 6; cut <= 11; ++cut) {
    DspMachine machine;
    Oracle oracle{.expected = prepare(machine, input)};
    DspFetchRun run(contract(), machine);
    while (run.completed_cycles() < cut) { step(run, machine, input, oracle); }
    const auto saved = run.checkpoint(machine);
    const auto before = oracle;
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      oracle = before;
      finish(run, machine, input, oracle);
    }
  }
}

void windows(const Input sample) {
  for (const bool same : {false, true}) {
    for (unsigned gap = 0; gap <= 1; ++gap) {
      auto input = sample;
      input.same = same;
      input.gap = gap;
      one(input);
    }
  }
  snapshots(sample);
}
}

int main() {
  try {
    for (const bool consumer_b : {false, true}) {
      for (const bool input_y : {false, true}) {
        for (const bool output_y : {false, true}) {
          for (unsigned load_part = 0; load_part < 4; ++load_part) {
            for (unsigned store_part = 0; store_part < 3; ++store_part) {
              windows({.consumer_b = consumer_b,
                       .input_y = input_y,
                       .output_y = output_y,
                       .load_part = load_part,
                       .store_part = store_part});
            }
          }
        }
      }
    }
    std::cout << "384 native load-to-part-store windows,576 cuts/1152 suffixes passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
