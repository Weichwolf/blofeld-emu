// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_sp_move.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
using namespace blofeld;
using test::DspProgram;
using test::require;
constexpr unsigned short_sp = 0x0500bb;
constexpr unsigned long_sp = 0x05f43b;
constexpr unsigned direct_sp = 0x04c0bb;
constexpr unsigned long_sr = 0x05f439;
constexpr unsigned long_ssh = 0x05f43c;
constexpr unsigned long_ssl = 0x05f43d;
constexpr unsigned status = 0x30d;
constexpr unsigned stack_high = 0x123400;
constexpr unsigned stack_low = 0x654300;
constexpr unsigned padding = 32;
constexpr unsigned budget = 48;
constexpr unsigned short_source = 0;
constexpr unsigned long_source = 1;
constexpr unsigned source_count = 14;
constexpr unsigned physical_entries = 16;
constexpr unsigned extal = 12000000;
constexpr unsigned timebase = 540000000;
constexpr std::array register_seeds{0x44f400U,
                                    0x45f400U,
                                    0x46f400U,
                                    0x47f400U,
                                    0x60f400U,
                                    0x61f400U,
                                    0x62f400U,
                                    0x63f400U,
                                    0x64f400U,
                                    0x65f400U,
                                    0x66f400U,
                                    0x67f400U};
unsigned suffixes = 0;
unsigned clones = 0;

struct Input {
  unsigned source, value;

  [[nodiscard]] unsigned words() const { return source == long_source ? 2U : 1U; }
};

DspFetchRunInputs contract() {
  return {.extal_hz = extal,
          .timebase_hz = timebase,
          .timing = DspFetchTiming::reference_cycle_order,
          .execution = DspReferenceExecution::staged_register_boundaries,
          .pipeline = DspFetchPipeline::automatic_memory};
}

unsigned prepare(DspMachine &machine, const Input input) {
  DspProgram code;
  code.word(long_sr);
  code.word(status);
  for (unsigned index = 1; index <= 2; ++index) {
    code.word(long_ssh);
    code.word(stack_high + index);
    code.word(long_ssl);
    code.word(stack_low + index);
  }
  unsigned seeds = 5;
  if (input.source >= 2) {
    code.word(register_seeds.at(input.source - 2));
    code.word(input.value);
    ++seeds;
  }
  const auto writer = code.pc();
  if (input.source == short_source) {
    code.word(short_sp | (input.value << 8U));
  } else if (input.source == long_source) {
    code.word(long_sp);
    code.word(input.value);
  } else {
    const auto field = input.source < 6 ? input.source + 2 : input.source + 10;
    code.word(direct_sp | (field << 8U));
  }
  for (unsigned index = 0; index < padding; ++index) { code.word(0); }
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  for (unsigned index = 0; index < seeds; ++index) { machine.cpu().step(); }
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == writer && before.cpu.sp == 2 && before.sc == 2 &&
              before.cpu.sr == status && before.stack_high[1] == stack_high + 1 &&
              before.stack_high[2] == stack_high + 2 && before.stack_low[1] == stack_low + 1 &&
              before.stack_low[2] == stack_low + 2,
          "Independent native MOVEC setup establishes a nonempty physical stack");
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return writer;
}

void decoder_checks() {
  for (unsigned field = 0; field < 64; ++field) {
    const bool expected = (field >= 4 && field <= 7) || (field >= 16 && field <= 23);
    const auto result = decode_sp_move(direct_sp | (field << 8U));
    require(result.has_value() == expected && (!result || result->words == 1),
            "FM12-20 control destination admits exactly the selected ordinary register sources");
  }
  for (unsigned value = 0; value < 256; ++value) {
    const auto result = decode_sp_move(short_sp | (value << 8U));
    require(result && result->words == 1, "Short MOVEC SP owns one opcode, not an extension");
  }
  const auto result = decode_sp_move(long_sp);
  require(result && result->words == 2, "Long MOVEC SP owns its actual extension");
  for (const unsigned opcode : {0U, long_sr, 0x05f03bU, 0x0444bbU, 0x04c4fbU, 0x1000000U}) {
    require(!decode_sp_move(opcode), "Memory, reverse, alias, other control and wide forms refuse");
  }
}

struct Seen {
  DspDebugState expected;
  bool e1 = false, writer = false, done = false;
};

void advance(
    DspMachine &machine, DspFetchRun &run, const Input input, const unsigned writer, Seen &seen) {
  const auto cycle =
      run.advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available});
  if (!cycle.pipeline) { throw Fault("Missing native SP pipeline cycle"); }
  const auto &phase = *cycle.pipeline;
  require(!phase.transfer, "SP never publishes a fictitious ordinary E1 register field");
  if (phase.execute1 && phase.execute1->address == writer) { seen.e1 = true; }
  if (phase.retirement_published) {
    if (!phase.execute2) { throw Fault("Missing native SP E2 identity"); }
    const auto pc = phase.execute2->address;
    const bool control = pc == writer;
    require(control ? !seen.writer : seen.writer && pc == writer + input.words(),
            "Writer and following NOP retire once in original order");
    seen.expected.cpu.pc += control ? input.words() : 1U;
    ++seen.expected.native_instructions;
    seen.expected.native_cycles += control ? input.words() : 1U;
    if (control) {
      require(seen.e1, "SP E2 follows the actual held native E1");
      seen.expected.cpu.sp = input.value;
      seen.expected.cpu.stack_pc = seen.expected.stack_high.at(input.value);
      seen.expected.cpu.stack_sr = seen.expected.stack_low.at(input.value);
      seen.writer = true;
    } else {
      seen.done = true;
    }
  }
  require(machine.cpu().debug_state() == seen.expected,
          "Literal per-period full CPU/SC/all16stack cells/CCR/counters;only real E2 changes SP");
}

void finish(
    DspMachine &machine, DspFetchRun &run, const Input input, const unsigned writer, Seen &seen) {
  for (unsigned index = 0; index < budget && !seen.done; ++index) {
    advance(machine, run, input, writer, seen);
  }
  require(seen.done, "Bounded SP control suffix reaches the independently expected NOP");
}

void one_case(const Input input) {
  DspMachine machine;
  const auto writer = prepare(machine, input);
  Seen seen{.expected = machine.cpu().debug_state()};
  DspFetchRun run(contract(), machine);
  for (unsigned index = 0; index < budget && !seen.e1; ++index) {
    advance(machine, run, input, writer, seen);
  }
  require(seen.e1 && !seen.writer, "Snapshot holds a genuine unretired SP result");
  const auto saved = run.checkpoint(machine);
  const auto prefix = seen;
  finish(machine, run, input, writer, seen);
  const auto endpoint = machine.cpu().debug_state();
  const auto ticks = run.elapsed_ticks();
  for (unsigned repeat = 0; repeat < 2; ++repeat) {
    run.restore(saved, machine);
    auto replay = prefix;
    finish(machine, run, input, writer, replay);
    require(machine.cpu().debug_state() == endpoint && run.elapsed_ticks() == ticks,
            "Trusted native replay preserves held SP and the complete endpoint");
    ++suffixes;
  }
  DspMachine target;
  DspFetchRun clone(saved, target);
  auto replay = prefix;
  finish(target, clone, input, writer, replay);
  require(target.cpu().debug_state() == endpoint && clone.elapsed_ticks() == ticks,
          "Fresh native owner retains the genuine pending SP result");
  ++clones;
}

void unsupported_status(const Input input) {
  DspMachine machine;
  static_cast<void>(prepare(machine, input));
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  std::optional<std::string> failure;
  try {
    for (unsigned index = 0; index < budget; ++index) {
      static_cast<void>(
          run.advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available}));
    }
  } catch (const Fault &error) { failure = error.what(); }
  require(failure && failure->contains("SP stack-status/extension/interrupt context") &&
              failure->contains("pc=0xff") && failure->contains("omr=0x000305") &&
              machine.cpu().debug_state() == before,
          "Unsupported status/extended pointer cannot silently write SP or consume native state");
}
}

int main() {
  try {
    decoder_checks();
    for (unsigned source = 0; source < source_count; ++source) {
      for (unsigned value = 0; value < physical_entries; ++value) {
        one_case({.source = source, .value = value});
      }
    }
    for (const unsigned value : {16U, 31U, 32U, 255U, 256U, 0xffffffU}) {
      unsupported_status({.source = long_source, .value = value});
      unsupported_status({.source = 2, .value = value});
      if (value < 256) { unsupported_status({.source = short_source, .value = value}); }
    }
    std::cout << "SP controls=224 suffixes=" << suffixes << " clones=" << clones
              << ";full per-period CPU/SC/stack/CCR;unsupported-status guards=16\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
