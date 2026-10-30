// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_full_peripheral_store.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_memory_form.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_repeat_fetch.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"
#include "dsp_scalar_store_fixture.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned entry = 0x100, jump = 0x0af080, once_store = 0x08ce3c;
constexpr unsigned reserved_x = 0x04ce80, unknown_y = 0x09ce3c, repeat3 = 0x0603a0;
constexpr unsigned raw_a0 = 0x08c83c, raw_a2 = 0x08ca3c, reserved_scale = 0xc00;
constexpr unsigned unsupported_modes = 0xb2000, padding = 16;
constexpr std::uint64_t source = 0x55800001abcdef;
constexpr unsigned middle_shift = 24, extension_shift = 48, byte_mask = 0xff;

struct Input {
  unsigned opcode = once_store, status = 0;
  bool repeat = false;
};

DspFetchRunInputs contract() {
  auto result = pipeline::contract();
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

DspDebugState prepare(DspMachine &machine, const Input input) {
  DspProgram code;
  code.immediate("a0", source & word_mask);
  code.immediate("a1", (source >> middle_shift) & word_mask);
  code.immediate("a2", (source >> extension_shift) & byte_mask);
  code.immediate("sr", masked_status | input.status);
  code.word(jump);
  code.word(entry);
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  for (unsigned index = 0; index < padding; ++index) {
    machine.bus().write({.address = entry + index, .value = 0});
  }
  machine.bus().write({.address = entry, .value = input.repeat ? repeat3 : input.opcode});
  if (input.repeat) { machine.bus().write({.address = entry + 1, .value = input.opcode}); }
  checkpoint::until(machine, entry);
  const auto before = machine.cpu().debug_state();
  require(before.cpu.a == source && before.cpu.sr == (masked_status | input.status),
          "Only native seeds and native JMP establish the tested RAM stream");
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  return before;
}

void refuses(const Input input, const std::string &diagnostic) {
  DspMachine machine;
  const auto before = prepare(machine, input);
  DspFetchRun run(contract(), machine);
  const auto saved = run.checkpoint(machine);
  std::string first;
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    const auto error = checkpoint::rejects(
        [&] {
          for (unsigned period = 0; period < padding; ++period) {
            static_cast<void>(run.advance_cycle(machine, fetch_clock::input()));
          }
        },
        "Unsupported full-source context must fail visibly");
    require(error.contains(diagnostic) && error.contains("pc=") && error.contains("omr=") &&
                run.faulted() && !machine.bus().once_latch(),
            "Unsupported mode/port/REP fails with execution context before any guessed write");
    if (!input.repeat) {
      require(machine.cpu().debug_state() == before,
              "A refused standalone instruction never retires");
    }
    if (first.empty()) { first = error; }
    require(first == error, "Restored fault reports are deterministic");
    static_cast<void>(checkpoint::rejects([&] { static_cast<void>(run.checkpoint(machine)); },
                                          "Faulted state must not create a trusted checkpoint"));
  }
}

void refuses_initialization(const unsigned mode) {
  DspMachine machine;
  const auto before = prepare(machine, {.status = mode});
  const auto error =
      checkpoint::rejects([&] { const DspFetchRun run(contract(), machine); },
                          "Unsupported initial fetch context must fail before a run exists");
  require(error.contains("Program-fetch start requires an ordinary pending opcode") &&
              error.contains("pc=") && error.contains("omr=") &&
              machine.cpu().debug_state() == before && !machine.bus().once_latch(),
          "Strict constructor refusal leaves complete native state and peripheral data unchanged");
}

void verify_held(DspFetchRun &run, DspMachine &machine, const DspDebugState &before) {
  const auto result = scalar_store::expected({.value = source});
  while (run.completed_cycles() < 7) {
    const auto cycle = run.advance_cycle(machine, fetch_clock::input());
    const auto period = run.completed_cycles();
    const auto &phase = pipeline::stages(cycle);
    auto wanted = before;
    if (period == 7) {
      ++wanted.cpu.pc;
      ++wanted.native_instructions;
      ++wanted.native_cycles;
      wanted.cpu.sr = masked_status | result.flags;
    }
    require(phase.memory.has_value() == (period == 6) &&
                phase.retirement_published == (period == 7) &&
                machine.cpu().debug_state() == wanted,
            "Actual accepted F2 opcode owns the sole E1 write and complete E2 native state");
    const auto latch = machine.bus().once_latch();
    require(period >= 6 ? latch == result.value : !latch,
            "Later backing-code mutation cannot replace the accepted full-source instruction");
  }
}

void fetched_identity() {
  DspMachine machine;
  const auto before = prepare(machine, {});
  DspFetchRun run(contract(), machine);
  pipeline::clocks(run, machine, 3);
  machine.bus().write({.address = entry, .value = 0});
  const auto saved = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    verify_held(run, machine, before);
  }
  DspMachine clone;
  DspFetchRun continuation(saved, clone);
  verify_held(continuation, clone, before);
}

void policy_checks() {
  constexpr unsigned b_field = 0x100;
  for (const unsigned word : {once_store, reserved_x, unknown_y}) {
    for (const unsigned part : {0U, b_field}) {
      require(
          full_accumulator_peripheral_store(word | part) && !fetch_repeat_body(word | part),
          "Full peripheral source admission never implicitly admits repeated peripheral writes");
    }
  }
  for (const unsigned word : {raw_a0, raw_a2}) {
    require(!decode_memory_stage(word) && !full_accumulator_peripheral_store(word),
            "Other accumulator parts retain their explicit unsupported-form refusal");
  }
}
}

int main() {
  try {
    policy_checks();
    refuses({.opcode = reserved_x}, "write X:0xffff80");
    refuses({.opcode = unknown_y}, "write Y:0xfffffc");
    refuses({.status = reserved_scale}, "scaling");
    for (unsigned bit = 1; bit <= unsupported_modes; bit <<= 1U) {
      constexpr unsigned initial_refusal = 0x12000; // Existing CE/FV fetch-start guard.
      if ((bit & unsupported_modes) == 0) { continue; }
      if ((bit & initial_refusal) != 0) {
        refuses_initialization(bit);
      } else {
        refuses({.status = bit}, "mode/interrupt");
      }
    }
    refuses({.repeat = true}, "repeated Decode");
    refuses({.opcode = raw_a0}, "automatic DSP decode");
    refuses({.opcode = raw_a2}, "automatic DSP decode");
    fetched_identity();
    std::cout << "Full MOVEP guards: strict ports,modes,REP,parts,faulted snapshots,actual held F2 "
                 "and fresh clone\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
