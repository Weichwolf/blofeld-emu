// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_repeat_register_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
using namespace blofeld::test;
namespace raw = repeat_register;
constexpr unsigned expected_refusals = 22;
constexpr unsigned expected_controls = 8;
constexpr unsigned reserved_scale = 0xc00;
constexpr unsigned all_flags = 0xff;
constexpr unsigned n4_field = 28;

struct Case {
  raw::Case store;
  std::optional<unsigned> prefix = std::nullopt;
  std::optional<unsigned> body = std::nullopt;
  std::string_view reason;
};

void preserved(DspFetchRun &run,
               DspMachine &machine,
               const raw::Trace &before,
               const Case sample,
               const std::string &error) {
  // Cold Decode: prefix at4, repeated body at8; zero-count admission is E1 at7.
  // Only the older entry NOP may retire before the count refusal in that same period.
  constexpr unsigned prefix_decode = 4;
  constexpr unsigned body_decode = 8;
  constexpr unsigned count_execute = 7;
  unsigned fault_period = count_execute;
  if (sample.body) { fault_period = body_decode; }
  if (sample.prefix) { fault_period = prefix_decode; }
  auto expected = before.initial;
  if (fault_period >= count_execute) {
    expected.cpu.pc = raw::prefix;
    ++expected.native_instructions;
    ++expected.native_cycles;
  }
  require(run.completed_cycles() + 1 == fault_period,
          "Each excluded form faults at its independently scheduled Decode or count E1");
  require(error.contains(sample.reason) && error.contains("pc=") && error.contains("omr=") &&
              run.faulted() && before.bodies == 0 && before.stores == 0 && before.addresses == 0 &&
              machine.cpu().debug_state() == expected,
          "Excluded forms retain exactly the older NOP retirement with no REP or body effect");
  raw::cells(machine, sample.store, 0);
}

void refused(const Case sample) {
  DspMachine machine;
  raw::prepare(machine, sample.store);
  if (sample.prefix) { machine.bus().write({.address = raw::prefix, .value = *sample.prefix}); }
  if (sample.body) { machine.bus().write({.address = raw::body, .value = *sample.body}); }
  const auto before = machine.cpu().debug_state();
  raw::Trace trace{.expected = before, .initial = before};
  auto contract = fetch_clock::contract();
  contract.execution = raw::execution;
  contract.pipeline = DspFetchPipeline::automatic_memory;
  DspFetchRun run(contract, machine);
  for (unsigned period = 0; period < raw::bound_slack; ++period) {
    const raw::Saved saved{.checkpoint = run.checkpoint(machine), .trace = trace};
    std::optional<std::string> failure = std::nullopt;
    try {
      static_cast<void>(raw::advance(run, machine, trace, sample.store));
    } catch (const Fault &error) { failure = error.what(); }
    if (!failure) { continue; }
    preserved(run, machine, saved.trace, sample, *failure);
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved.checkpoint, machine);
      auto tail = saved.trace;
      const auto error = checkpoint::rejects(
          [&] { static_cast<void>(raw::advance(run, machine, tail, sample.store)); },
          "Restored excluded REP forms must fail again");
      require(error == *failure, "Restored refusals preserve the exact address/opcode/PC/context");
      preserved(run, machine, saved.trace, sample, error);
    }
    return;
  }
  throw Fault("Excluded own REP form unexpectedly completed");
}

unsigned excluded_forms() {
  // Native-valid but unqualified register count sources remain excluded.
  constexpr std::array prefixes{0x06c820U,
                                0x06ca20U,
                                0x06ce20U,
                                0x06c920U,
                                0x06cd20U,
                                0x06cb20U,
                                0x06cf20U,
                                0x06c520U,
                                0x06c620U,
                                0x06c720U,
                                0x06dc20U};
  for (const auto word : prefixes) {
    refused({.store = {}, .prefix = word, .reason = "Unqualified automatic DSP decode P:"});
  }
  // Loads, parallel ALU, two-word absolute stores, R stores and flow controls.
  constexpr std::array bodies{
      0x44d900U, 0x74d900U, 0x445913U, 0x745913U, 0x447000U, 0x747000U, 0x635900U, 0x0af080U};
  for (const auto word : bodies) {
    refused({.store = {}, .body = word, .reason = "Unqualified repeated Decode read P:"});
  }
  constexpr std::array counters{raw::Counter::immediate, raw::Counter::x0, raw::Counter::a1};
  for (const auto counter : counters) {
    refused({.store = {.counter = counter, .count_input = 0},
             .reason = "Unqualified native program-fetch control opcode"});
  }
  return static_cast<unsigned>(prefixes.size() + bodies.size() + counters.size());
}

unsigned raw_scale_controls() {
  unsigned controls = 0;
  for (const unsigned source : {raw::data_first, n4_field}) {
    for (const bool y : {false, true}) {
      for (const unsigned flags : {1U, all_flags}) {
        static_cast<void>(raw::one({.source = source,
                                    .flags = reserved_scale | flags,
                                    .counter = raw::Counter::a1,
                                    .y = y}));
        ++controls;
      }
    }
  }
  return controls;
}
}

int main() {
  try {
    const auto refusals = excluded_forms();
    const auto controls = raw_scale_controls();
    require(refusals == expected_refusals && controls == expected_controls,
            "All count/body exclusions, zero counts and unrelated raw-scale controls execute");
    std::cout << refusals << " REP register/body/count refusals with two suffixes; " << controls
              << " raw-scale controls passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
