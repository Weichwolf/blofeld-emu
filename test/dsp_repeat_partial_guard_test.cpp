// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_repeat_partial_fixture.hpp"
#include "dsp_scalar_store_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
using namespace blofeld::test::repeat_partial;

void preserved(DspFetchRun &run,
               DspMachine &machine,
               const Trace &prefix_trace,
               const Case sample,
               const std::string &error,
               const std::string &reason) {
  const bool expected_reason =
      reason == "repeated body" ? body_refusal(error) : error.contains(reason);
  require(expected_reason && error.contains("pc=") && error.contains("omr=") && run.faulted() &&
              prefix_trace.bodies == 0 && prefix_trace.stores == 0 && prefix_trace.addresses == 0 &&
              machine.cpu().debug_state() == prefix_trace.expected,
          "Unqualified REP body/scaling refuses before any body effect with hardware context");
  memory(machine, sample, false);
}

void refused(const Case sample, const unsigned word, const std::string &reason) {
  DspMachine machine;
  prepare(machine, sample);
  // Own synthetic program, before its first F1/F2 capture.
  machine.bus().write({.address = body, .value = word});
  Trace trace{.expected = machine.cpu().debug_state()};
  DspFetchRun run(scalar_store::contract(), machine);
  for (unsigned index = 0; index < sample.count + 64; ++index) {
    const Saved saved{.checkpoint = run.checkpoint(machine), .trace = trace};
    std::optional<std::string> failure;
    try {
      static_cast<void>(advance(run, machine, trace, sample));
    } catch (const Fault &error) { failure = error.what(); }
    if (!failure) { continue; }
    preserved(run, machine, saved.trace, sample, *failure, reason);
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved.checkpoint, machine);
      auto tail = saved.trace;
      const auto error =
          checkpoint::rejects([&] { static_cast<void>(advance(run, machine, tail, sample)); },
                              "A restored unqualified REP continuation must refuse again");
      preserved(run, machine, saved.trace, sample, error, reason);
    }
    return;
  }
  throw Fault("Own unqualified REP body unexpectedly completed");
}
}

int main() {
  try {
    // Full/partial loads,parallel ALU,modes5..7 and full B-long stay unqualified.
    constexpr std::array words{0x56d800U,
                               0x5fd800U,
                               0x50d800U,
                               0x52d800U,
                               0x54d800U,
                               0x51d800U,
                               0x53d800U,
                               0x55d800U,
                               0x545813U,
                               0x546800U,
                               0x547000U,
                               0x547800U,
                               0x495c00U};
    for (const auto word : words) { refused({}, word, "repeated body"); }
    constexpr unsigned reserved_scaling = 0xc00;
    for (const bool b : {false, true}) {
      for (const bool y : {false, true}) {
        for (unsigned part = 0; part < 3; ++part) {
          const Case sample{.store = {.scale = reserved_scaling, .b = b, .y = y},
                            .source_part = part};
          refused(sample, opcode(sample), "accumulator MOVE scaling read P:");
        }
        constexpr std::uint64_t raw_input = 0x80800000654321;
        constexpr unsigned incoming_flags = 0xff;
        one({.count = 3,
             .reg = true,
             .store = {.value = raw_input, .flags = incoming_flags, .b = b, .y = y},
             .source_part = 2});
      }
    }
    std::cout << "13 repeated-form/12 reserved-scale refusals with two suffixes and4 incoming-CCR "
                 "controls passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
