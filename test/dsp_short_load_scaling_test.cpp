// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_short_load_fixture.hpp"

#include <exception>
#include <iostream>
#include <string_view>

namespace {
using namespace blofeld;
using namespace blofeld::test;
using namespace blofeld::test::short_load;

DspDebugState prepare_scaled(DspMachine &machine, const Case sample, const unsigned scale) {
  const auto initial = prepare(machine, sample);
  const auto status = masked_status | sample.ccr | scale;
  // Own native setup only: actually write SR, settle its mode and jump back to entry.
  const auto text = "move #>" + number(status) + ",sr";
  auto code = checkpoint::assemble(text.c_str());
  require(code.size() == 2, "The independent SR setup has one genuine immediate extension");
  code.insert(code.end(), 5, 0);
  code.push_back(absolute_jump);
  code.push_back(entry);
  checkpoint::ram(machine.bus(), entry, code);
  machine.cpu().step();
  checkpoint::until(machine, entry);
  auto expected = initial.cpu;
  expected.sr = status;
  const auto before = machine.cpu().debug_state();
  require(before.cpu == expected && !machine.bus().last_dma_cycle(),
          "Native SR setup changes only the declared status and does not advance host DMA");
  machine.bus().write({.address = entry, .value = opcode(sample)});
  for (unsigned index = 1; index < 16; ++index) { machine.bus().write({.address = entry + index}); }
  return before;
}

void one(const Case sample, const unsigned scale, const bool staged) {
  DspMachine machine;
  const auto before = prepare_scaled(machine, sample, scale);
  if (!staged) {
    auto expected = before;
    destination(expected.cpu, sample);
    ++expected.cpu.pc;
    ++expected.native_instructions;
    ++expected.native_cycles;
    machine.cpu().step();
    require(machine.cpu().debug_state() == expected,
            "FM13-118/131 input loads do not scale memory data or modify incoming CCR");
    memory(machine, sample);
    return;
  }
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  DspFetchRun run(contract(), machine);
  for (unsigned count = 1; count <= 7; ++count) { period(run, machine, before, sample, count); }
}

unsigned snapshots(const Case sample, const unsigned scale) {
  for (unsigned cut = 0; cut < 7; ++cut) {
    DspMachine machine;
    const auto before = prepare_scaled(machine, sample, scale);
    static_cast<void>(
        machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
    DspFetchRun run(contract(), machine);
    for (unsigned count = 1; count <= cut; ++count) { period(run, machine, before, sample, count); }
    const auto saved = run.checkpoint(machine);
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      for (unsigned count = cut + 1; count <= 7; ++count) {
        period(run, machine, before, sample, count);
      }
    }
  }
  return 7;
}

unsigned variants(const unsigned target, const bool y, const bool staged) {
  unsigned count = 0;
  for (const auto scale : {scale_down, scale_up}) {
    for (const auto value : values) {
      for (const auto ccr : flags) {
        one({.destination = target, .address = addresses[2], .value = value, .ccr = ccr, .y = y},
            scale,
            staged);
        ++count;
      }
    }
  }
  return count;
}
}

int main(const int count, char **arguments) {
  try {
    const std::string_view mode = count == 2 ? arguments[1] : "--staged";
    if (count > 2 || (mode != "--native" && mode != "--staged")) {
      throw Fault("Usage: short-load-scaling-test [--native|--staged]");
    }
    const bool staged = mode == "--staged";
    unsigned cases = 0;
    for (unsigned target = 0; target < destination_count; ++target) {
      for (const bool y : {false, true}) { cases += variants(target, y, staged); }
    }
    constexpr unsigned expected_cases = 3456;
    require(cases == expected_cases, "All36 destinations retain both supported scaling modes");
    unsigned cuts = 0;
    if (staged) {
      for (const unsigned target : {6U, 7U}) {
        for (const bool y : {false, true}) {
          for (const auto scale : {scale_down, scale_up}) {
            cuts += snapshots({.destination = target,
                               .address = addresses[2],
                               .value = values.back(),
                               .ccr = flags.back(),
                               .y = y},
                              scale);
          }
        }
      }
    }
    constexpr unsigned expected_cuts = 56;
    require(cuts == (staged ? expected_cuts : 0U), "Scaled full-accumulator continuations execute");
    std::cout << cases << " AA6 scaling cases," << cuts << "cuts/" << 2 * cuts
              << "suffixes passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
