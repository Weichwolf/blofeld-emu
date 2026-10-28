// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_short_load_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
using namespace blofeld::test::short_load;

void unknown(const Case sample, const unsigned cut) {
  DspMachine machine;
  const auto before = prepare(machine, sample, false);
  require(!machine.bus().last_dma_cycle(), "Unclocked native prefix establishes DMA epoch zero");
  begin(machine);
  DspFetchRun run(contract(), machine);
  for (unsigned count = 1; count <= cut; ++count) {
    period(run, machine, before, sample, count, false);
  }
  const auto saved = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    for (unsigned count = cut + 1; count <= 5; ++count) {
      period(run, machine, before, sample, count, false);
    }
    const auto error = checkpoint::rejects(
        [&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); },
        "Missing AA6 operand must fail at actual native E1");
    const std::string space = sample.y ? "Y:" : "X:";
    require(error.contains("read " + space + hex(sample.address, 6)) &&
                error.contains("pc=0x000100") && error.contains("omr=0x000300") && run.faulted() &&
                run.completed_cycles() == 5 && machine.bus().last_dma_cycle() == 5 &&
                machine.cpu().debug_state() == before,
            "Missing RAM reports operation,address,PC and OMR without publishing a result");
    const auto quarantine =
        checkpoint::rejects([&] { machine.cpu().step(); }, "Faulted native CPU cannot continue");
    require(quarantine.contains("requires reset after a rejected reference step") &&
                machine.cpu().debug_state() == before,
            "Every new destination retains public native quarantine after an unknown read");
    static_cast<void>(checkpoint::rejects([&] { static_cast<void>(run.checkpoint(machine)); },
                                          "A fault cannot become a trusted pipeline checkpoint"));
    memory(machine, sample, false);
  }
  run.restore(saved, machine);
  for (unsigned count = cut + 1; count <= 5; ++count) {
    period(run, machine, before, sample, count, false);
  }
  machine.bus().write({.space = sample.y ? DspSpace::y : DspSpace::x,
                       .address = sample.address,
                       .value = sample.value});
  period(run, machine, before, sample, 6);
  period(run, machine, before, sample, 7);
}

void captured(const Case sample) {
  DspMachine machine;
  const auto before = prepare(machine, sample);
  begin(machine);
  DspFetchRun run(contract(), machine);
  for (unsigned count = 1; count <= 4; ++count) { period(run, machine, before, sample, count); }
  const auto saved = run.checkpoint(machine);
  auto changed = sample;
  changed.destination = (sample.destination + 1) % 32;
  changed.address = sample.address == 0 ? addresses.back() : addresses.front();
  changed.y = !sample.y;
  auto live = sample;
  live.value = values.back();
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    machine.bus().write({.address = entry, .value = opcode(changed)});
    machine.bus().write({.space = sample.y ? DspSpace::y : DspSpace::x,
                         .address = sample.address,
                         .value = live.value});
    for (unsigned count = 5; count <= 7; ++count) { period(run, machine, before, live, count); }
    require(machine.bus().read({.address = entry}) == opcode(changed),
            "The captured instruction owns address/target while E1 reads live source RAM");
  }
}

void refusal(const Case sample, const unsigned word) {
  DspMachine machine;
  const auto before = prepare(machine, sample);
  require(!machine.bus().last_dma_cycle(), "Unclocked refusal prefix establishes DMA epoch zero");
  machine.bus().write({.address = entry, .value = word});
  begin(machine);
  DspFetchRun run(contract(), machine);
  for (unsigned count = 0; count < 2; ++count) {
    static_cast<void>(run.advance_cycle(machine, fetch_clock::input()));
  }
  const auto saved = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    const auto error = checkpoint::rejects(
        [&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); },
        "Excluded short store,parallel ALU or non-M MOVEC must refuse before effects");
    require(error.contains("automatic DSP decode") && error.contains("pc=0x000100") &&
                error.contains("omr=0x000300") && run.completed_cycles() == 2 &&
                machine.bus().last_dma_cycle() == 1 && machine.cpu().debug_state() == before,
            "Unqualified short forms fail visibly at Decode without CPU or RAM effects");
    memory(machine, sample);
  }
}

unsigned refusals(const Case sample) {
  constexpr unsigned write_destination = 0x8000;
  constexpr unsigned parallel_clr = 0x13;
  refusal(sample, opcode(sample) & ~write_destination);
  if (sample.destination < m_begin) { refusal(sample, opcode(sample) | parallel_clr); }
  return sample.destination < m_begin ? 2U : 1U;
}

unsigned invalid_regular_refusals() {
  constexpr unsigned invalid_base = 0x408000;
  unsigned count = 0;
  for (unsigned selector = 0; selector < 4; ++selector) {
    for (const auto address : addresses) {
      for (const bool y : {false, true}) {
        const Case sample{.address = address, .value = values.back(), .ccr = flags.back(), .y = y};
        refusal(sample,
                invalid_base | (selector << register_shift) | (address << address_shift) |
                    (y ? ordinary_y : 0U));
        ++count;
      }
    }
  }
  return count;
}

unsigned control_refusals() {
  unsigned count = 0;
  for (unsigned destination = 8; destination < 32; ++destination) {
    for (const auto address : addresses) {
      for (const bool y : {false, true}) {
        const Case sample{.address = address, .value = values.back(), .ccr = flags.back(), .y = y};
        refusal(sample, m_base | destination | (address << address_shift) | (y ? control_y : 0U));
        ++count;
      }
    }
  }
  return count;
}
}

int main() {
  try {
    unsigned unknown_count = 0;
    unsigned captured_count = 0;
    unsigned refused = 0;
    for (unsigned target = 0; target < 32; ++target) {
      for (const auto address : addresses) {
        for (const bool y : {false, true}) {
          const Case sample{.destination = target,
                            .address = address,
                            .value = values[4],
                            .ccr = flags.back(),
                            .y = y};
          refused += refusals(sample);
          if (address != addresses.front() && address != addresses.back()) { continue; }
          unknown(sample, target % 5);
          captured(sample);
          ++unknown_count;
          ++captured_count;
        }
      }
    }
    refused += control_refusals();
    refused += invalid_regular_refusals();
    constexpr unsigned expected_guards = 128;
    constexpr unsigned expected_refusals = 672;
    require(unknown_count == expected_guards && captured_count == expected_guards &&
                refused == expected_refusals,
            "All32 destinations retain missing/captured guards and672 strict short exclusions");
    std::cout
        << "128 unknown AA6 E1 cases/two faults+trusted repairs,128 captured/live-RAM cases/"
           "256suffixes,672 strict store/parallel/non-M-control refusals/1344suffixes passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
