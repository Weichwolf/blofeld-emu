// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp_modifier_window_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <exception>
#include <iostream>
#include <vector>

namespace {
using namespace blofeld;
using test::require;
namespace window = blofeld::test::modifier_window;
constexpr unsigned from_control = 0x0460a0; // FM13-131/12-20: W=0,M1 -> M[destination].
constexpr unsigned field_shift = 8;
constexpr unsigned high_modulo = 0xff0007;
constexpr unsigned ordinary_modulo = 7;
unsigned windows = 0;
unsigned restores = 0;
unsigned clones = 0;

void finish(DspFetchRun &run,
            DspMachine &machine,
            const DspDebugState &before,
            const window::Case sample) {
  while (run.completed_cycles() < window::last(sample)) {
    window::step(run, machine, before, sample);
  }
  require(run.elapsed_ticks() == window::last(sample) * test::pipeline::period,
          "W=0 retains the independently specified zero/one/two/three AGU holds");
}

void one(const window::Case sample) {
  DspMachine machine;
  const auto before = window::prepare(machine, sample);
  // Setup the own future word before its first F2; the current owned opcode is an unchanged NOP.
  machine.bus().write({.address = window::entry + 1,
                       .value = from_control | (sample.destination << field_shift) | 1U});
  DspFetchRun run(window::contract(), machine);
  std::vector<DspFetchCheckpoint> cuts;
  for (const unsigned cut : {6U, 8U}) {
    while (run.completed_cycles() < cut) { window::step(run, machine, before, sample); }
    cuts.push_back(run.checkpoint(machine));
  }
  finish(run, machine, before, sample);
  const auto expected = machine.cpu().debug_state();
  for (const auto &cut : cuts) {
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(cut, machine);
      finish(run, machine, before, sample);
      require(machine.cpu().debug_state() == expected,
              "Every restored period retains independent full CPU,modifier,address,flags and "
              "memory expectations");
      ++restores;
    }
    DspMachine target;
    DspFetchRun clone(cut, target);
    finish(clone, target, before, sample);
    require(target.cpu().debug_state() == expected && clone.elapsed_ticks() == run.elapsed_ticks(),
            "Fresh native ownership retains the checked AGU state and real memory cells");
    ++clones;
  }
  ++windows;
}
}

int main() {
  try {
    for (const unsigned value : {high_modulo, window::linear_modifier}) {
      for (unsigned gap = 0; gap < 4; ++gap) {
        for (const unsigned destination : {0U, ordinary_modulo}) {
          for (const unsigned initial : {ordinary_modulo, window::linear_modifier}) {
            for (const bool y : {false, true}) {
              one({.family = window::Family::agu,
                   .gap = gap,
                   .value = value,
                   .destination = destination,
                   .initial_modifier = initial,
                   .y = y});
            }
          }
        }
      }
    }
    std::cout << "PASS MODIFIER_AGU windows=" << windows << " restores=" << restores
              << " fresh-owner-clones=" << clones << '\n';
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
