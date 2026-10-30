// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"
#include "native_access_fixture.hpp"

#include <algorithm>
#include <span>

namespace blofeld::test::peripheral_pipeline {
struct UnknownFetchStore {
  unsigned words = 1, payload = fetch_clock::initial_r0;
};

inline void unknown_fetch(DspMachine &machine, DspFetchRun &run, const UnknownFetchStore store) {
  constexpr unsigned missing = fetch_clock::program + 4;
  const auto before = machine.cpu().debug_state();
  require(run.completed_cycles() == 5 && !machine.bus().once_latch(),
          "The real peripheral E1 and its simultaneous unknown F2 are still pending");
  const auto saved = run.checkpoint(machine);
  const auto e1 = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(e1);
  const auto fetched = machine.reference_fetch();
  const auto samples = std::span(fetched.words).first(fetched.used);
  require(phase.memory && phase.execute1 && phase.execute1->address == fetch_clock::program &&
              !phase.retirement_published && run.completed_cycles() == 6 &&
              run.elapsed_ticks() == 6 * pipeline::period &&
              machine.cpu().debug_state() == before &&
              machine.bus().once_latch() == store.payload &&
              std::ranges::any_of(samples,
                                  [](const DspProgramSample sample) {
                                    return sample.address == missing && !sample.data;
                                  }),
          "The older native E1 writes once while real mapped F2 retains an unknown sample");
  const auto unresolved = run.checkpoint(machine);
  for (const unsigned replacement : {0U, word_mask}) {
    run.restore(unresolved, machine);
    machine.bus().write({.address = missing, .value = replacement});
    require(machine.reference_fetch() == fetched,
            "Backing RAM cannot initialize an already captured unknown F2 receipt");
    const auto error = native_access::fault(
        [&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
    require(error.contains("Unknown DSP program sample read P:" + hex(missing, 6)) &&
                error.contains("pc=") && error.contains("omr=") && run.faulted() &&
                run.completed_cycles() == 6 && run.elapsed_ticks() == 6 * pipeline::period &&
                machine.cpu().debug_state() == before &&
                machine.bus().once_latch() == store.payload && machine.bus().last_dma_cycle() == 6,
            "Actual unknown Decode fails without another clock or E2 and preserves the older I/O");
    static_cast<void>(checkpoint::rejects([&] { static_cast<void>(run.checkpoint(machine)); },
                                          "Faulted unknown Decode cannot create a checkpoint"));
  }
  run.restore(saved, machine);
  require(!machine.bus().once_latch(), "Trusted restore before E1 restores the actual port state");
  for (unsigned offset = 0; offset < 2; ++offset) {
    machine.bus().write({.address = missing + offset, .value = 0});
  }
  const auto recovered_e1 = run.advance_cycle(machine, fetch_clock::input());
  const auto recovered_e2 = run.advance_cycle(machine, fetch_clock::input());
  auto expected = before;
  expected.cpu.pc += store.words;
  ++expected.native_instructions;
  expected.native_cycles += store.words;
  require(
      pipeline::stages(recovered_e1).memory && !pipeline::stages(recovered_e2).memory &&
          pipeline::stages(recovered_e2).retirement_published && !run.faulted() &&
          machine.cpu().debug_state() == expected && run.completed_cycles() == 7 &&
          run.elapsed_ticks() == 7 * pipeline::period &&
          machine.bus().once_latch() == store.payload && machine.bus().last_dma_cycle() == 7,
      "Known F2 sampled after trusted restore permits one real write and genuine E2 retirement");
}
}
