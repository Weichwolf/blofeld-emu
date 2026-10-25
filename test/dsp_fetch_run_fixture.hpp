// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "bank_fixture.hpp"
#include "dma_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <cstdint>
#include <span>

namespace blofeld::test::fetch_clock {
inline constexpr unsigned program = 0x100;
inline constexpr unsigned initial_r0 = 0x7e;
inline constexpr unsigned copied_r0 = 0x34;
inline constexpr unsigned program_source = 0x800;
inline constexpr unsigned program_copy = 0x88024a;
inline constexpr unsigned csr0 = 0xffff8f;
inline constexpr unsigned count0 = 0xffff8c;
inline constexpr unsigned compare0 = 0xffff8d;
inline constexpr unsigned load0 = 0xffff8e;
inline constexpr unsigned timer_flag = 0x200000;
inline constexpr unsigned load_value = 7;
inline constexpr unsigned compare_value = 9;
inline constexpr unsigned extal = 12000000;
inline constexpr unsigned timebase = 540000000;

inline DspFetchRunInputs contract() {
  return {
      .extal_hz = extal, .timebase_hz = timebase, .timing = DspFetchTiming::reference_cycle_order};
}

inline DspFetchCycleInputs input(const bool admit = false) {
  return {.known_requests = 15, .grant = DspDmaGrant::available, .admit_native = admit};
}

struct Setup {
  banks::Transfer transfer{.control = 0};
  bool timer = false;
  DspReferenceExecution execution = reference::boundaries;
};

inline void
prepare(DspMachine &machine, const std::span<const std::uint32_t> words, const Setup setup = {}) {
  DspProgram code;
  code.immediate("r0", initial_r0);
  if (setup.timer) {
    for (const auto store : {DspStore{.address = csr0},
                             DspStore{.address = load0, .value = load_value},
                             DspStore{.address = compare0, .value = compare_value},
                             DspStore{.address = csr0, .value = 7}}) {
      code.emit("movep #>" + number(store.value) + ",x:<<" + number(store.address));
    }
  }
  for (const auto store :
       {DspStore{.address = dma_test::source0, .value = setup.transfer.source},
        DspStore{.address = dma_test::destination0, .value = setup.transfer.destination},
        DspStore{.address = dma_test::counter0},
        DspStore{.address = dma_test::control0, .value = setup.transfer.control}}) {
    code.emit("movep #>" + number(store.value) + ",x:<<" + number(store.address));
  }
  code.word(reference::long_jump);
  code.word(program);
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::ram(machine.bus(), program, words);
  static_cast<void>(machine.advance_dma(banks::inputs({.index = 0}), banks::use()));
  checkpoint::run(machine, setup.timer ? 10 : 6);
  require(
      machine.cpu().state().pc == program && machine.cpu().state().r[0] == initial_r0,
      "Real native MOVEP setup configures the pre-epoch DMA/timer state and enters own RAM code");
  static_cast<void>(machine.begin_reference_instruction(setup.execution));
}

void guard_checks();
}
