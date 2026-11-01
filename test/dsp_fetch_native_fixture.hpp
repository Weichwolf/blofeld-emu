// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "bank_fixture.hpp"
#include "common/bus.hpp"
#include "dma_fixture.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "esai_fixture.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <utility>

namespace blofeld::test::fetch_native {
namespace native = dma_test;
using esai::rejects;
using esai::require;
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
inline constexpr unsigned long_r0 = 0x60f400;
inline constexpr unsigned long_omr = 0x05f43a;
inline constexpr unsigned long_jump = 0x0af080;
inline constexpr unsigned short_move = 0x303400;
inline constexpr unsigned faster = 0x02e112;
inline constexpr std::uint64_t base_period = 45;
inline constexpr std::uint64_t faster_period = 10;
inline constexpr auto timing = OwnDspAutomaticTiming::one_effect_per_cycle;
inline constexpr auto model = OwnDspNativeCycleModel::isolated_effect;

inline void healthy(const OwnDspFetchCycleResult &cycle) {
  if (cycle.native && cycle.native->fault.kind != dsp56300::FaultKind::none) {
    const auto fault = cycle.native->fault;
    throw Fault(
        "Native fetch cycle fault kind=" + std::to_string(static_cast<unsigned>(fault.kind)) +
        " address=" + hex(fault.address, 6) + " pc=" + hex(fault.pc, 6));
  }
}

struct Setup {
  banks::Transfer transfer{.control = 0};
  bool timer = false;
  DspRunClockInputs clocks = native::native_clocks;
};

struct Fixture {
  explicit Fixture(const std::span<const std::uint32_t> words, const Setup setup = {})
      : machine(setup.clocks) {
    native::NativeProgram code;
    code.word(long_r0);
    code.word(initial_r0);
    if (setup.timer) {
      for (const auto store :
           {DspStore{.space = DspSpace::x, .address = csr0},
            DspStore{.space = DspSpace::x, .address = load0, .value = load_value},
            DspStore{.space = DspSpace::x, .address = compare0, .value = compare_value},
            DspStore{.space = DspSpace::x, .address = csr0, .value = 7}}) {
        code.write(store);
      }
    }
    for (const auto store :
         {DspStore{
              .space = DspSpace::x, .address = native::source0, .value = setup.transfer.source},
          DspStore{.space = DspSpace::x,
                   .address = native::destination0,
                   .value = setup.transfer.destination},
          DspStore{.space = DspSpace::x, .address = native::counter0},
          DspStore{.space = DspSpace::x,
                   .address = native::control0,
                   .value = setup.transfer.control}}) {
      code.write(store);
    }
    code.word(long_jump);
    code.word(program);
    code.install(machine.bus());
    const auto reset_period = setup.clocks.timebase_hz / setup.clocks.extal_hz;
    const auto hold = 2 * reset_period;
    machine.begin_hardware_reset(
        {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2});
    require(machine.advance_hardware_reset_ticks(hold) == hold, "Actual pre-epoch reset hold");
    machine.release_hardware_reset();
    require(machine.advance_hardware_reset_ticks(hold) == hold, "Actual pre-epoch reset recovery");
    for (std::size_t i = 0; i < words.size(); ++i) {
      machine.bus().write({.address = program + static_cast<unsigned>(i), .value = words[i]});
    }
    for (unsigned i = 0; i < (setup.timer ? 10U : 6U); ++i) { native::step_native(machine); }
    require(machine.cpu().registers().pc == program && machine.cpu().registers().r[0] == initial_r0,
            "Real reset and literal native MOVEP setup establish the pre-epoch state");
    // Explicit stopped-CPU request observation at the epoch, not an elapsed clock.
    static_cast<void>(native::dma(machine, banks::inputs({.index = 0}), banks::use()));
    epoch_ticks = machine.clock().elapsed_ticks();
  }

  void attach() { fetch = std::make_unique<OwnDspFetch>(machine); }

  OwnDspFetchCycleResult advance(const bool admit = false,
                                 const unsigned requests = 0,
                                 const DspDmaGrant grant = DspDmaGrant::available) {
    const auto last = machine.bus().last_dma_cycle();
    const auto inputs =
        banks::inputs({.index = last ? *last + 1 : 0, .requests = requests, .grant = grant});
    auto result = admit ? fetch->advance_dma_automatic(inputs, timing)
                        : fetch->advance_dma_pipeline(inputs, false, model);
    healthy(result);
    return result;
  }

  void restore(const OwnDspFetchCheckpoint &saved) {
    const auto old = std::move(fetch);
    fetch = std::make_unique<OwnDspFetch>(machine, saved);
    require(rejects([&] { static_cast<void>(old->view()); }).contains("Stale"),
            "Joint restore abandons the prior frontend capability");
  }

  [[nodiscard]] std::uint64_t elapsed() const {
    return machine.clock().elapsed_ticks() - epoch_ticks;
  }

  OwnDspMachine machine;
  std::unique_ptr<OwnDspFetch> fetch;
  std::uint64_t epoch_ticks = 0;
};

void guard_checks();
}
