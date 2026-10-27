// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_fetch_schedule.hpp"
#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_fetch_schedule.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "mcu_checkpoint_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using blofeld::test::require;
constexpr unsigned oscillator = 12000000, timebase = 540000000, boot = 0xff0000;
constexpr unsigned drive = 128, settle = 64, reset_ticks = 45, bus_ticks = 90;
constexpr unsigned vector_cycles = 5, pipeline_fill = 6, nop_words = 512;

McuRunContract contract() {
  return {.clock = {.oscillator_hz = oscillator,
                    .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = McuRunMemory::unstretched_internal};
}

McuDspFetchInputs inputs(const bool held) {
  McuDspFetchInputs result{.dsp = {.extal_hz = oscillator,
                                   .timebase_hz = timebase,
                                   .timing = DspFetchTiming::reference_cycle_order,
                                   .execution = DspReferenceExecution::staged_register_boundaries,
                                   .pipeline = DspFetchPipeline::automatic_memory,
                                   .resets = DspFetchResetTiming::reference_cycle_boundary},
                           .shared_write_order = McuDspTieOrder::dsp_before_mcu,
                           .maximum_dsp_cycles_per_period = 16,
                           .reset_order = McuDspResetOrder::dsp_before_mcu};
  if (held) {
    result.initial_reset = McuDspFetchInitialReset{
        .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2};
  }
  return result;
}

DspMachine &prepare(DspMachine &dsp) {
  // Own literal NOP words; no manufacturer ROM or fabricated completion.
  const std::array<std::uint32_t, nop_words> code{};
  dsp.bus().install_rom({.start = boot, .words = code});
  dsp.bus().pll().observe_pinit(false);
  dsp.cpu().reset_spi_boot();
  static_cast<void>(
      dsp.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return dsp;
}

struct Fixture {
  Fixture(const unsigned sync, const bool hold_dsp)
      : mcu(blofeld::test::checkpoint_image(), BLOFELD_DEFAULT_SLA, contract(), {sync}),
        held(hold_dsp),
        schedule(inputs(hold_dsp), mcu, hold_dsp ? dsp : prepare(dsp), events()) {
    require(cursor == 1 && observed == std::vector<std::uint64_t>{0} && schedule.end_ticks() == 0 &&
                schedule.run().completed_cycles() == 0 && mcu.run_clock().bus_cycles() == 0,
            "The joint owner accepts actual initial input at zero before any MCU or DSP period");
    attach();
  }

  Fixture(const Fixture &) = delete;
  Fixture &operator=(const Fixture &) = delete;

  ~Fixture() {
    mcu.set_cycle_observer({});
    mcu.set_reset_period_observer({});
  }

  static DspFetchCycleInputs pins() {
    return {.known_requests = 15, .grant = DspDmaGrant::available};
  }

  DspFetchExternalEvents events() {
    return {
        .identity = identity,
        .next_ticks = [this]() -> std::optional<std::uint64_t> {
          if (cursor < event_ticks.size()) { return event_ticks.at(cursor); }
          return std::nullopt;
        },
        .observe =
            [this](const auto ticks) {
              require(ticks == event_ticks.at(cursor) && mcu.system_reset_active() &&
                          mcu.run_clock().bus_cycles() == 0,
                      "Initial external events observe actual held MCU reset before vector reads");
              if (ticks == reset_ticks) {
                require(schedule.run().completed_cycles() == 0,
                        "Explicit external-before-DSP ordering observes the first native boundary");
              }
              observed.push_back(ticks);
              ++cursor;
            },
        .order = DspExternalOrder::external_before_dsp,
        .maximum_events = 4};
  }

  void record(const DspFetchCycleResult &cycle) {
    ++cycles;
    require(!held && cycle.completed_cycles == cycles &&
                cycle.completion_ticks == cycles * reset_ticks &&
                cycle.period_ticks == reset_ticks && cycle.pipeline.has_value(),
            "Each shared startup interval advances the actual automatic pipeline exactly once");
    if (!cycle.pipeline) { throw Fault("Missing actual automatic pipeline result"); }
    if (cycle.pipeline->retirement_published) {
      require(
          cycle.pipeline->execute2 && cycle.pipeline->execute2->address == boot + retired &&
              cycles == retired + pipeline_fill + 1,
          "Seven-stage NOP retirements have independent literal address and latency expectations");
      ++retired;
    }
  }

  void attach() {
    mcu.set_reset_period_observer([this](const auto period) {
      schedule.begin_reset_period(period, pins, [this](const auto &cycle) { record(cycle); });
    });
    mcu.set_cycle_observer([this](const Hc12BusCycle period) {
      slots += period.kind;
      schedule.begin_cycle(period, false, pins, [this](const auto &cycle) { record(cycle); });
    });
  }

  McuMachine mcu;
  DspMachine dsp;
  std::shared_ptr<const DspFetchEventIdentity> identity = std::make_shared<DspFetchEventIdentity>();
  std::array<std::uint64_t, 2> event_ticks{0, reset_ticks};
  unsigned cursor = 0;
  std::vector<std::uint64_t> observed;
  const bool held;
  std::uint64_t cycles = 0, retired = 0;
  std::string slots;
  McuDspFetchSchedule schedule;
};

void test_joint_startup(const unsigned sync, const bool held) {
  Fixture f(sync, held);
  const auto zero = f.schedule.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    f.cursor = 1;
    f.schedule.restore(zero);
    f.cycles = 0;
    f.retired = 0;
    f.observed = {0};
    f.slots.clear();
    f.attach();
    f.mcu.advance_system_reset(drive + sync, McuResetPad::unknown);
    const auto cut = f.schedule.checkpoint();
    const auto cycles = f.cycles;
    const auto retired = f.retired;
    for (unsigned suffix = 0; suffix < 2; ++suffix) {
      f.cursor = 2;
      f.schedule.restore(cut);
      f.cycles = cycles;
      f.retired = retired;
      f.slots.clear();
      f.attach();
      f.mcu.advance_system_reset(settle, McuResetPad::high);
      f.schedule.end_system_reset();
      const auto edges = drive + sync + settle;
      const auto expected = (edges * reset_ticks) + (vector_cycles * bus_ticks);
      const auto periods = edges + (2 * vector_cycles);
      require(
          !f.schedule.faulted() && !f.mcu.system_reset_active() && f.slots == "VfPPP" &&
              f.schedule.end_ticks() == expected && f.mcu.run_clock().bus_cycles() == 5 &&
              f.mcu.cpu().state().pc == blofeld::test::entry &&
              f.observed == std::vector<std::uint64_t>{0, reset_ticks} &&
              f.schedule.timeline_state().external_events == 2 &&
              f.schedule.run().hardware_reset_active() == held,
          "Initial reset, early input, vector fetch and replay share one exact common timeline");
      if (held) {
        require(f.cycles == 0 && f.retired == 0 &&
                    f.schedule.run().hardware_reset_ticks() == expected &&
                    f.schedule.run().completed_cycles() == 0,
                "Explicit independent DSP reset stays asserted across all MCU startup periods");
      } else {
        require(
            f.cycles == periods && f.retired == periods - pipeline_fill &&
                f.schedule.run().elapsed_ticks() == expected &&
                f.dsp.cpu().state().pc == boot + f.retired,
            "A declared running DSP retires real NOPs throughout MCU startup without a hidden gap");
      }
    }
  }
}

void test_unqualified_order() {
  McuMachine mcu(blofeld::test::checkpoint_image(), BLOFELD_DEFAULT_SLA, contract(), {3});
  DspMachine dsp;
  auto unqualified = inputs(false);
  unqualified.reset_order = McuDspResetOrder::unqualified;
  try {
    const McuDspFetchSchedule schedule(unqualified, mcu, prepare(dsp));
  } catch (const Fault &error) {
    require(
        std::string(error.what()).contains("initial held reset requires an explicit reset order") &&
            mcu.run_clock().elapsed_ticks() == 0 && mcu.run_clock().bus_cycles() == 0,
        "Joining initial reset cannot silently select a board phase/order or consume MCU time");
    return;
  }
  throw Fault("Expected unqualified initial-reset order rejection");
}

void test_missing_rom_after_release() {
  Fixture f(3, true);
  constexpr unsigned nominal_edges = 195;
  f.mcu.advance_system_reset(nominal_edges, McuResetPad::high);
  f.schedule.end_system_reset();
  const auto before = f.schedule.end_ticks();
  f.mcu.set_cycle_observer(
      [&](const Hc12BusCycle period) { f.schedule.begin_cycle(period, false, Fixture::pins); });
  f.schedule.release_dsp_reset();
  try {
    for (unsigned step = 0; step < 2; ++step) {
      f.mcu.cpu().step();
      f.schedule.end_instruction();
    }
  } catch (const Fault &error) {
    const std::string diagnostic(error.what());
    require(
        diagnostic.contains("Unknown DSP memory read") && diagnostic.contains("0xff0000") &&
            f.schedule.faulted() && f.schedule.run().completed_cycles() == 1 &&
            f.schedule.run().hardware_reset_ticks() == before + bus_ticks,
        "A genuinely missing ROM fails at the first post-recovery native read, with its address");
    std::cout << "Missing-ROM startup refusal: " << diagnostic << '\n';
    return;
  }
  throw Fault("Missing DSP ROM must not be replaced by a successful reset or opcode stub");
}
}

int main() {
  try {
    for (unsigned sync = 3; sync <= 6; ++sync) {
      for (const bool held : {false, true}) { test_joint_startup(sync, held); }
    }
    test_unqualified_order();
    test_missing_rom_after_release();
    std::cout
        << "Joint POR: eight explicit profiles, thirty-two suffixes and time-zero events passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
