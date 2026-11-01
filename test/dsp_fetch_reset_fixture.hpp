// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"
#include "esai_fixture.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace blofeld::test::fetch_reset {
using esai::rejects;
using esai::require;
inline constexpr unsigned boot = 0xff0000, ram = 0x100, marker = 0x123456;
inline constexpr std::uint64_t period = 45, held_ticks = 91, partial_recovery = 44;
inline constexpr std::uint64_t recovery_ticks = 135;
inline constexpr unsigned suffix_cycles = 32, cuts = 20;
inline constexpr unsigned reset_status = 0xc00300, reset_omr = 0x305, reset_pctl = 0x04401d;
inline constexpr unsigned immediate_x0 = 0x44f400, clear_a = 0x200013;
inline constexpr unsigned add_x0 = 0x200040, jump = 0x0af080;
inline constexpr unsigned accumulator_bits = 24;
inline constexpr auto timing = OwnDspAutomaticTiming::one_effect_per_cycle;
inline constexpr DspRunClockInputs clocks{.extal_hz = 12000000,
                                          .timebase_hz = 540000000,
                                          .switch_timing =
                                              DspClockSwitch::reference_cycle_boundary};

OwnDspResetInputs pins();
DspDmaInputs inputs(OwnDspMachine &machine);

struct Ledger {
  std::uint64_t cycles = 0, ticks = 0, reset_ticks = 0, clk2 = 0, epoch_start = 0, epoch = 0;
  std::uint64_t adds = 0, instructions = 0, cost = 0;
  unsigned pc = boot;
  dsp56300::Registers expected{};
};

struct Observation {
  dsp56300::Registers registers;
  dsp56300::Progress progress;
  dsp56300::Fault fault;
  std::optional<dsp56300::InstructionEffect> pending;
  std::optional<dsp56300::InstructionOutcome> outcome;
  std::optional<DspHardwareResetState> reset;
  std::optional<std::uint64_t> dma;
  std::optional<std::uint32_t> issued;
  std::uint64_t ticks = 0, cycles = 0, clk2 = 0, epoch = 0;
  bool captured = false, retired = false;
  bool operator==(const Observation &) const = default;
};

enum class Start : std::uint8_t { powered, blank };

struct Fixture {
  explicit Fixture(Start start = Start::powered);
  void power_on();
  void reset();
  void held(std::uint64_t requested, std::uint64_t expected);
  void time() const;
  void gated();
  OwnDspFetchCycleResult step();
  void advance(unsigned count);
  void oracle();
  void restore(const OwnDspTimelineCheckpoint &checkpoint, const Ledger &saved);
  [[nodiscard]] Observation observe(const OwnDspFetchCycleResult *cycle = nullptr);

  OwnDspMachine machine{clocks};
  std::unique_ptr<OwnDspTimeline> timeline;
  Ledger ledger;
  std::vector<Observation> trace;
};

void store_checks();
void guard_checks();
}
