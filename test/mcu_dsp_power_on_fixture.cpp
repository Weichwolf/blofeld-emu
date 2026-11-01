// SPDX-License-Identifier: GPL-3.0-only
#include "mcu_dsp_power_on_fixture.hpp"

#include "board/mcu_dsp_runtime.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "mcu_checkpoint_fixture.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace blofeld::test::power_on {
void require(const bool condition, const char *message) {
  if (!condition) { throw Fault(message); }
}

McuRunContract contract() {
  return {.clock = {.oscillator_hz = oscillator,
                    .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = McuRunMemory::unstretched_internal};
}

McuDspRuntimeInputs inputs() {
  return {.timing = OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts,
          .maximum_cycles_per_period = 16,
          .writes = McuWriteOrder::before_mcu,
          .external_writes = McuWriteOrder::before_mcu,
          .wait_dsp_first = true,
          .reset_dsp_first = true,
          .initial_reset = OwnDspResetInputs{
              .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}};
}

DspDmaInputs pins() {
  return {.known_requests = 15,
          .timing = DspDmaTiming::reference_sampled_cycles,
          .grant = DspDmaGrant::available};
}

dsp56300::Registers nop_registers(const std::uint64_t cycles) {
  constexpr unsigned reset_status = 0xc00300;
  constexpr unsigned reset_omr = 0x305;
  dsp56300::Registers expected{};
  expected.pc = boot;
  expected.sr = reset_status;
  expected.omr = reset_omr;
  expected.m.fill(dsp56300::word_mask);
  if (cycles >= 3) { expected.pc += static_cast<unsigned>((cycles - 1) / 2); }
  return expected;
}

dsp56300::Progress nop_progress(const std::uint64_t cycles) {
  const auto count = cycles < 2 ? 0 : (cycles - 2) / 2;
  return {.instructions = count, .estimated_cycles = count};
}

Fixture::Fixture(const unsigned synchronization, const bool install_rom)
    : mcu(checkpoint_image(), BLOFELD_DEFAULT_SLA, contract(), {synchronization}) {
  if (install_rom) {
    const std::array<std::uint32_t, nop_words> program{};
    dsp.bus().install_rom({.space = DspSpace::p, .start = boot, .words = program});
  }
  runtime = std::make_unique<McuDspRuntime>(inputs(), mcu, dsp, events());
  require(cursor == 1 && observed == std::vector<std::uint64_t>{0} && runtime->end_ticks() == 0 &&
              runtime->completed_cycles() == 0 && mcu.run_clock().bus_cycles() == 0 &&
              dsp.clock().elapsed_ticks() == 0,
          "Native POR did not consume its actual time-zero input before any clock period");
  attach();
}

Fixture::~Fixture() {
  mcu.set_cycle_observer({});
  mcu.set_reset_period_observer({});
}

DspRuntimeEvents Fixture::events() {
  return {
      .identity = identity,
      .next_ticks = [this]() -> std::optional<std::uint64_t> {
        return cursor < event_ticks.size() ? std::optional{event_ticks.at(cursor)} : std::nullopt;
      },
      .observe =
          [this](const std::uint64_t ticks) {
            require(ticks == event_ticks.at(cursor) && mcu.system_reset_active() &&
                        mcu.run_clock().bus_cycles() == 0 && dsp.clock().native_cycle_units() == 0,
                    "Initial pad events preceded real held-reset time or native vector fetch");
            dsp.bus().timers().set_tio_drive(0, cursor != 0);
            observed.push_back(ticks);
            ++cursor;
          },
      .order = DspPeerOrder::peer_first,
      .maximum_events_per_period = 4};
}

void Fixture::record(const DspRuntimeCycle &cycle) {
  require(cycle.completion_ticks >= recovery_end + reset_period,
          "A native DSP clock preceded its actual held/reset-recovery interval");
  const auto count = (cycle.completion_ticks - recovery_end) / reset_period;
  require(
      cycle.completion_ticks == recovery_end + (count * reset_period) &&
          cycle.completion_ticks - cycle.start_ticks == reset_period && cycle.cycle + 1 == count &&
          runtime->completed_cycles() == count && dsp.cpu().registers() == nop_registers(count) &&
          dsp.cpu().progress() == nop_progress(count) &&
          dsp.clock().timer_clk2_edges() == count / 2,
      "Native POR NOP clocks,PCU/retirement,flags or full registers lost independent expectations");
  const auto pending = dsp.cpu().pending_instruction();
  trace.push_back({.registers = dsp.cpu().registers(),
                   .progress = dsp.cpu().progress(),
                   .pending = pending ? std::optional{pending->effect} : std::nullopt,
                   .issued = cycle.result.issued_address,
                   .start = cycle.start_ticks,
                   .completion = cycle.completion_ticks,
                   .clk2 = dsp.clock().timer_clk2_edges(),
                   .captured = cycle.result.captured,
                   .retired = cycle.result.native && cycle.result.native->retired});
}

void Fixture::attach() {
  mcu.set_reset_period_observer([this](const McuResetPeriod period) {
    runtime->begin_reset_period(period, pins, [this](const auto &cycle) { record(cycle); });
  });
  mcu.set_cycle_observer([this](const Hc12BusCycle period) {
    if (mcu.system_reset_active()) { slots += period.kind; }
    runtime->begin_cycle(period, false, pins, [this](const auto &cycle) { record(cycle); });
  });
}

void Fixture::restore(const McuDspRuntimeCheckpoint &snapshot, const unsigned source_cursor) {
  require(source_cursor <= event_ticks.size(), "Restored POR input cursor exceeds its source");
  runtime.reset();
  cursor = source_cursor;
  observed.assign(event_ticks.begin(), event_ticks.begin() + source_cursor);
  trace.clear();
  slots.clear();
  runtime = std::make_unique<McuDspRuntime>(snapshot, mcu, dsp, events());
  attach();
}

void Fixture::advance_reset(const unsigned edges, const McuResetPad pad) {
  require(mcu.advance_system_reset(edges, pad) == edges,
          "Actual native SYSCLK reset sequence did not complete its requested edge count");
}

void Fixture::step() {
  require(mcu.cpu().step().instruction_executed, "Actual post-vector MCU NOP did not retire");
  runtime->end_instruction();
}
}
