// SPDX-License-Identifier: GPL-3.0-only
#include "mcu_dsp_fetch_events_fixture.hpp"

#include "board/mcu_dsp_runtime.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace blofeld::test::joint_events {
McuFlashImage image() {
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  constexpr unsigned nop = 0xa7;
  constexpr unsigned protection = 0x7f0d;
  constexpr unsigned security = 0x7f0f;
  constexpr unsigned unsecured = 0xfe;
  // Independent HCS12 encodings: LDAB #1 (P), STAB direct PORTB (Pw).
  constexpr std::array<std::uint8_t, 4> code{0xc6, 1, 0x5b, 1};
  bytes.fill(nop);
  for (unsigned i = 0; i < code.size(); ++i) { bytes.at(i) = code[i]; }
  bytes[protection] = byte_mask;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  return McuFlashImage::from_known(bytes);
}

McuRunContract mcu_contract() {
  return {.clock = {.oscillator_hz = oscillator,
                    .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = McuRunMemory::unstretched_internal};
}

McuDspRuntimeInputs settings(const Case order) {
  return {.timing = OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts,
          .maximum_cycles_per_period = 16,
          .writes = order.native,
          .external_writes = order.external,
          .wait_dsp_first = true,
          .reset_dsp_first = true,
          .initial_reset = OwnDspResetInputs{
              .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 3}};
}

DspDmaInputs cycle_inputs() {
  return {.known_requests = 15,
          .timing = DspDmaTiming::reference_sampled_cycles,
          .grant = DspDmaGrant::available};
}

dsp56300::Registers reset_registers() {
  constexpr unsigned boot = 0xff0000;
  constexpr unsigned status = 0xc00300;
  constexpr unsigned mode = 0x305;
  dsp56300::Registers expected{};
  expected.pc = boot;
  expected.sr = status;
  expected.omr = mode;
  expected.m.fill(dsp56300::word_mask);
  return expected;
}

void prepare(McuMachine &mcu, OwnDspMachine &dsp) {
  mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
  mcu.bus().write(hc12::portb, 0);
  mcu.bus().write(hc12::ddrb, byte_mask);
  mcu.bus().write(marker_address, marker);
  // Authored register MOVEP X:<<$ffffbd,X0; MOVEP X:<<$ffffbd,Y0; NOPs.
  constexpr unsigned boot = 0xff0000;
  constexpr unsigned words = 128;
  constexpr unsigned read_x0 = 0x0444dd;
  constexpr unsigned read_y0 = 0x0446dd;
  std::array<std::uint32_t, words> program{read_x0, read_y0};
  dsp.bus().install_rom({.space = DspSpace::p, .start = boot, .words = program});
}

Fixture::Fixture(const Case policy, const std::uint64_t at, std::vector<std::uint64_t> observations)
    : mcu(image(), BLOFELD_DEFAULT_SLA, mcu_contract()),
      ticks(std::move(observations)),
      order(policy),
      start(at) {
  prepare(mcu, dsp);
  runtime = std::make_unique<McuDspRuntime>(settings(policy), mcu, dsp, events());
  require(runtime->end_ticks() == initial_time &&
              dsp.clock().hardware_reset_ticks() == initial_time,
          "Actual five MCU reset-vector periods preserve DSP hold from zero");
  dsp.bus().gpio(DspPort::c).write({.reg = DspGpioRegister::control, .value = pins});
  drive();
  if (at == initial_time) { runtime->release_dsp_reset(); }
  attach();
}

void Fixture::restore(const McuDspRuntimeCheckpoint &saved, const SourceState source) {
  runtime.reset();
  cursor = source.cursor;
  levels = source.levels;
  trace.clear();
  runtime = std::make_unique<McuDspRuntime>(saved, mcu, dsp, events());
  attach();
}

Fixture::~Fixture() {
  mcu.set_cycle_observer({});
  mcu.bus().set_observer({});
}

void Fixture::drive() {
  dsp.bus().gpio(DspPort::c).set_drive({.mask = pins, .levels = levels});
}

DspRuntimeEvents Fixture::events() {
  return {.identity = identity,
          .next_ticks = [this]() -> std::optional<std::uint64_t> {
            return cursor < ticks.size() ? std::optional{ticks.at(cursor)} : std::nullopt;
          },
          .observe =
              [this](const std::uint64_t at) {
                require(cursor < ticks.size() && ticks.at(cursor) == at,
                        "External source retains actual cursor and timestamp");
                levels = (levels & ~2U) | (cursor == 0 ? 2U : 0U);
                drive();
                trace.push_back({.actor = 'E', .time = at});
                ++cursor;
                if (after_event) { after_event(); }
              },
          .order = order.peers,
          .maximum_events_per_period = 64};
}

void Fixture::attach() {
  const auto observed = [this](const DspRuntimeCycle &cycle) {
    constexpr unsigned recovery_clocks = 3;
    require(cycle.completion_ticks - cycle.start_ticks == period &&
                cycle.completion_ticks == start + ((cycle.cycle + 1 + recovery_clocks) * period),
            "Actual native periods retain independent held/recovery/EXTAL timing");
    trace.push_back({.actor = 'D', .time = cycle.completion_ticks});
  };
  mcu.set_cycle_observer([this, observed](const auto cycle) {
    runtime->begin_cycle(cycle, cycle.kind == 'w' && cycle.address == 1, cycle_inputs, observed);
  });
  mcu.bus().set_observer([this, observed](const auto &event) {
    if (!event.write || event.address != 1) { return; }
    runtime->observe_write(
        event,
        [this] {
          const auto pads = mcu.bus().observe_pads(McuPort::b, byte_mask);
          require(pads.known == byte_mask && pads.levels == 1,
                  "Actual native STAB precedes its explicit laboratory pin connection");
          levels |= 1U;
          drive();
          trace.push_back({.actor = 'M', .time = runtime->end_ticks()});
        },
        cycle_inputs,
        observed);
  });
}

void Fixture::step(const unsigned instructions) {
  for (unsigned i = 0; i < instructions; ++i) {
    require(mcu.cpu().step().instruction_executed, "Actual native MCU instruction executes");
    runtime->end_instruction();
  }
}
} // namespace blofeld::test::joint_events
