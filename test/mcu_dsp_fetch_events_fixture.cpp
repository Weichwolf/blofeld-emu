// SPDX-License-Identifier: GPL-3.0-only
#include "mcu_dsp_fetch_events_fixture.hpp"

#include "board/mcu_dsp_fetch_schedule.hpp"
#include "board/mcu_dsp_schedule.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_fetch_schedule.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_memory_pipeline_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
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

McuDspFetchInputs settings(const Case order, const std::uint64_t start) {
  return {.dsp = {.extal_hz = oscillator,
                  .timebase_hz = common,
                  .timing = DspFetchTiming::reference_cycle_order,
                  .execution = DspReferenceExecution::staged_register_boundaries,
                  .pipeline = DspFetchPipeline::automatic_memory,
                  .resets = DspFetchResetTiming::reference_cycle_boundary},
          .epoch_ticks = start,
          .shared_write_order = order.native,
          .maximum_dsp_cycles_per_period = 16,
          .external_write_order = order.external,
          .wait_order = McuDspWaitOrder::dsp_before_mcu,
          .reset_order = McuDspResetOrder::dsp_before_mcu};
}

DspMachine &prepare(McuMachine &mcu, DspMachine &dsp) {
  mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
  mcu.bus().write(hc12::portb, 0);
  mcu.bus().write(hc12::ddrb, byte_mask);
  mcu.bus().write(marker_address, marker);
  const std::array<std::string, 2> code{"movep x:<<$ffffbd,x0", "movep x:<<$ffffbd,y0"};
  memory_pipeline::prepare(dsp, code);
  dsp.bus().gpio(DspPort::c).write({.reg = DspGpioRegister::control, .value = pins});
  dsp.bus().gpio(DspPort::c).set_drive({.mask = pins, .levels = 0});
  return dsp;
}

Fixture::Fixture(const Case policy, const std::uint64_t at, std::vector<std::uint64_t> observations)
    : mcu(image(), BLOFELD_DEFAULT_SLA, mcu_contract()),
      ticks(std::move(observations)),
      order(policy),
      start(at),
      schedule(settings(policy, at), mcu, prepare(mcu, dsp), events()) {
  attach();
}

Fixture::~Fixture() {
  mcu.set_cycle_observer({});
  mcu.bus().set_observer({});
}

void Fixture::drive() {
  dsp.bus().gpio(DspPort::c).set_drive({.mask = pins, .levels = levels});
}

DspFetchExternalEvents Fixture::events() {
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
          .maximum_events = 64};
}

void Fixture::attach() {
  const auto input = [] { return fetch_clock::input(); };
  const auto observed = [this](const DspFetchCycleResult &cycle) {
    require(cycle.period_ticks == period &&
                cycle.completion_ticks == cycle.completed_cycles * period,
            "Actual native periods retain independent bypass timing");
    trace.push_back({.actor = 'D', .time = start + cycle.completion_ticks});
  };
  mcu.set_cycle_observer([this, input, observed](const auto cycle) {
    schedule.begin_cycle(cycle, cycle.kind == 'w' && cycle.address == 1, input, observed);
  });
  mcu.bus().set_observer([this, input, observed](const auto &event) {
    if (!event.write || event.address != 1) { return; }
    schedule.observe_write(
        event,
        [this] {
          const auto pads = mcu.bus().observe_pads(McuPort::b, byte_mask);
          require(pads.known == byte_mask && pads.levels == 1,
                  "Actual native STAB precedes its explicit laboratory pin connection");
          levels |= 1U;
          drive();
          trace.push_back({.actor = 'M', .time = schedule.end_ticks()});
        },
        input,
        observed);
  });
}

void Fixture::step(const unsigned instructions) {
  for (unsigned i = 0; i < instructions; ++i) {
    require(mcu.cpu().step().instruction_executed, "Actual native MCU instruction executes");
    schedule.end_instruction();
  }
}
} // namespace blofeld::test::joint_events
