// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/dsp_scheduled_run.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_dsp_schedule.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/mcu_run_clock.hpp"
#include "dsp_external_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace blofeld::test::external {
namespace {
constexpr unsigned byte_mask = 0xff;
constexpr unsigned reset_ticks = 1200;
constexpr unsigned dsp_epoch = reset_ticks + 1;
constexpr unsigned early_event = 1325;
constexpr unsigned write_tick = 1920;
constexpr unsigned write_value = 0x42;
constexpr unsigned factor = 24;

McuFlashImage image(const std::span<const std::uint8_t> code) {
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  constexpr unsigned nop = 0xa7;
  constexpr unsigned protection = 0x7f0d;
  constexpr unsigned security = 0x7f0f;
  constexpr unsigned unsecured = 0xfe;
  bytes.fill(nop);
  for (unsigned index = 0; index < code.size(); ++index) { bytes.at(index) = code[index]; }
  bytes.at(protection) = byte_mask;
  bytes.at(security) = unsecured;
  bytes.at(bytes.size() - 2) = 0x80;
  bytes.back() = 0;
  return McuFlashImage::from_known(bytes);
}

McuRunContract mcu_contract() {
  return {.clock = {.oscillator_hz = extal,
                    .initial_oscillator_phase_ticks = 0,
                    .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = McuRunMemory::unstretched_internal};
}

McuDspScheduleInputs inputs(const McuDspTieOrder order) {
  return {.dsp = {.epoch_ticks = dsp_epoch,
                  .observation = DspStepObservation::reference_completion_boundary},
          .shared_write_order = order,
          .maximum_dsp_steps_per_cycle = 16,
          .wait_order = McuDspWaitOrder::dsp_before_mcu};
}

void shared_write(const McuDspTieOrder order) {
  // Independent HCS12 LDAB #42 (one period), STAB direct PORTB (two).
  constexpr std::array<std::uint8_t, 4> code{0xc6, write_value, 0x5b, 1};
  McuMachine mcu(image(code), BLOFELD_DEFAULT_SLA, mcu_contract());
  mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
  mcu.bus().write(hc12::portb, 0);
  mcu.bus().write(hc12::ddrb, byte_mask);
  DspMachine dsp(contract());
  initialize(dsp);
  McuDspSchedule schedule(inputs(order), mcu, dsp);
  unsigned cursor = 0;
  std::vector<char> ledger;
  const std::array<std::uint64_t, 2> ticks{early_event, write_tick};
  const DspExternalEvents source{
      .next_ticks = [&]() -> std::optional<std::uint64_t> {
        return cursor < ticks.size() ? std::optional{ticks.at(cursor)} : std::nullopt;
      },
      .observe =
          [&](const std::uint64_t now) {
            const auto pads = mcu.bus().observe_pads(McuPort::b, byte_mask);
            const auto expected =
                now == write_tick && order == McuDspTieOrder::mcu_before_dsp ? write_value : 0U;
            require(
                pads.known == byte_mask && pads.levels == expected &&
                    dsp.run_clock().elapsed_ticks() == (now - dsp_epoch) / unit_ticks * unit_ticks,
                "The external-only MCU tie observes the actual native write in the declared order");
            ledger.push_back('E');
            ++cursor;
          },
      .order = DspExternalOrder::reject_tie,
      .maximum_events = 4};
  const auto attach = [&] {
    mcu.set_cycle_observer([&](const auto cycle) {
      schedule.begin_cycle(
          cycle, cycle.kind == 'w' && cycle.address == hc12::portb, {}, {}, source);
    });
    mcu.bus().set_observer([&](const auto &event) {
      if (event.write && event.address == hc12::portb) {
        schedule.observe_write(event, [&] { ledger.push_back('W'); }, {}, {}, source);
      }
    });
  };
  attach();
  static_cast<void>(mcu.cpu().step());
  schedule.end_instruction();
  require(cursor == 1 && ledger == std::vector<char>{'E'} && schedule.completed_steps() == 1,
          "An external event inside the first real MCU period follows the earlier DSP completion");
  const auto saved_mcu = mcu.checkpoint();
  const auto saved_dsp = dsp.checkpoint();
  const auto saved_schedule = schedule.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    mcu.restore(saved_mcu);
    dsp.restore(saved_dsp);
    schedule.restore(saved_schedule);
    cursor = 1;
    ledger.clear();
    attach();
    if (order == McuDspTieOrder::unqualified) {
      const auto error = rejects([&] { static_cast<void>(mcu.cpu().step()); });
      require(error.contains("simultaneous external event and MCU shared write") &&
                  error.contains("mcu-pc=") && error.contains("dsp-pc=") && cursor == 1 &&
                  schedule.completed_steps() == 5 && schedule.faulted(),
              "An external-only MCU tie rejects visibly before the native shared store");
    } else {
      static_cast<void>(mcu.cpu().step());
      schedule.end_instruction();
      const std::vector<char> expected = order == McuDspTieOrder::dsp_before_mcu
                                             ? std::vector<char>{'E', 'W'}
                                             : std::vector<char>{'W', 'E'};
      require(ledger == expected && cursor == 2 && schedule.ties() == 1 &&
                  schedule.completed_steps() == 5 &&
                  mcu.run_clock().elapsed_ticks() * factor == write_tick &&
                  dsp.run_clock().elapsed_ticks() == 5 * unit_ticks,
              "Native MCU store, external clock and unfinished DSP instruction replay exactly");
    }
  }
  mcu.set_cycle_observer({});
  mcu.bus().set_observer({});
}

void wait_events(const bool cancel) {
  // Own LDS #2000 (two), ANDCC #ef (one), WAI (seven native periods).
  constexpr std::array<std::uint8_t, 7> code{0xcf, 0x20, 0, 0x10, 0xef, 0x3e, 0xa7};
  McuMachine mcu(image(code), BLOFELD_DEFAULT_SLA, mcu_contract());
  mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
  DspMachine dsp(contract());
  initialize(dsp);
  McuDspSchedule schedule(inputs(McuDspTieOrder::dsp_before_mcu), mcu, dsp);
  constexpr unsigned first_edge = 3701;
  constexpr unsigned second_edge = first_edge + 1;
  constexpr unsigned units_at_event = 20;
  constexpr unsigned steps_before_wait = 19;
  constexpr unsigned steps_after_period = 21;
  unsigned cursor = 0;
  unsigned periods = 0;
  const DspExternalEvents source{
      .next_ticks = [&]() -> std::optional<std::uint64_t> {
        if (cursor == 0) { return first_edge; }
        if (cursor == 1) { return second_edge; }
        return std::nullopt;
      },
      .observe =
          [&](const std::uint64_t now) {
            require(now == first_edge + cursor && mcu.idle_bus_cycles() == 0 &&
                        dsp.run_clock().elapsed_ticks() == units_at_event * unit_ticks,
                    "Two close external edges run inside the committed idle period before its "
                    "endpoint");
            mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 1});
            ++cursor;
          },
      .order = DspExternalOrder::reject_tie,
      .maximum_events = 4};
  const auto attach = [&] {
    mcu.set_cycle_observer(
        [&](const auto cycle) { schedule.begin_cycle(cycle, false, {}, {}, source); });
    mcu.set_wait_period_observer([&](const auto period) {
      ++periods;
      schedule.begin_wait_period(period, {}, {}, source);
    });
    if (cancel) {
      mcu.set_wait_observer([&](const auto /*cycle*/) {
        mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 1});
      });
    }
  };
  attach();
  for (unsigned i = 0; i < 3; ++i) {
    static_cast<void>(mcu.cpu().step());
    schedule.end_instruction();
  }
  constexpr unsigned before_wait = 3600;
  require(mcu.cpu().waiting() && schedule.end_ticks() == before_wait && cursor == 0,
          "Independent native setup reaches WAI before either external clock event");
  const auto saved_mcu = mcu.checkpoint();
  const auto saved_dsp = dsp.checkpoint();
  const auto saved_schedule = schedule.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    mcu.restore(saved_mcu);
    dsp.restore(saved_dsp);
    schedule.restore(saved_schedule);
    cursor = 0;
    periods = 0;
    attach();
    const unsigned expected_periods = cancel ? 0U : 1U;
    require(mcu.advance_wait(8) == expected_periods, "IRQ delivery ends the next WAI boundary");
    schedule.end_wait();
    require(
        periods == expected_periods && cursor == (cancel ? 0U : 2U) &&
            mcu.idle_bus_cycles() == expected_periods &&
            schedule.end_ticks() == before_wait + expected_periods * (2 * unit_ticks) &&
            schedule.completed_steps() == (cancel ? steps_before_wait : steps_after_period) &&
            mcu.cpu().pending_interrupt(),
        "Cancelled offers execute no speculative events; committed offers consume one real period");
  }
  mcu.set_cycle_observer({});
  mcu.set_wait_period_observer({});
  mcu.set_wait_observer({});
}
}

void paired_events() {
  shared_write(McuDspTieOrder::unqualified);
  shared_write(McuDspTieOrder::dsp_before_mcu);
  shared_write(McuDspTieOrder::mcu_before_dsp);
  wait_events(false);
  wait_events(true);
}
}
