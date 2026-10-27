// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_fetch_schedule.hpp"
#include "board/mcu_dsp_schedule.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_fetch_schedule.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned mcu_hz = 18000000, dsp_hz = 12000000, common = 540000000, entry = 0x8000;
constexpr unsigned byte_mask = 0xff, u_data = 0x0278, u_direction = 0x027a;
constexpr std::uint64_t epoch = 300, release = 480, recovery = 570, end = 660;
constexpr std::uint64_t recovery_duration = 90, reset_before_edge = 269, total_reset = 270;
constexpr std::uint64_t trailing_end = 540, trailing_remaining = 30, total_elapsed = 360;

struct Event {
  char actor;
  std::uint64_t ticks;
  bool operator==(const Event &) const = default;
};

McuFlashImage image() {
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  constexpr unsigned nop = 0xa7;
  constexpr unsigned protection = 0x7f0d;
  constexpr unsigned security = 0x7f0f;
  constexpr unsigned unsecured = 0xfe;
  // Own LDAB #1;STAB extended PORTU. Literal1+3 native periods.
  constexpr std::array<std::uint8_t, 5> code{0xc6, 1, 0x7b, 2, 0x78};
  bytes.fill(nop);
  for (unsigned i = 0; i < code.size(); ++i) { bytes.at(i) = code[i]; }
  bytes[protection] = byte_mask;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  return McuFlashImage::from_known(bytes);
}

McuRunContract mcu_inputs() {
  return {.clock = {.oscillator_hz = mcu_hz,
                    .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = McuRunMemory::unstretched_internal};
}

McuDspFetchInputs inputs(const McuExternalWriteOrder order) {
  return {.dsp = {.extal_hz = dsp_hz,
                  .timebase_hz = common,
                  .timing = DspFetchTiming::reference_cycle_order,
                  .execution = DspReferenceExecution::staged_register_boundaries,
                  .pipeline = DspFetchPipeline::automatic_memory,
                  .resets = DspFetchResetTiming::reference_cycle_boundary},
          .epoch_ticks = epoch,
          .shared_write_order = McuDspTieOrder::dsp_before_mcu,
          .maximum_dsp_cycles_per_period = 16,
          .external_write_order = order,
          .wait_order = McuDspWaitOrder::dsp_before_mcu,
          .reset_order = McuDspResetOrder::dsp_before_mcu,
          .initial_reset = McuDspFetchInitialReset{
              .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}};
}

DspMachine &prepare(McuMachine &mcu, DspMachine &dsp) {
  mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
  mcu.bus().write(u_data, 0);
  mcu.bus().write(u_direction, 1);
  DspProgram code;
  constexpr unsigned nops = 128;
  for (unsigned i = 0; i < nops; ++i) { code.emit("nop"); }
  code.install(dsp.bus());
  dsp.bus().pll().observe_pinit(false);
  dsp.cpu().reset_spi_boot();
  static_cast<void>(
      dsp.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return dsp;
}

struct Fixture {
  Fixture(McuExternalWriteOrder writes, DspExternalOrder clocks)
      : write_order(writes),
        clock_order(clocks),
        schedule(inputs(writes), mcu, prepare(mcu, dsp), events()) {
    attach();
  }

  ~Fixture() {
    mcu.set_cycle_observer({});
    mcu.bus().set_observer({});
  }

  Fixture(const Fixture &) = delete;
  Fixture &operator=(const Fixture &) = delete;

  DspFetchExternalEvents events() {
    return {.identity = identity,
            .next_ticks = [this]() -> std::optional<std::uint64_t> {
              if (cursor == 0) { return release; }
              if (cursor == 1) { return recovery; }
              return std::nullopt;
            },
            .observe =
                [this](const std::uint64_t at) {
                  require(schedule.run().completed_cycles() == 0 &&
                              at == (cursor == 0 ? release : recovery),
                          "Reset pin events never substitute native DSP work");
                  const auto remaining = schedule.run().hardware_reset_remaining_ticks();
                  if (cursor == 0) {
                    require(schedule.run().hardware_reset_active() &&
                                remaining.has_value() ==
                                    (write_order == McuExternalWriteOrder::mcu_before_external),
                            "Exact release-time input observes asserted or recovering reset in its "
                            "declared MCU order");
                    if (remaining) {
                      require(*remaining == recovery_duration,
                              "Release starts literal two-EXTAL recovery");
                    }
                  } else {
                    const bool before = clock_order == DspExternalOrder::external_before_dsp;
                    require(schedule.run().hardware_reset_active() == before &&
                                schedule.run().hardware_reset_ticks() ==
                                    (before ? reset_before_edge : total_reset),
                            "An exact recovery-time input observes the excluded or completed real "
                            "reset edge");
                    if (before) {
                      require(
                          remaining == 1,
                          "The excluded recovery edge retains precisely one EXTAL timebase tick");
                    }
                  }
                  dsp.bus().timers().set_tio_drive(0, cursor == 0);
                  trace.push_back({.actor = 'E', .ticks = at});
                  ++cursor;
                },
            .order = clock_order,
            .maximum_events = 2};
  }

  void attach() {
    const auto observed = [this](const DspFetchCycleResult &cycle) {
      trace.push_back({.actor = 'D', .ticks = epoch + cycle.completion_ticks});
    };
    mcu.set_cycle_observer([this, observed](const auto cycle) {
      schedule.begin_cycle(
          cycle,
          cycle.kind == 'w' && cycle.address == u_data,
          [] { return fetch_clock::input(); },
          observed);
    });
    mcu.bus().set_observer([this, observed](const auto &event) {
      if (!event.write || event.address != u_data) { return; }
      schedule.observe_write(
          event,
          [this] {
            const auto pads = mcu.bus().gpio_outputs(McuPort::u);
            require((pads.mask & pads.known & pads.levels & 1U) != 0,
                    "Actual native PORTU output supplies the declared laboratory reset release");
            trace.push_back({.actor = 'M', .ticks = schedule.end_ticks()});
            schedule.release_dsp_reset();
          },
          [] { return fetch_clock::input(); },
          observed);
    });
  }

  void step(unsigned count) {
    for (unsigned i = 0; i < count; ++i) {
      require(mcu.cpu().step().instruction_executed,
              "Actual native MCU instructions advance reset control");
      schedule.end_instruction();
    }
  }

  McuMachine mcu{image(), BLOFELD_DEFAULT_SLA, mcu_inputs()};
  DspMachine dsp;
  std::shared_ptr<const DspFetchEventIdentity> identity =
      std::make_shared<const DspFetchEventIdentity>();
  unsigned cursor = 0;
  const McuExternalWriteOrder write_order;
  const DspExternalOrder clock_order;
  std::vector<Event> trace;
  McuDspFetchSchedule schedule;
};

void run(const McuExternalWriteOrder writes, const DspExternalOrder clocks) {
  Fixture f(writes, clocks);
  const auto native = f.dsp.cpu().debug_state();
  const auto saved = f.schedule.checkpoint();
  std::vector<Event> suffix;
  for (unsigned replay = 0; replay < 2; ++replay) {
    f.schedule.restore(saved);
    f.cursor = 0;
    f.attach();
    f.trace.clear();
    f.step(2);
    require(f.schedule.end_ticks() == trailing_end && f.schedule.run().hardware_reset_active() &&
                f.schedule.run().hardware_reset_remaining_ticks() == trailing_remaining &&
                f.cursor == 1 && f.schedule.run().completed_cycles() == 0,
            "Real trailing MCU slot consumes recovery time without running native DSP periods");
    f.step(2);
    require(
        f.schedule.end_ticks() == end && f.mcu.run_clock().bus_cycles() == 11 &&
            f.mcu.cpu().state().pc == entry + 7 && f.mcu.cpu().state().d == 1 && f.cursor == 2 &&
            !f.schedule.run().hardware_reset_active() &&
            f.schedule.run().hardware_reset_ticks() == total_reset &&
            f.schedule.run().elapsed_ticks() == total_elapsed &&
            f.schedule.run().completed_cycles() == 2 && f.schedule.run().timer_clk2_edges() == 1 &&
            f.dsp.cpu().debug_state() == native,
        "Independent60tick MCU,45tick DSP and90tick recovery ledgers agree after native restart");
    std::vector<char> tied;
    for (const auto event : f.trace) {
      if (event.ticks == release) { tied.push_back(event.actor); }
    }
    require(tied == (writes == McuExternalWriteOrder::external_before_mcu
                         ? std::vector<char>{'E', 'M'}
                         : std::vector<char>{'M', 'E'}),
            "Reset release preserves its explicit MCU/external order exactly once");
    if (replay == 0) { suffix = f.trace; }
    require(f.trace == suffix,
            "Trusted held snapshot and external cursor retain the complete reset suffix");
  }
}
}

int main() {
  try {
    for (const auto writes :
         {McuExternalWriteOrder::external_before_mcu, McuExternalWriteOrder::mcu_before_external}) {
      for (const auto clocks :
           {DspExternalOrder::external_before_dsp, DspExternalOrder::dsp_before_external}) {
        run(writes, clocks);
      }
    }
    std::cout << "Timed native MCU/DSP reset event ordering passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
