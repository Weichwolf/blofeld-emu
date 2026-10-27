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
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned oscillator = 12000000, common = 540000000, entry = 0x8000, handler = 0x9000;
constexpr unsigned flags = 0xc0, stack = 0x2000, wait_stack = 0x1ff7, vector = 0xfff2,
                   byte_mask = 0xff;
constexpr std::uint64_t epoch = 450, period = 45, waiting = 1350, irq = 1395, idle_end = 1440,
                        wake_end = 1980;
constexpr std::uint64_t deassert = 2000, final_end = 2700;
constexpr unsigned idle_native_periods = 22, final_mcu_periods = 30, final_native_periods = 50,
                   final_timer_edges = 25;

struct Event {
  char actor;
  std::uint64_t ticks;
  bool operator==(const Event &) const = default;
};

McuFlashImage image() {
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  constexpr unsigned nop = 0xa7;
  // Own LDS #$2000;ANDCC #$ef;WAI;NOP. Literal2+1+7 native periods.
  constexpr std::array<std::uint8_t, 7> code{0xcf, 0x20, 0, 0x10, 0xef, 0x3e, nop};
  bytes.fill(nop);
  for (unsigned i = 0; i < code.size(); ++i) { bytes.at(i) = code[i]; }
  constexpr unsigned rti = 0x0b;
  constexpr unsigned protection = 0x7f0d;
  constexpr unsigned security = 0x7f0f;
  constexpr unsigned unsecured = 0xfe;
  bytes[handler - entry] = rti;
  bytes[vector - entry] = handler >> 8U;
  bytes[vector - entry + 1] = 0;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  bytes[protection] = byte_mask;
  bytes[security] = unsecured;
  return McuFlashImage::from_known(bytes);
}

McuRunContract mcu_inputs() {
  return {.clock = {.oscillator_hz = oscillator,
                    .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = McuRunMemory::unstretched_internal};
}

McuDspFetchInputs inputs() {
  return {.dsp = {.extal_hz = oscillator,
                  .timebase_hz = common,
                  .timing = DspFetchTiming::reference_cycle_order,
                  .execution = DspReferenceExecution::staged_register_boundaries,
                  .pipeline = DspFetchPipeline::automatic_memory},
          .epoch_ticks = epoch,
          .shared_write_order = McuDspTieOrder::dsp_before_mcu,
          .maximum_dsp_cycles_per_period = 16,
          .wait_order = McuDspWaitOrder::dsp_before_mcu,
          .reset_order = McuDspResetOrder::dsp_before_mcu};
}

DspMachine &prepare(DspMachine &dsp) {
  DspProgram program;
  constexpr unsigned nops = 128;
  for (unsigned i = 0; i < nops; ++i) { program.emit("nop"); }
  program.install(dsp.bus());
  dsp.bus().pll().observe_pinit(false);
  dsp.cpu().reset_spi_boot();
  static_cast<void>(
      dsp.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return dsp;
}

struct Fixture {
  explicit Fixture(DspExternalOrder policy)
      : order(policy), schedule(inputs(), mcu, prepare(dsp), events()) {
    mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
    attach();
  }

  ~Fixture() {
    mcu.set_cycle_observer({});
    mcu.set_wait_period_observer({});
  }

  Fixture(const Fixture &) = delete;
  Fixture &operator=(const Fixture &) = delete;

  DspFetchExternalEvents events() {
    return {.identity = identity,
            .next_ticks = [this]() -> std::optional<std::uint64_t> {
              if (cursor == 0) { return irq; }
              if (cursor == 1) { return deassert; }
              return std::nullopt;
            },
            .observe =
                [this](const std::uint64_t at) {
                  require(at == (cursor == 0 ? irq : deassert),
                          "Actual IRQ input time and cursor match");
                  mcu.bus().set_pin_drive(
                      McuPort::e,
                      {.mask = 3, .levels = static_cast<std::uint8_t>(cursor == 0 ? 1U : 3U)});
                  trace.push_back({.actor = 'E', .ticks = at});
                  ++cursor;
                },
            .order = order,
            .maximum_events = 2};
  }

  void attach() {
    const auto observed = [this](const DspFetchCycleResult &cycle) {
      require(cycle.period_ticks == period &&
                  cycle.completion_ticks == cycle.completed_cycles * period,
              "Native DSP period census retains independent bypass timing");
      trace.push_back({.actor = 'D', .ticks = epoch + cycle.completion_ticks});
    };
    mcu.set_cycle_observer([this, observed](const auto cycle) {
      schedule.begin_cycle(cycle, false, [] { return fetch_clock::input(); }, observed);
    });
    mcu.set_wait_period_observer([this, observed](const auto slice) {
      schedule.begin_wait_period(slice, [] { return fetch_clock::input(); }, observed);
    });
  }

  CpuStep step() {
    const auto result = mcu.cpu().step();
    schedule.end_instruction();
    return result;
  }

  void frame() {
    constexpr std::array<unsigned, 9> bytes{flags, 0, 0, 0, 0, 0, 0, 0x80, 6};
    for (unsigned i = 0; i < bytes.size(); ++i) {
      require(mcu.bus().read(static_cast<std::uint16_t>(wait_stack + i)) == bytes[i],
              "Literal native WAI frame survives timed IRQ input");
    }
  }

  McuMachine mcu{image(), BLOFELD_DEFAULT_SLA, mcu_inputs()};
  DspMachine dsp;
  std::shared_ptr<const DspFetchEventIdentity> identity =
      std::make_shared<const DspFetchEventIdentity>();
  unsigned cursor = 0;
  const DspExternalOrder order;
  std::vector<Event> trace;
  McuDspFetchSchedule schedule;
};

void run(const DspExternalOrder order) {
  Fixture f(order);
  auto expected = f.dsp.cpu().debug_state();
  for (unsigned i = 0; i < 3; ++i) {
    require(f.step().instruction_executed, "Native setup reaches actual WAI");
  }
  require(f.mcu.cpu().waiting() && f.mcu.cpu().state().pc == entry + 6 &&
              f.mcu.cpu().state().sp == wait_stack && f.schedule.end_ticks() == waiting &&
              f.mcu.run_clock().bus_cycles() == 15 && f.cursor == 0,
          "Independent2+1+7 instruction periods and genuine reset establish the WAI cut");
  f.frame();
  const auto saved = f.schedule.checkpoint();
  // A seven-stage NOP stream publishes44 results in50 genuine periods.
  constexpr unsigned native_results = 44;
  expected.cpu.pc = boot + native_results;
  expected.native_instructions += native_results;
  expected.native_cycles += native_results;
  std::vector<Event> suffix;
  for (unsigned replay = 0; replay < 2; ++replay) {
    f.schedule.restore(saved);
    f.cursor = 0;
    f.attach();
    const auto first = f.trace.size();
    require(f.mcu.advance_wait(10) == 1, "Timed IRQ stops after its already committed idle period");
    f.schedule.end_wait();
    f.frame();
    require(f.cursor == 1 && f.mcu.cpu().waiting() && f.mcu.cpu().pending_interrupt() &&
                f.mcu.idle_bus_cycles() == 1 && f.mcu.run_clock().bus_cycles() == 16 &&
                f.schedule.end_ticks() == idle_end &&
                f.schedule.run().completed_cycles() == idle_native_periods,
            "Asynchronous input preserves real idle completion and continuing native DSP clocks");
    const auto wake = f.step();
    require(!wake.instruction_executed && wake.stack_reused && wake.interrupt_vector == vector &&
                f.mcu.cpu().state().sp == wait_stack && f.mcu.cpu().state().pc == handler &&
                f.schedule.end_ticks() == wake_end,
            "Native six-period IRQ wake reuses the genuine WAI frame");
    f.frame();
    require(f.step().instruction_executed, "Actual eight-period RTI executes");
    const auto cpu = f.mcu.cpu().state();
    require(cpu.pc == entry + 6 && cpu.sp == stack && cpu.d == 0 && cpu.x == 0 && cpu.y == 0 &&
                cpu.ccr == flags && !f.mcu.cpu().waiting() && !f.mcu.cpu().pending_interrupt() &&
                f.mcu.run_clock().bus_cycles() == final_mcu_periods &&
                f.schedule.end_ticks() == final_end &&
                f.schedule.run().completed_cycles() == final_native_periods &&
                f.schedule.run().timer_clk2_edges() == final_timer_edges && f.cursor == 2 &&
                f.dsp.cpu().debug_state() == expected,
            "Literal CPU/stack/flags/time endpoints and actual native NOP publications agree");
    std::vector<Event> tied;
    for (const auto event : f.trace) {
      if (event.ticks == irq) { tied.push_back(event); }
    }
    const std::vector<Event> wanted =
        order == DspExternalOrder::external_before_dsp
            ? std::vector<Event>{{.actor = 'E', .ticks = irq}, {.actor = 'D', .ticks = irq}}
            : std::vector<Event>{{.actor = 'D', .ticks = irq}, {.actor = 'E', .ticks = irq}};
    require(tied == wanted,
            "Exact IRQ/DSP tie retains declared ordering inside the committed idle period");
    const std::vector<Event> actual(f.trace.begin() + static_cast<std::ptrdiff_t>(first),
                                    f.trace.end());
    if (replay == 0) { suffix = actual; }
    require(actual == suffix,
            "Joint idle/native/cursor restore retains the complete replay suffix");
    f.trace.resize(first);
  }
}
}

int main() {
  try {
    run(DspExternalOrder::external_before_dsp);
    run(DspExternalOrder::dsp_before_external);
    std::cout << "Timed native WAI/IRQ/RTI event ordering passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
