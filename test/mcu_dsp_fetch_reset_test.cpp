// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_fetch_schedule.hpp"
#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_cop.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <tuple>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned oscillator_hz = 12000000, common_hz = 540000000;
constexpr unsigned epoch = 450, dsp_period = 45, common_factor = 9;
constexpr unsigned entry = 0x8000, cop_handler = 0x8100, ram = 0x1200, marker = 0x5a;
constexpr unsigned byte_mask = 0xff, nop = 0xa7, protection = 0x7f0d, security = 0x7f0f;
constexpr unsigned cop_edges = 16384, cop_ticks = 81920, reset_edges = 195;
constexpr unsigned unsecured = 0xfe, branch_self = 0xfe, vector_ticks = 50, wait_ticks = 140;
constexpr unsigned branch_back = 0x050fdf;
constexpr std::uint64_t accumulator_unit = std::uint64_t{1} << 24U;

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected pipeline COP/reset rejection");
}

McuFlashImage image(const bool waiting) {
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  bytes.fill(nop);
  if (waiting) {
    // Own LDS #2000;WAI. Literal2+7entry periods;all nine frame bytes fit SRAM.
    constexpr std::array<std::uint8_t, 4> code{0xcf, 0x20, 0, 0x3e};
    for (unsigned i = 0; i < code.size(); ++i) { bytes.at(i) = code[i]; }
  } else {
    // Own BRA -2,exactly three unstretched periods per completed native branch.
    bytes[0] = 0x20;
    bytes[1] = branch_self;
  }
  bytes[protection] = byte_mask;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  bytes[bytes.size() - 6] = cop_handler >> 8U;
  bytes[bytes.size() - 5] = 0;
  return McuFlashImage::from_known(bytes);
}

McuRunContract mcu_inputs(const unsigned phase) {
  return {.clock = {.oscillator_hz = oscillator_hz,
                    .initial_oscillator_phase_ticks = phase,
                    .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = McuRunMemory::unstretched_internal};
}

McuDspFetchInputs inputs(const McuDspResetOrder reset) {
  return {.dsp = {.extal_hz = oscillator_hz,
                  .timebase_hz = common_hz,
                  .timing = DspFetchTiming::reference_cycle_order,
                  .execution = DspReferenceExecution::staged_register_boundaries,
                  .pipeline = DspFetchPipeline::automatic_memory},
          .epoch_ticks = epoch,
          .shared_write_order = McuDspTieOrder::dsp_before_mcu,
          .maximum_dsp_cycles_per_period = 16,
          .wait_order = McuDspWaitOrder::dsp_before_mcu,
          .reset_order = reset};
}

DspMachine &initialize(DspMachine &dsp, const bool failing) {
  DspProgram code;
  if (failing) {
    code.immediate("r0", 0x2000);
    code.immediate("x0", 7);
    code.emit("move x0,y:(r0)");
  } else {
    code.immediate("x1", 1);
    code.emit("clr a");
    code.emit("add x1,a");
    code.word(branch_back);
  }
  constexpr unsigned padding = 32;
  for (unsigned i = 0; i < padding; ++i) { code.emit("nop"); }
  code.install(dsp.bus());
  dsp.bus().pll().observe_pinit(false);
  dsp.cpu().reset_spi_boot();
  static_cast<void>(
      dsp.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return dsp;
}

struct Fixture {
  explicit Fixture(const bool waiting = false,
                   const unsigned phase = 0,
                   const McuDspResetOrder reset = McuDspResetOrder::dsp_before_mcu,
                   const bool fault = false)
      : mcu(image(waiting), BLOFELD_DEFAULT_SLA, mcu_inputs(phase)),
        schedule(inputs(reset), mcu, initialize(dsp, fault)),
        failing(fault) {
    mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
    attach();
    if (waiting) {
      step();
      step();
      require(mcu.cpu().waiting() && mcu.run_clock().bus_cycles() == 14 &&
                  mcu.run_clock().elapsed_ticks() == wait_ticks,
              "Native LDS/WAI creates its actual frame before any idle COP interval");
    }
  }

  void attach() {
    mcu.set_cycle_observer([this](const auto period) {
      schedule.begin_cycle(
          period,
          false,
          [] { return fetch_clock::input(); },
          [this](const auto &result) { record(result); });
    });
    mcu.set_wait_period_observer([this](const auto period) {
      schedule.begin_wait_period(
          period,
          [] { return fetch_clock::input(); },
          [this](const auto &result) { record(result); });
    });
    mcu.set_reset_period_observer([this](const auto period) {
      schedule.begin_reset_period(
          period,
          [] { return fetch_clock::input(); },
          [this](const auto &result) { record(result); });
    });
  }

  void record(const DspFetchCycleResult &result) {
    require(result.completed_cycles == cycles + 1 && result.period_ticks == dsp_period &&
                result.completion_ticks == result.completed_cycles * dsp_period,
            "Each actual automatic pipeline period retains its literal45tick clock");
    ++cycles;
    if (!result.pipeline) { throw Fault("Actual automatic cycle must retain its pipeline result"); }
    const auto &native = *result.pipeline;
    if (failing || !native.retirement_published) { return; }
    require(native.execute2 && native.execute2->address == expected_pc,
            "Real native retirements retain the independent MOVE/CLR/ADD/BRA sequence");
    if (expected_pc == boot) {
      expected_pc = boot + 2;
    } else if (expected_pc == boot + 2) {
      require(dsp.cpu().state().a == 0, "Actual native CLR defines the zero accumulator");
      expected_pc = boot + 3;
    } else if (expected_pc == boot + 3) {
      ++adds;
      require(dsp.cpu().state().a == adds * accumulator_unit,
              "Every native ADD changes the full accumulator independently of MCU reset");
      expected_pc = boot + 4;
    } else {
      expected_pc = boot + 3;
    }
  }

  void step() {
    static_cast<void>(mcu.cpu().step());
    schedule.end_instruction();
  }

  void time() const {
    const auto common = mcu.run_clock().elapsed_ticks() * common_factor;
    require(schedule.end_ticks() == common && cycles == (common - epoch) / dsp_period &&
                schedule.run().completed_cycles() == cycles &&
                schedule.run().elapsed_ticks() == cycles * dsp_period &&
                schedule.run().timer_clk2_edges() == cycles / 2,
            "Actual MCU/pipeline/CLK2 ledgers equal the independent integer horizon");
  }

  void begin(const McuResetSource source, const unsigned sync = 3) {
    schedule.begin_system_reset(source, sync);
    attach(); // Native MCU reset replaces its owner and deliberately drops observers.
  }

  McuMachine mcu;
  DspMachine dsp;
  McuDspFetchSchedule schedule;
  std::uint64_t cycles = 0, adds = 0;
  unsigned expected_pc = boot;
  bool failing;
};

struct ResetScenario {
  unsigned sync, phase;
};

void external_reset(const ResetScenario scenario) {
  const unsigned sync = scenario.sync;
  const unsigned phase = scenario.phase;
  Fixture f(false, phase);
  f.step();
  f.mcu.bus().write(ram, marker);
  const auto start = f.mcu.run_clock().elapsed_ticks();
  const auto before = f.dsp.cpu().debug_state();
  const auto adds = f.adds;
  f.begin(McuResetSource::external, sync);
  require(f.dsp.cpu().debug_state() == before, "An explicit MCU reset does not reset the DSP");
  const unsigned edges = 128 + sync + 64;
  require(f.mcu.advance_system_reset(edges, McuResetPad::high) == edges,
          "All drive/synchronization/settling edges precede the real native reset vector");
  f.schedule.end_system_reset();
  const auto end = start + (edges * 5ULL) - phase + vector_ticks;
  require(f.mcu.run_clock().elapsed_ticks() == end && f.mcu.run_clock().native_epoch() == 2 &&
              f.mcu.run_clock().reset_sysclk_edges() == edges && f.mcu.cpu().state().pc == entry &&
              f.mcu.cpu().state().ccr == hc12::reset_ccr && f.mcu.bus().read(ram) == marker &&
              f.adds > adds && !f.schedule.faulted(),
          "Native external reset preserves SRAM and DSP work,with literal OSC/vector durations");
  f.time();
}

void cop_timeout(const bool waiting, const bool phy, const unsigned phase) {
  Fixture f(waiting, phase);
  if (phy) {
    constexpr unsigned enable_phy = 0x40;
    constexpr unsigned select_phy = 0x80;
    f.mcu.bus().write(hc12::usb_control, enable_phy);
    f.mcu.bus().observe_phy({.edges = 15});
    f.mcu.bus().write(hc12::clock_select, select_phy);
  }
  f.mcu.bus().write(hc12::cop_control, 1);
  const auto start = f.mcu.run_clock().elapsed_ticks();
  const auto bus = f.mcu.run_clock().bus_cycles();
  const auto idle = f.mcu.idle_bus_cycles();
  const auto duration = cop_ticks - phase;
  const auto period = phy ? 2U : 10U;
  const auto completed = (duration - 1) / period;
  const auto reason = rejects([&] {
    constexpr unsigned bound = cop_edges * 3;
    for (unsigned i = 0; i < bound; ++i) {
      if (waiting) {
        static_cast<void>(f.mcu.advance_wait(1));
        f.schedule.end_wait();
      } else {
        f.step();
      }
    }
  });
  require(reason.contains("COP system reset (timeout)") && f.schedule.faulted() &&
              !f.schedule.run().faulted() &&
              f.mcu.bus().cop_state().reset == McuCopReset::timeout &&
              f.mcu.bus().cop_state().phase == cop_edges &&
              f.mcu.run_clock().elapsed_ticks() == start + duration &&
              f.mcu.run_clock().bus_cycles() == bus + completed &&
              f.mcu.idle_bus_cycles() == idle + (waiting ? completed : 0),
          "Natural COP timeout preempts the real bus/idle slot at its exact fractional edge");
  f.time();
  const auto adds = f.adds;
  f.begin(McuResetSource::cop);
  require(f.mcu.advance_system_reset(reset_edges, McuResetPad::high) == reset_edges,
          "The actual latched timeout admits an explicit native COP reset");
  f.schedule.end_system_reset();
  require(f.mcu.cpu().state().pc == cop_handler && f.mcu.run_clock().native_epoch() == 2 &&
              f.mcu.run_clock().elapsed_ticks() ==
                  start + duration + reset_edges * 5ULL + vector_ticks &&
              f.adds > adds && !f.schedule.faulted(),
          "Actual FFFA COP vector differs from FFFE and preserves continuous native DSP work");
  f.time();
}

void unknown_pad() {
  Fixture f;
  f.begin(McuResetSource::external);
  f.mcu.advance_system_reset(reset_edges - 1, McuResetPad::unknown);
  const auto time = f.mcu.run_clock().elapsed_ticks();
  const auto cycles = f.cycles;
  require(
      rejects([&] { f.mcu.advance_system_reset(1, McuResetPad::unknown); }).contains("RESET pad") &&
          f.schedule.faulted() && f.cycles == cycles && f.mcu.run_clock().elapsed_ticks() == time,
      "Unknown source sampling cannot invent a reset edge or consume another DSP period");
}

void malformed_offer(const bool wrong_epoch) {
  Fixture f;
  f.begin(McuResetSource::external);
  const auto state = f.dsp.cpu().debug_state();
  const McuResetPeriod bad{.context = {},
                           .duration_ticks = wrong_epoch ? 5U : 4U,
                           .native_epoch = wrong_epoch ? 1U : 2U};
  require(rejects([&] {
            f.schedule.begin_reset_period(bad, [] { return fetch_clock::input(); });
          }).contains("owned SYSCLK duration and epoch") &&
              f.schedule.faulted() && f.cycles == 0 && f.dsp.cpu().debug_state() == state,
          "Wrong reset duration/epoch refuses before native DSP or MCU work");
}

void default_cop_refusal() {
  Fixture f(false, 0, McuDspResetOrder::unqualified);
  f.mcu.bus().write(hc12::cop_control, 1);
  const auto start = f.mcu.run_clock().elapsed_ticks();
  const auto reason = rejects([&] {
    for (unsigned i = 0; i < cop_edges; ++i) { f.step(); }
  });
  constexpr unsigned before_timeout = cop_ticks - 10;
  require(reason.contains("Unqualified MCU/DSP fetch COP deadline") && f.schedule.faulted() &&
              f.mcu.bus().cop_state().reset == McuCopReset::none &&
              f.mcu.bus().cop_state().phase == cop_edges - 2 &&
              f.mcu.run_clock().elapsed_ticks() == start + before_timeout,
          "Unqualified COP timing refuses before the timeout slot and cannot invent reset success");
  f.time();
}

void peer_failure() {
  Fixture f(false, 0, McuDspResetOrder::dsp_before_mcu, true);
  f.begin(McuResetSource::external);
  const auto reason =
      rejects([&] { f.mcu.advance_system_reset(reset_edges, McuResetPad::unknown); });
  require(reason.contains("Scheduled pipeline reset peer") && reason.contains("mcu-request-pc=") &&
              reason.contains("Y:0x002000") && f.schedule.faulted() && f.schedule.run().faulted() &&
              f.mcu.run_clock().reset_sysclk_edges() == f.cycles &&
              f.mcu.run_clock().elapsed_ticks() == vector_ticks + f.cycles * 5 &&
              f.schedule.run().elapsed_ticks() == f.cycles * dsp_period &&
              f.schedule.end_ticks() == epoch + ((f.cycles + 1) * dsp_period),
          "Real reserved-Y failure during reset preserves consumed peer/MCU prefix and context");
  require(rejects([&] { f.schedule.end_system_reset(); }).contains("healthy inactive") &&
              rejects([&] {
                static_cast<void>(f.schedule.run().checkpoint(f.dsp));
              }).contains("healthy"),
          "Failed reset execution and native capture require coordinated recovery");
}

void unconsumed_reset_offer() {
  Fixture f;
  f.begin(McuResetSource::external);
  const McuResetPeriod period{.context = {}, .duration_ticks = 5, .native_epoch = 2};
  f.schedule.begin_reset_period(period, [] { return fetch_clock::input(); });
  const auto reason = rejects([&] { f.mcu.advance_system_reset(1, McuResetPad::unknown); });
  require(
      reason.contains("every offered native MCU period") && f.schedule.faulted() &&
          f.schedule.run().completed_cycles() == 1 && f.mcu.run_clock().reset_sysclk_edges() == 0 &&
          f.mcu.run_clock().elapsed_ticks() == vector_ticks,
      "A missing consumed reset edge cannot silently clock a second DSP period:reason=" + reason +
          " dsp-cycles=" + std::to_string(f.schedule.run().completed_cycles()) +
          " reset-edges=" + std::to_string(f.mcu.run_clock().reset_sysclk_edges()) +
          " mcu-ticks=" + std::to_string(f.mcu.run_clock().elapsed_ticks()));
}

bool same_cpu(const CpuState &left, const CpuState &right) {
  return std::tie(left.pc, left.d, left.x, left.y, left.sp, left.ccr) ==
         std::tie(right.pc, right.d, right.x, right.y, right.sp, right.ccr);
}

void default_reset_refusal() {
  Fixture f(false, 0, McuDspResetOrder::unqualified);
  const auto mcu = f.mcu.cpu().state();
  const auto dsp = f.dsp.cpu().debug_state();
  require(rejects([&] { f.begin(McuResetSource::external); }).contains("explicit order") &&
              same_cpu(f.mcu.cpu().state(), mcu) && f.dsp.cpu().debug_state() == dsp,
          "Default unqualified reset contract cannot silently restart either processor");
}
}

int main(const int count, const char *const *const arguments) {
  try {
    require(count == 1 || count == 2, "Expected an optional isolated failure-case selector");
    if (count == 2) {
      require(std::string(arguments[1]) == "unconsumed", "Unknown isolated test selector");
      unconsumed_reset_offer();
      std::cout << "Unconsumed reset-offer control passed\n";
      return 0;
    }
    unsigned cases = 0;
    for (unsigned sync = 3; sync <= 6; ++sync) {
      external_reset({.sync = sync, .phase = sync - 3});
      ++cases;
    }
    for (unsigned phase = 0; phase < 5; ++phase) {
      for (const bool waiting : {false, true}) {
        for (const bool phy : {false, true}) {
          cop_timeout(waiting, phy, phase);
          ++cases;
        }
      }
    }
    for (const auto test : {unknown_pad,
                            default_reset_refusal,
                            default_cop_refusal,
                            peer_failure,
                            unconsumed_reset_offer}) {
      test();
      ++cases;
    }
    malformed_offer(false);
    ++cases;
    malformed_offer(true);
    ++cases;
    constexpr unsigned expected_cases = 31;
    require(cases == expected_cases, "Independent complete COP/reset case census");
    std::cout << "Pipeline COP/reset cases=" << cases
              << " whole-COP=20 external-resets=4 refusals=7\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
