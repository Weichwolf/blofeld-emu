// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_pll.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/dsp_scheduled_run.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_cycles.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_cop.hpp"
#include "blofeld/mcu_dsp_schedule.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/mcu_reset.hpp"
#include "blofeld/mcu_run_clock.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using blofeld::test::require;
constexpr unsigned oscillator_hz = 12000000, common_hz = 540000000;
constexpr unsigned epoch = 450, byte_mask = 255;
constexpr std::uint64_t native_unit_ticks = 45;
constexpr unsigned entry = 0x8000, nop = 0xa7, protection = 0x7f0d, security = 0x7f0f,
                   unsecured = 0xfe;
constexpr unsigned drive = 128, settle = 64, minimum_sync = 3, maximum_sync = 6;
constexpr unsigned initial_ticks = 50, vector_ticks = 50, reset_cycles = 5;
constexpr unsigned absolute_jump = 0x0af080, reserved = 0x2000;
constexpr unsigned failure_units = 4, failure_mcu_ticks = 70;
constexpr unsigned cop_edges = 16384, cop_ticks = 81920, phy_select = 0x80;
// Own CPU12 WAI, BRA -2 and STAA EXT encodings; target is known SRAM.
constexpr unsigned wai_opcode = 0x3e, branch_self = 0xfe, store_opcode = 0x7a;
constexpr unsigned store_target = 0x1200, stored_value = 0x5a;
constexpr unsigned nominal_reset_edges = 195, nominal_reset_ticks = 1025;
constexpr unsigned interrupted_store_ticks = 65, interrupted_store_common = 585;
enum class Program : std::uint8_t { nop, branch, wait, store };

std::string rejected(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected joint-reset refusal");
}

McuRunContract mcu_contract(const unsigned phase = 0) {
  return {.clock = {.oscillator_hz = oscillator_hz,
                    .initial_oscillator_phase_ticks = phase,
                    .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = McuRunMemory::internal_flash_reads};
}

DspRunContract dsp_contract() {
  return {.clock = {.extal_hz = oscillator_hz,
                    .timebase_hz = common_hz,
                    .switch_timing = DspClockSwitch::reference_instruction_boundary},
          .timing = DspRunTiming::native_internal_estimates};
}

DspMachine &initialize(DspMachine &dsp, const bool failing) {
  blofeld::test::DspProgram code;
  if (failing) {
    code.immediate("r0", reserved);
    code.immediate("x0", 7);
    code.emit("move x0,y:(r0)");
  } else {
    code.emit("nop");
    code.word(absolute_jump);
    code.word(blofeld::test::boot);
  }
  code.install(dsp.bus());
  dsp.bus().pll().observe_pinit(false);
  dsp.cpu().reset_spi_boot();
  return dsp;
}

McuFlashImage image(const Program program = Program::nop) {
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  bytes.fill(nop);
  if (program == Program::wait) { bytes[0] = wai_opcode; }
  if (program == Program::branch) {
    bytes[0] = 0x20;
    bytes[1] = branch_self;
  }
  if (program == Program::store) {
    bytes[0] = store_opcode;
    bytes[1] = store_target >> 8U;
    bytes[2] = 0;
  }
  bytes[protection] = byte_mask;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  bytes[bytes.size() - 6] = entry >> 8U;
  bytes[bytes.size() - 5] = 0;
  return McuFlashImage::from_known(bytes);
}

struct Fixture {
  explicit Fixture(const bool failing = false,
                   const Program program = Program::nop,
                   const unsigned phase = 0)
      : mcu(image(program), BLOFELD_DEFAULT_SLA, mcu_contract(phase)),
        schedule({.dsp = {.epoch_ticks = epoch,
                          .observation = DspStepObservation::reference_completion_boundary},
                  .maximum_dsp_steps_per_cycle = 16,
                  .wait_order = McuDspWaitOrder::dsp_before_mcu},
                 mcu,
                 initialize(dsp, failing)),
        faulting(failing) {}

  void record(const DspScheduledStep &event) {
    std::uint64_t cost = 2;
    if (!faulting) { cost = event.plan.before.cpu.pc == blofeld::test::boot ? 1 : 4; }
    require(event.plan.native_cycle_units == cost &&
                event.plan.duration_ticks == cost * native_unit_ticks &&
                event.plan.start_ticks == units * native_unit_ticks &&
                event.completion_ticks == epoch + (units + cost) * native_unit_ticks,
            "Each real DSP instruction follows independent immediate/NOP/JMP cycle expectations");
    units += cost;
    ++steps;
  }

  void attach(const bool shared = false) {
    mcu.set_cycle_observer([this, shared](const Hc12BusCycle cycle) {
      schedule.begin_cycle(cycle,
                           shared && cycle.kind == 'w',
                           {},
                           {.observe = [this](const auto &event) { record(event); }});
      ++native_cycles;
    });
    mcu.set_wait_period_observer([this](const McuWaitPeriod period) {
      schedule.begin_wait_period(
          period, {}, {.observe = [this](const auto &event) { record(event); }});
    });
    mcu.set_reset_period_observer([this](const McuResetPeriod period) {
      require(period.native_epoch == 2 && period.duration_ticks == 5,
              "Each nominal SYSCLK offer is an actual OSC edge in the fresh MCU epoch");
      schedule.begin_reset_period(
          period, {}, {.observe = [this](const auto &event) { record(event); }});
      ++reset_edges;
    });
  }

  void begin(const unsigned synchronization = minimum_sync) {
    schedule.begin_system_reset(
        McuResetSource::external, synchronization, McuDspResetOrder::dsp_before_mcu);
    attach();
  }

  McuMachine mcu;
  DspMachine dsp{dsp_contract()};
  McuDspSchedule schedule;
  bool faulting;
  std::uint64_t units = 0, steps = 0, reset_edges = 0, native_cycles = 0;
};

void test_all_phases_and_checkpoint() {
  for (unsigned sync = minimum_sync; sync <= maximum_sync; ++sync) {
    Fixture fixture;
    fixture.begin(sync);
    require(
        rejected([&] { static_cast<void>(fixture.schedule.checkpoint()); }).contains("mid-reset"),
        "Joint snapshots cannot silently omit board reset state");
    const auto count = drive + sync + settle;
    require(fixture.mcu.advance_system_reset(count, McuResetPad::high) == count,
            "All195..198 reset edges execute before the real native vector sequence");
    fixture.schedule.end_system_reset();
    const std::uint64_t available = count + (2 * reset_cycles);
    const auto loops = available / 5;
    const auto tail = static_cast<std::uint64_t>(available % 5 != 0);
    const auto completed = (loops * 5) + tail;
    require(
        fixture.units == completed && fixture.steps == 2 * loops + tail &&
            fixture.reset_edges == count && fixture.native_cycles == reset_cycles &&
            fixture.mcu.run_clock().elapsed_ticks() ==
                initial_ticks + (count * 5ULL) + vector_ticks &&
            fixture.schedule.end_ticks() == epoch + available * native_unit_ticks &&
            fixture.dsp.run_clock().elapsed_ticks() == completed * native_unit_ticks &&
            fixture.dsp.run_clock().native_epoch() == 1 &&
            fixture.mcu.run_clock().native_epoch() == 2 &&
            fixture.mcu.run_clock().bus_cycles() == 2ULL * reset_cycles &&
            !fixture.schedule.faulted(),
        "DSP epoch/time is continuous across every reset edge and all five native reset cycles");
    const auto mcu = fixture.mcu.checkpoint();
    const auto dsp = fixture.dsp.checkpoint();
    const auto schedule = fixture.schedule.checkpoint();
    const auto old_units = fixture.units;
    const auto old_steps = fixture.steps;
    for (unsigned replay = 0; replay < 2; ++replay) {
      fixture.mcu.restore(mcu);
      fixture.dsp.restore(dsp);
      fixture.schedule.restore(schedule);
      fixture.units = old_units;
      fixture.steps = old_steps;
      fixture.attach();
      fixture.mcu.cpu().step();
      fixture.schedule.end_instruction();
      require(fixture.mcu.cpu().state().pc == entry + 1 && !fixture.schedule.faulted(),
              "Completed joint-reset snapshots replay in the new epoch without resetting DSP time");
    }
  }
}

void test_unknown_pad_and_reentrancy() {
  Fixture fixture;
  const auto original = fixture.mcu.checkpoint();
  require(rejected([&] {
            fixture.schedule.begin_system_reset(
                McuResetSource::external, minimum_sync, McuDspResetOrder::unqualified);
          }).contains("explicit DSP-before-MCU") &&
              fixture.mcu.run_clock().native_epoch() == 1,
          "Unqualified reset ordering cannot replace the MCU or change native epochs");
  fixture.begin();
  bool guarded = false;
  fixture.mcu.set_reset_period_observer([&](const McuResetPeriod period) {
    if (!guarded) {
      require(rejected([&] { fixture.mcu.restore(original); }).contains("active MCU") &&
                  rejected([&] {
                    static_cast<void>(fixture.mcu.checkpoint());
                  }).contains("reset advancement") &&
                  rejected([&] {
                    fixture.mcu.advance_system_reset(1, McuResetPad::high);
                  }).contains("inactive CPU"),
              "A reset observer cannot replace or recursively advance its active native owner");
      guarded = true;
    }
    fixture.schedule.begin_reset_period(
        period, {}, {.observe = [&](const auto &event) { fixture.record(event); }});
  });
  fixture.mcu.advance_system_reset(drive + minimum_sync + settle - 1, McuResetPad::unknown);
  const auto mcu_ticks = fixture.mcu.run_clock().elapsed_ticks();
  const auto dsp_ticks = fixture.dsp.run_clock().elapsed_ticks();
  require(guarded && rejected([&] {
                       fixture.mcu.advance_system_reset(1, McuResetPad::unknown);
                     }).contains("Unknown MCU RESET pad") &&
              fixture.mcu.run_clock().elapsed_ticks() == mcu_ticks &&
              fixture.dsp.run_clock().elapsed_ticks() == dsp_ticks && fixture.schedule.faulted(),
          "Unknown final source sampling stops before either owner advances the unqualified edge");
}

void test_unconsumed_boundaries() {
  Fixture skipped_native;
  skipped_native.mcu.cpu().step();
  require(rejected([&] {
            skipped_native.schedule.begin_system_reset(
                McuResetSource::external, minimum_sync, McuDspResetOrder::dsp_before_mcu);
          }).contains("every offered native MCU period") &&
              skipped_native.mcu.run_clock().native_epoch() == 1 &&
              skipped_native.dsp.run_clock().elapsed_ticks() == 0,
          "Unobserved native time cannot be discarded by reset admission");
  Fixture skipped_reset;
  skipped_reset.begin();
  skipped_reset.mcu.set_reset_period_observer({});
  skipped_reset.mcu.advance_system_reset(1, McuResetPad::unknown);
  require(rejected([&] {
            skipped_reset.schedule.begin_reset_period(
                {.context = {}, .duration_ticks = 5, .native_epoch = 2});
          }).contains("every offered native MCU period") &&
              skipped_reset.dsp.run_clock().elapsed_ticks() == 0,
          "A missing reset edge observer cannot silently retime the independent DSP");
}

void test_dsp_failure_prefix() {
  Fixture fixture(true);
  fixture.begin();
  const auto error = rejected([&] { fixture.mcu.advance_system_reset(32, McuResetPad::high); });
  require(error.contains("Scheduled reset peer execution") && error.contains("0x002000") &&
              fixture.schedule.faulted() && fixture.mcu.timing_faulted() &&
              fixture.dsp.timing_faulted() && fixture.units == failure_units &&
              fixture.dsp.run_clock().elapsed_ticks() == failure_units * native_unit_ticks &&
              fixture.mcu.run_clock().elapsed_ticks() == failure_mcu_ticks &&
              fixture.mcu.run_clock().reset_sysclk_edges() == failure_units,
          "A real unknown DSP write quarantines both owners with their actual completed "
          "reset/instruction prefix");
  const auto ticks = fixture.dsp.run_clock().elapsed_ticks();
  static_cast<void>(rejected([&] { fixture.mcu.advance_system_reset(1, McuResetPad::high); }));
  require(fixture.dsp.run_clock().elapsed_ticks() == ticks,
          "Failed peer execution cannot accrue hidden DSP time");
}

void check_literal_dsp_time(const Fixture &fixture) {
  const auto available = (fixture.schedule.end_ticks() - epoch) / native_unit_ticks;
  const auto loops = available / 5;
  const auto tail = static_cast<std::uint64_t>(available % 5 != 0);
  require(fixture.units == loops * 5 + tail && fixture.steps == loops * 2 + tail &&
              fixture.dsp.run_clock().elapsed_ticks() == fixture.units * native_unit_ticks &&
              fixture.dsp.run_clock().native_epoch() == 1,
          "Independent 1/4-unit NOP/JMP costs bound DSP progress at the exact COP/reset horizon");
}

void test_timeout(const bool waiting, const bool phy, const unsigned phase) {
  Fixture fixture(false, waiting ? Program::wait : Program::branch, phase);
  fixture.mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
  fixture.attach();
  if (waiting) {
    fixture.mcu.cpu().set_state({.pc = entry, .sp = 0x2000, .ccr = hc12::reset_ccr});
    fixture.mcu.cpu().step();
    fixture.schedule.end_instruction();
    require(fixture.mcu.cpu().waiting() && fixture.mcu.run_clock().bus_cycles() == 12,
            "Native WAI enters after five reset and seven entry bus cycles");
  }
  if (phy) {
    fixture.mcu.bus().write(hc12::usb_control, 0x40);
    fixture.mcu.bus().observe_phy({.edges = 15});
    fixture.mcu.bus().write(hc12::clock_select, phy_select);
  }
  fixture.mcu.bus().write(hc12::cop_control, 1);
  const auto start = fixture.mcu.run_clock().elapsed_ticks();
  const auto cycles = fixture.mcu.run_clock().bus_cycles();
  const auto duration = cop_ticks - phase;
  const auto period = phy ? 2U : 10U;
  const auto completed_periods = (duration - 1) / period;
  unsigned completed = 0;
  const auto error = rejected([&] {
    for (; completed < cop_edges * 3; ++completed) {
      if (waiting) {
        fixture.mcu.advance_wait(1);
        fixture.schedule.end_wait();
      } else {
        fixture.mcu.cpu().step();
        fixture.schedule.end_instruction();
      }
    }
  });
  require(error.contains("COP system reset (timeout)") &&
              fixture.mcu.bus().cop_state().reset == McuCopReset::timeout &&
              fixture.mcu.bus().cop_state().phase == cop_edges &&
              fixture.mcu.run_clock().elapsed_ticks() == start + duration &&
              fixture.mcu.run_clock().bus_cycles() == cycles + completed_periods &&
              fixture.mcu.idle_bus_cycles() == (waiting ? completed_periods : 0) &&
              completed == (waiting ? completed_periods : completed_periods / 3) &&
              fixture.schedule.end_ticks() == (start + duration) * 9 &&
              fixture.schedule.faulted() && !fixture.dsp.timing_faulted(),
          "A natural COP timeout preempts its real IPBus slot without a completed bus/idle cycle");
  check_literal_dsp_time(fixture);
  fixture.schedule.begin_system_reset(
      McuResetSource::cop, minimum_sync, McuDspResetOrder::dsp_before_mcu);
  fixture.attach();
  require(fixture.mcu.advance_system_reset(nominal_reset_edges, McuResetPad::high) ==
              nominal_reset_edges,
          "The exact timed-out joint boundary admits the explicit owned COP reset");
  fixture.schedule.end_system_reset();
  require(fixture.mcu.cpu().state().pc == entry &&
              fixture.mcu.run_clock().elapsed_ticks() == start + duration + nominal_reset_ticks &&
              fixture.mcu.run_clock().bus_cycles() == cycles + completed_periods + 5 &&
              fixture.mcu.run_clock().native_epoch() == 2 && !fixture.schedule.faulted(),
          "COP native vector reset retains the partial period and advances the same DSP epoch");
  check_literal_dsp_time(fixture);
}

void test_timeout_preempts_write() {
  Fixture fixture(false, Program::store);
  fixture.attach(true);
  fixture.mcu.bus().write(store_target, stored_value);
  fixture.mcu.bus().write(hc12::cop_control, 1);
  fixture.mcu.bus().advance_oscillator(cop_edges -
                                       3); // Explicit near-deadline module precondition.
  const auto error = rejected([&] { fixture.mcu.cpu().step(); });
  require(error.contains("COP system reset (timeout)") &&
              fixture.mcu.bus().read(store_target) == stored_value &&
              fixture.mcu.run_clock().elapsed_ticks() == interrupted_store_ticks &&
              fixture.mcu.run_clock().bus_cycles() == 6 &&
              fixture.schedule.end_ticks() == interrupted_store_common,
          "A PwO store times out inside w; neither the shared write nor its bus period commits");
  check_literal_dsp_time(fixture);
  fixture.schedule.begin_system_reset(
      McuResetSource::cop, minimum_sync, McuDspResetOrder::dsp_before_mcu);
  fixture.attach();
  fixture.mcu.advance_system_reset(nominal_reset_edges, McuResetPad::high);
  fixture.schedule.end_system_reset();
  require(fixture.mcu.bus().read(store_target) == stored_value && !fixture.schedule.faulted(),
          "Reset admission discards only the timed-out write offer and retains pre-write SRAM");
}

}

int main() {
  try {
    test_all_phases_and_checkpoint();
    test_unknown_pad_and_reentrancy();
    test_unconsumed_boundaries();
    test_dsp_failure_prefix();
    for (const bool waiting : {false, true}) {
      for (const bool phy : {false, true}) {
        for (const unsigned phase : {0U, 3U}) { test_timeout(waiting, phy, phase); }
      }
    }
    test_timeout_preempts_write();
    std::cout << "Joint reset edges, native vectors, independent DSP costs, replay and failure "
                 "prefixes verified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
