// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_fetch_schedule.hpp"
#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_norm_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned oscillator_hz = 12000000, common_hz = 540000000;
constexpr unsigned epoch = 450, dsp_period = 45, common_factor = 9;
constexpr unsigned entry = 0x8000, resume = 0x800f, handler = 0x9000;
constexpr unsigned stack = 0x2000, wait_stack = stack - 9, unmasked = 0xc0;
constexpr unsigned foreground_d = 0x1234, foreground_x = 0x4567, foreground_y = 0x6789;
constexpr unsigned mcu_ram = 0x1200, dsp_ram = 0x100, initial_byte = 0x5a;
constexpr unsigned initial_word = 0x123456, mutation = 0x79;
constexpr unsigned nop = 0xa7, protection = 0x7f0d, security = 0x7f0f;
constexpr unsigned byte_mask = 0xff, unsecured = 0xfe, irq_vector = 0xfff2;
constexpr unsigned branch_back = 0x050fdf, reset_edges = 195, initial_wait_ticks = 210;
constexpr std::uint64_t accumulator_unit = std::uint64_t{1} << 24U;

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected joint native checkpoint rejection");
}

auto registers(const CpuState state) {
  return std::tuple{state.pc, state.d, state.x, state.y, state.sp, state.ccr};
}

McuFlashImage image() {
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  // Own LDS/LDD/LDX/LDY;ANDCC #ef;WAI;NOP: literal2+2+2+2+1+7+1 periods.
  constexpr std::array<std::uint8_t, 16> code{
      0xcf, 0x20, 0, 0xcc, 0x12, 0x34, 0xce, 0x45, 0x67, 0xcd, 0x67, 0x89, 0x10, 0xef, 0x3e, nop};
  // Own LDAA #80;STAA CRGFLG;RTI: literal1+3+8 periods.
  constexpr std::array<std::uint8_t, 6> isr{0x86, 0x80, 0x7a, 0, 0x37, 0x0b};
  bytes.fill(nop);
  for (unsigned i = 0; i < code.size(); ++i) { bytes.at(i) = code[i]; }
  for (unsigned i = 0; i < isr.size(); ++i) { bytes.at(handler - entry + i) = isr[i]; }
  bytes.at(irq_vector - entry) = handler >> 8U;
  bytes.at(irq_vector - entry + 1) = 0;
  bytes[protection] = byte_mask;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  return McuFlashImage::from_known(bytes);
}

McuRunContract mcu_inputs(const unsigned phase) {
  return {.clock = {.oscillator_hz = oscillator_hz,
                    .initial_oscillator_phase_ticks = phase,
                    .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = McuRunMemory::unstretched_internal};
}

McuDspFetchInputs inputs() {
  return {.dsp = {.extal_hz = oscillator_hz,
                  .timebase_hz = common_hz,
                  .timing = DspFetchTiming::reference_cycle_order,
                  .execution = DspReferenceExecution::staged_register_boundaries,
                  .pipeline = DspFetchPipeline::automatic_memory},
          .epoch_ticks = epoch,
          .shared_write_order = McuDspTieOrder::dsp_before_mcu,
          .maximum_dsp_cycles_per_period = 16,
          .wait_order = McuDspWaitOrder::dsp_before_mcu,
          .reset_order = McuDspResetOrder::dsp_before_mcu};
}

DspFetchCycleInputs cycle_inputs() {
  return {.known_requests = 15, .grant = DspDmaGrant::available};
}

DspMachine &initialize(DspMachine &dsp, const bool fault) {
  DspProgram code;
  if (fault) {
    code.immediate("r0", stack);
    code.immediate("x0", 7);
    constexpr unsigned delay = 48;
    for (unsigned i = 0; i < delay; ++i) { code.emit("nop"); }
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
  dsp.bus().write({.space = DspSpace::x, .address = dsp_ram, .value = initial_word});
  return dsp;
}

struct Ledger {
  std::uint64_t cycles = 0, adds = 0;
  unsigned expected_pc = boot;
};

// Include actual fetch/latches and complete native debug state,not just final values.
struct NativeTrace {
  DspDebugState cpu;
  DspProgramFetchView fetch;
  std::array<std::optional<DspProgramWord>, 3> stages;
  std::optional<DspProgramWord> retiring;
  std::optional<DspState> operands;
  std::uint32_t decode_address;
  std::uint64_t cycles;
  bool operator==(const NativeTrace &) const = default;
};

struct Scenario {
  unsigned phase = 0;
  bool failing = false;
  std::filesystem::path backend = BLOFELD_DEFAULT_SLA;
};

struct Fixture {
  explicit Fixture(const Scenario &scenario = {})
      : mcu(image(), scenario.backend, mcu_inputs(scenario.phase)),
        schedule(inputs(), mcu, initialize(dsp, scenario.failing)),
        failing(scenario.failing) {
    mcu.bus().write(mcu_ram, initial_byte);
    mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
    attach();
  }

  [[nodiscard]] NativeTrace native() const {
    const auto view = schedule.run().pipeline();
    if (!view) { throw Fault("Joint snapshot requires its real automatic pipeline"); }
    return {.cpu = dsp.cpu().debug_state(),
            .fetch = dsp.reference_fetch(),
            .stages = view->stages,
            .retiring = view->awaiting_retirement,
            .operands = view->awaiting_inputs,
            .decode_address = view->decode_address,
            .cycles = schedule.run().completed_cycles()};
  }

  void attach() {
    const auto observe = [this](const auto &result) { record(result); };
    mcu.set_cycle_observer([this, observe](const auto period) {
      schedule.begin_cycle(period, false, cycle_inputs, observe);
    });
    mcu.set_wait_period_observer([this, observe](const auto period) {
      schedule.begin_wait_period(period, cycle_inputs, observe);
    });
    mcu.set_reset_period_observer([this, observe](const auto period) {
      schedule.begin_reset_period(period, cycle_inputs, observe);
    });
  }

  void record(const DspFetchCycleResult &result) {
    require(result.completed_cycles == ledger.cycles + 1 && result.period_ticks == dsp_period &&
                result.completion_ticks == result.completed_cycles * dsp_period,
            "Every actual pipeline period follows the independent45tick clock");
    ++ledger.cycles;
    if (!result.pipeline) { throw Fault("Automatic cycle must retain its actual native phases"); }
    const auto &phase = *result.pipeline;
    if (!failing && phase.retirement_published) {
      require(phase.execute2 && phase.execute2->address == ledger.expected_pc,
              "Native retirements follow literal MOVE/CLR/ADD/BRA addresses");
      if (ledger.expected_pc == boot) {
        ledger.expected_pc = boot + 2;
      } else if (ledger.expected_pc == boot + 2) {
        require(dsp.cpu().state().a == 0, "Native CLR has its independent zero result");
        ledger.expected_pc = boot + 3;
      } else if (ledger.expected_pc == boot + 3) {
        ++ledger.adds;
        require(dsp.cpu().state().a == ledger.adds * accumulator_unit,
                "Native ADD has the independently counted full accumulator result");
        ledger.expected_pc = boot + 4;
      } else {
        ledger.expected_pc = boot + 3;
      }
    }
    trace.push_back(native());
  }

  CpuStep step() {
    const auto result = mcu.cpu().step();
    schedule.end_instruction();
    return result;
  }

  void idle(const unsigned periods) {
    require(mcu.advance_wait(periods) == periods, "Requested actual native idle periods complete");
    schedule.end_wait();
    time();
  }

  void enter_wait(const unsigned completed = 0) {
    for (unsigned i = completed; i < 6; ++i) { static_cast<void>(step()); }
    const auto cpu = mcu.cpu().state();
    require(mcu.cpu().waiting() && cpu.pc == resume && cpu.sp == wait_stack &&
                cpu.d == foreground_d && cpu.x == foreground_x && cpu.y == foreground_y &&
                cpu.ccr == unmasked && mcu.run_clock().elapsed_ticks() == initial_wait_ticks,
            "Literal native setup reaches WAI with all independently expected registers");
    constexpr std::array<unsigned, 9> frame{
        unmasked, 0x34, 0x12, 0x45, 0x67, 0x67, 0x89, 0x80, 0x0f};
    for (unsigned i = 0; i < frame.size(); ++i) {
      require(mcu.bus().read(static_cast<std::uint16_t>(wait_stack + i)) == frame[i],
              "Native WAI frame matches all nine independent NXP bytes");
    }
    time();
  }

  void wake() {
    const auto before = mcu.run_clock().bus_cycles();
    mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 1});
    const auto irq = step();
    require(irq.interrupt_vector == irq_vector && irq.stack_reused && !irq.instruction_executed &&
                mcu.cpu().state().pc == handler && mcu.cpu().state().sp == wait_stack,
            "Actual IRQ uses the native six-period wake and existing WAI frame");
    mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
    for (unsigned i = 0; i < 3; ++i) { static_cast<void>(step()); }
    const auto cpu = mcu.cpu().state();
    constexpr unsigned wake_periods = 18;
    require(cpu.pc == resume && cpu.sp == stack && cpu.d == foreground_d && cpu.x == foreground_x &&
                cpu.y == foreground_y && cpu.ccr == unmasked &&
                mcu.run_clock().bus_cycles() == before + wake_periods && !mcu.cpu().waiting(),
            "Native1+3+8-period ISR/RTI restores the full foreground without debug writes");
    time();
  }

  void time() const {
    const auto common = mcu.run_clock().elapsed_ticks() * common_factor;
    require(schedule.end_ticks() == common && ledger.cycles == (common - epoch) / dsp_period &&
                schedule.run().completed_cycles() == ledger.cycles &&
                schedule.run().elapsed_ticks() == ledger.cycles * dsp_period &&
                schedule.run().timer_clk2_edges() == ledger.cycles / 2,
            "Both native owners and CLK2 match the independent integer time ledger");
  }

  McuMachine mcu;
  DspMachine dsp;
  McuDspFetchSchedule schedule;
  bool failing;
  Ledger ledger;
  std::vector<NativeTrace> trace;
};

struct Saved {
  McuDspFetchCheckpoint checkpoint;
  Ledger ledger;
  NativeTrace native;
  std::uint64_t mcu_ticks;
};

Saved capture(Fixture &f) {
  return {.checkpoint = f.schedule.checkpoint(),
          .ledger = f.ledger,
          .native = f.native(),
          .mcu_ticks = f.mcu.run_clock().elapsed_ticks()};
}

void restore(Fixture &f, const Saved &saved) {
  f.schedule.restore(saved.checkpoint);
  f.ledger = saved.ledger; // External test observer cursors are explicitly caller-owned.
  f.trace.clear();
  f.attach();
  require(f.native() == saved.native && f.mcu.run_clock().elapsed_ticks() == saved.mcu_ticks &&
              f.mcu.bus().read(mcu_ram) == initial_byte &&
              f.dsp.bus().read({.space = DspSpace::x, .address = dsp_ram}) == initial_word &&
              !f.schedule.faulted(),
          "Joint restore recovers complete native CPU/fetch/latches,clocks and deep bus contents");
  f.time();
}

void ordinary(const unsigned cut) {
  Fixture f;
  for (unsigned i = 0; i < cut; ++i) { static_cast<void>(f.step()); }
  const auto expected_cpu = registers(f.mcu.cpu().state());
  const auto saved = capture(f);
  f.enter_wait(cut);
  f.wake();
  const auto suffix = f.trace;
  f.mcu.bus().write(mcu_ram, mutation);
  f.dsp.bus().write({.space = DspSpace::x, .address = dsp_ram, .value = mutation});
  restore(f, saved);
  require(registers(f.mcu.cpu().state()) == expected_cpu, "All six actual MCU registers rewind");
  f.enter_wait(cut);
  f.wake();
  require(
      std::vector<NativeTrace>(suffix.begin() + static_cast<std::ptrdiff_t>(saved.ledger.cycles),
                               suffix.end()) == f.trace,
      "Independently checked future native effects and every fetch/latch observation repeat");
}

void waiting(const unsigned cut) {
  Fixture f;
  f.enter_wait();
  f.idle(cut);
  const auto saved = capture(f);
  for (const unsigned delay : {0U, 7U}) {
    restore(f, saved);
    f.idle(delay);
    f.wake();
    const auto suffix = f.trace;
    const auto end = f.mcu.run_clock().elapsed_ticks();
    restore(f, saved);
    f.idle(delay);
    f.wake();
    require(f.trace == suffix && f.mcu.run_clock().elapsed_ticks() == end,
            "One immutable WAI checkpoint supports independent future IRQ schedules");
  }
}

struct ResetScenario {
  unsigned cut;
  unsigned phase;
  bool hold = false;
};

void reset(const ResetScenario scenario) {
  const auto [cut, phase, hold] = scenario;
  Fixture f({.phase = phase});
  f.schedule.begin_system_reset(McuResetSource::external, 3);
  f.attach();
  require(f.mcu.advance_system_reset(cut, McuResetPad::low) == cut,
          "Actual reset reaches the requested native SYSCLK capture edge");
  const auto reset_state = f.mcu.system_reset_state();
  const auto saved = capture(f);
  for (unsigned repeat = 0; repeat < 2; ++repeat) {
    restore(f, saved);
    require(f.mcu.system_reset_state() == reset_state && f.mcu.system_reset_active(),
            "Capture retains actual reset phase,source,synchronization and edge count");
    const unsigned remaining = reset_edges - cut;
    require(f.mcu.advance_system_reset(remaining, hold ? McuResetPad::low : McuResetPad::high) ==
                remaining,
            "Native CRG reset continues from its saved edge");
    if (hold) {
      const auto state = f.mcu.system_reset_state();
      require(state && state->phase == McuResetPhase::external_hold,
              "Observed low RESET pad holds the real CPU in reset");
      require(f.mcu.advance_system_reset(1, McuResetPad::high) == 1,
              "A later observed high pad releases the actual reset");
    }
    f.schedule.end_system_reset();
    const auto expected = 50U + ((reset_edges + (hold ? 1U : 0U)) * 5U) - phase + 50U;
    require(f.mcu.run_clock().elapsed_ticks() == expected && f.mcu.cpu().state().pc == entry &&
                f.mcu.cpu().state().ccr == hc12::reset_ccr &&
                f.mcu.run_clock().native_epoch() == 2 && f.ledger.adds > saved.ledger.adds,
            "Native reset and five vector periods preserve SRAM/DSP with literal OSC phase");
    f.time();
  }
}

void fault_recovery() {
  Fixture f({.failing = true});
  f.enter_wait();
  const auto saved = capture(f);
  std::string expected;
  std::vector<NativeTrace> suffix;
  for (unsigned repeat = 0; repeat < 2; ++repeat) {
    restore(f, saved);
    const auto failure = rejects([&] { f.idle(64); });
    require(failure.contains("2000") && failure.contains("mcu-pc=") && f.schedule.faulted() &&
                !rejects([&] { static_cast<void>(f.schedule.checkpoint()); }).empty(),
            "Actual reserved-Y store remains a contextual native failure and quarantines capture");
    if (repeat == 0) {
      expected = failure;
      suffix = f.trace;
    } else {
      require(failure == expected && f.trace == suffix,
              "Trusted joint restore recovers both owners and reproduces the same visible fault");
    }
  }
  restore(f, saved);
}

void foreign_owner() {
  Fixture left;
  Fixture right;
  left.enter_wait();
  right.enter_wait();
  const auto saved = capture(left);
  const auto before = right.native();
  const auto cpu = registers(right.mcu.cpu().state());
  const auto ticks = right.mcu.run_clock().elapsed_ticks();
  require(rejects([&] { right.schedule.restore(saved.checkpoint); }).contains("matching") &&
              right.native() == before && registers(right.mcu.cpu().state()) == cpu &&
              right.mcu.run_clock().elapsed_ticks() == ticks && !right.schedule.faulted(),
          "A foreign owner rejects before changing either native machine or its health");
  right.idle(1);
}

void active_owner() {
  Fixture f;
  const auto saved = capture(f);
  unsigned rejected = 0;
  f.mcu.set_cycle_observer([&](const auto period) {
    f.schedule.begin_cycle(period, false, cycle_inputs, [&](const auto &result) {
      require(rejects([&] { f.schedule.restore(saved.checkpoint); }).contains("inactive") &&
                  rejects([&] { static_cast<void>(f.schedule.checkpoint()); }).contains("inactive"),
              "Reentrant snapshot operations fail before any native replacement");
      ++rejected;
      f.record(result);
    });
  });
  static_cast<void>(f.step());
  require(rejected > 0 && !f.schedule.faulted(),
          "Caught reentrant requests preserve outer execution");
  restore(f, saved);
  f.enter_wait();
}

void incomplete_offer(const bool wait) {
  Fixture f;
  if (wait) { f.enter_wait(); }
  const auto saved = capture(f);
  if (wait) {
    require(f.mcu.advance_wait(1) == 1,
            "Native wait offer is consumed before its end notification");
  } else {
    static_cast<void>(f.mcu.cpu().step());
  }
  require(rejects([&] { static_cast<void>(f.schedule.checkpoint()); }).contains("completed"),
          "Capture refuses an unclosed native instruction or idle offer");
  restore(f, saved);
  if (wait) {
    f.idle(1);
    f.wake();
  } else {
    f.enter_wait();
  }
}

void external_observers() {
  Fixture f;
  f.enter_wait();
  const auto saved = capture(f);
  unsigned external_calls = 0;
  f.mcu.set_wait_observer([&](const auto) { ++external_calls; });
  f.idle(1);
  require(external_calls == 1, "Caller-owned native observation is attached before restore");
  f.schedule.restore(saved.checkpoint);
  require(f.mcu.advance_wait(1) == 1 && external_calls == 1,
          "Native restore drops external wiring and observer callbacks");
  require(rejects([&] { static_cast<void>(f.schedule.checkpoint()); }).contains("consumed"),
          "Missing reattachment cannot silently relabel an unobserved native idle period");
  restore(f, saved);
  f.wake();
}

void unknown_reset_pad() {
  Fixture f;
  f.schedule.begin_system_reset(McuResetSource::external, 3);
  f.attach();
  f.mcu.advance_system_reset(reset_edges - 1, McuResetPad::unknown);
  const auto saved = capture(f);
  require(
      rejects([&] { f.mcu.advance_system_reset(1, McuResetPad::unknown); }).contains("RESET pad") &&
          f.schedule.faulted() && f.native() == saved.native &&
          f.mcu.run_clock().elapsed_ticks() == saved.mcu_ticks,
      "Unknown RESET sampling rejects before either native clock advances");
  restore(f, saved);
  require(f.mcu.advance_system_reset(1, McuResetPad::high) == 1,
          "Trusted restore permits a later explicitly known reset-pad observation");
  f.schedule.end_system_reset();
  require(f.mcu.cpu().state().pc == entry, "Recovered reset fetches the actual original vector");
  f.time();
}

void input_contract() {
  McuMachine mcu(image(), BLOFELD_DEFAULT_SLA, mcu_inputs(0));
  DspMachine dsp;
  initialize(dsp, false);
  const auto before_clock = dsp.checkpoint();
  std::optional<McuDspFetchCheckpoint> saved;
  {
    const McuDspFetchSchedule schedule(inputs(), mcu, dsp);
    saved.emplace(schedule.checkpoint());
  }
  dsp.restore(before_clock);
  auto other_inputs = inputs();
  other_inputs.maximum_dsp_cycles_per_period *= 2;
  McuDspFetchSchedule other(other_inputs, mcu, dsp);
  const auto cpu = dsp.cpu().debug_state();
  const auto fetch = dsp.reference_fetch();
  if (!saved) { throw Fault("Actual checkpoint must survive its schedule's lifetime"); }
  require(rejects([&] { other.restore(*saved); }).contains("contracts") && !other.faulted() &&
              other.run().completed_cycles() == 0 && dsp.cpu().debug_state() == cpu &&
              dsp.reference_fetch() == fetch && mcu.cpu().state().pc == entry,
          "Even matching native owners reject a different scheduler contract before mutation");
}

void backend_failure(const std::filesystem::path &temporary) {
  const auto backend = temporary / "joint-checkpoint.sla";
  std::filesystem::copy_file(BLOFELD_DEFAULT_SLA, backend);
  Fixture f({.backend = backend});
  f.enter_wait();
  const auto saved = capture(f);
  f.idle(1);
  const auto before = f.native();
  {
    std::ofstream changed(backend, std::ios::binary | std::ios::app);
    changed.exceptions(std::ios::failbit | std::ios::badbit);
    changed.put('x');
  }
  require(!rejects([&] { f.schedule.restore(saved.checkpoint); }).empty() && f.schedule.faulted() &&
              f.native() == before,
          "A rejected native restore quarantines the joint owner without inventing success");
  std::filesystem::remove(backend);
  std::filesystem::copy_file(BLOFELD_DEFAULT_SLA, backend);
  restore(f, saved);
  f.wake();
  std::filesystem::remove(backend);
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 2) { throw Fault("Usage: mcu-dsp-fetch-checkpoint-test temporary-directory"); }
    for (unsigned cut = 0; cut < 6; ++cut) { ordinary(cut); }
    for (const unsigned cut : {0U, 1U, 17U, 31U}) { waiting(cut); }
    for (unsigned phase = 0; phase < 5; ++phase) {
      for (const unsigned cut : {0U, 1U, 131U, 194U}) { reset({.cut = cut, .phase = phase}); }
      reset({.cut = reset_edges, .phase = phase, .hold = true});
    }
    fault_recovery();
    foreign_owner();
    active_owner();
    incomplete_offer(false);
    incomplete_offer(true);
    external_observers();
    unknown_reset_pad();
    input_contract();
    backend_failure(arguments[1]);
    std::cout << "44 joint native checkpoint scenarios passed:6 run,4 WAI,25 reset,9 guards\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
