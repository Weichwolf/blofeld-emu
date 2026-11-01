// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
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
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

namespace {
using namespace blofeld;
constexpr unsigned oscillator_hz = 12000000, common_hz = 540000000;
constexpr unsigned boot = 0xff0000, dsp_period = 45, common_factor = 9, recovery_ticks = 90;
constexpr unsigned entry = 0x8000, resume = 0x800f, handler = 0x9000;
constexpr unsigned stack = 0x2000, wait_stack = stack - 9, unmasked = 0xc0;
constexpr unsigned foreground_d = 0x1234, foreground_x = 0x4567, foreground_y = 0x6789;
constexpr unsigned mcu_ram = 0x1200, dsp_ram = 0x100, initial_byte = 0x5a;
constexpr unsigned initial_word = 0x123456, mutation = 0x79;
constexpr unsigned nop = 0xa7, protection = 0x7f0d, security = 0x7f0f;
constexpr unsigned byte_mask = 0xff, unsecured = 0xfe, irq_vector = 0xfff2;
constexpr unsigned branch_back = 0x050fdf, reset_edges = 195, initial_wait_ticks = 210;
constexpr unsigned fault_delay = 48;
constexpr std::uint64_t accumulator_unit = std::uint64_t{1} << 24U;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const std::exception &error) { return error.what(); }
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

McuDspRuntimeInputs inputs() {
  return {.timing = OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts,
          .maximum_cycles_per_period = 16,
          .wait_dsp_first = true,
          .reset_dsp_first = true,
          .initial_reset = OwnDspResetInputs{
              .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}};
}

DspDmaInputs cycle_inputs() {
  return {.known_requests = 15,
          .timing = DspDmaTiming::reference_sampled_cycles,
          .grant = DspDmaGrant::available};
}

void initialize(OwnDspMachine &dsp, const bool fault) {
  // DSP56300FM literal MOVE immediate, CLR, ADD, BRA and indirect Y store forms.
  constexpr unsigned immediate_x1 = 0x45f400;
  constexpr unsigned clear_a = 0x200013;
  constexpr unsigned add_x1 = 0x200060;
  constexpr unsigned immediate_r0 = 0x60f400;
  constexpr unsigned immediate_x0 = 0x44f400;
  constexpr unsigned store_x0_y_r0 = 0x4c6000;
  constexpr unsigned rom_words = 128;
  std::array<std::uint32_t, rom_words> code{};
  if (fault) {
    code[0] = immediate_r0;
    code[1] = stack;
    code[2] = immediate_x0;
    code[3] = 7;
    code[4 + fault_delay] = store_x0_y_r0;
  } else {
    code[0] = immediate_x1;
    code[1] = 1;
    code[2] = clear_a;
    code[3] = add_x1;
    code[4] = branch_back;
  }
  dsp.bus().install_rom({.space = DspSpace::p, .start = boot, .words = code});
  dsp.bus().write({.space = DspSpace::x, .address = dsp_ram, .value = initial_word});
}

struct Ledger {
  std::uint64_t cycles = 0, adds = 0;
  unsigned expected_pc = boot;
};

struct NativeTrace {
  dsp56300::Registers cpu;
  dsp56300::Progress progress;
  std::optional<dsp56300::InstructionEffect> pending;
  std::uint64_t elapsed, reset_ticks, cycles, clk2;
  bool interrupt_pending;
  bool operator==(const NativeTrace &) const = default;
};

struct CycleTrace {
  NativeTrace native;
  std::optional<std::uint32_t> issued, native_pc;
  bool captured, started, retired;
  bool operator==(const CycleTrace &) const = default;
};

struct Scenario {
  unsigned phase = 0;
  bool failing = false;
  std::filesystem::path backend = BLOFELD_DEFAULT_SLA;
};

struct Fixture {
  explicit Fixture(const Scenario &scenario = {})
      : mcu(image(), scenario.backend, mcu_inputs(scenario.phase)), failing(scenario.failing) {
    initialize(dsp, failing);
    mcu.bus().write(mcu_ram, initial_byte);
    mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
    recovered = mcu.run_clock().elapsed_ticks() * common_factor + recovery_ticks;
    runtime = std::make_unique<McuDspRuntime>(inputs(), mcu, dsp);
    runtime->release_dsp_reset();
    attach();
  }

  [[nodiscard]] NativeTrace native() const {
    const auto pending = dsp.cpu().pending_instruction();
    return {.cpu = dsp.cpu().registers(),
            .progress = dsp.cpu().progress(),
            .pending = pending ? std::optional{pending->effect} : std::nullopt,
            .elapsed = dsp.clock().elapsed_ticks(),
            .reset_ticks = dsp.clock().hardware_reset_ticks(),
            .cycles = dsp.clock().native_cycle_units(),
            .clk2 = dsp.clock().timer_clk2_edges(),
            .interrupt_pending = dsp.interrupt_pending()};
  }

  void attach() {
    const auto observe = [this](const auto &result) { record(result); };
    mcu.set_cycle_observer([this, observe](const auto period) {
      runtime->begin_cycle(period, false, cycle_inputs, observe);
    });
    mcu.set_wait_period_observer([this, observe](const auto period) {
      runtime->begin_wait_period(period, cycle_inputs, observe);
    });
    mcu.set_reset_period_observer([this, observe](const auto period) {
      runtime->begin_reset_period(period, cycle_inputs, observe);
    });
  }

  void record(const DspRuntimeCycle &cycle) {
    const auto &result = cycle.result;
    require(cycle.cycle == ledger.cycles &&
                cycle.completion_ticks - cycle.start_ticks == dsp_period &&
                cycle.completion_ticks == recovered + ((ledger.cycles + 1) * dsp_period),
            "Every actual F1/F2/effect period follows held reset and the independent45tick clock");
    ++ledger.cycles;
    if (!failing && result.native && result.native->retired) {
      require(result.native_pc && *result.native_pc == ledger.expected_pc,
              "Native retirements follow literal MOVE/CLR/ADD/BRA addresses");
      if (ledger.expected_pc == boot) {
        ledger.expected_pc = boot + 2;
      } else if (ledger.expected_pc == boot + 2) {
        require(dsp.cpu().registers().a == 0, "Native CLR has its independent zero result");
        ledger.expected_pc = boot + 3;
      } else if (ledger.expected_pc == boot + 3) {
        ++ledger.adds;
        require(dsp.cpu().registers().a == ledger.adds * accumulator_unit,
                "Native ADD has the independently counted full accumulator result");
        ledger.expected_pc = boot + 4;
      } else {
        ledger.expected_pc = boot + 3;
      }
    }
    trace.push_back({.native = native(),
                     .issued = result.issued_address,
                     .native_pc = result.native_pc,
                     .captured = result.captured,
                     .started = result.instruction_started,
                     .retired = result.native && result.native->retired});
  }

  CpuStep step() {
    const auto result = mcu.cpu().step();
    runtime->end_instruction();
    return result;
  }

  void idle(const unsigned periods) {
    require(mcu.advance_wait(periods) == periods, "Requested actual native idle periods complete");
    runtime->end_wait();
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
    const auto cycles = common < recovered ? 0 : (common - recovered) / dsp_period;
    const auto reset = common < recovered ? common : recovered;
    require(runtime->end_ticks() == common && ledger.cycles == cycles &&
                runtime->completed_cycles() == cycles &&
                dsp.clock().elapsed_ticks() == reset + (cycles * dsp_period) &&
                dsp.clock().hardware_reset_ticks() == reset &&
                dsp.clock().timer_clk2_edges() == cycles / 2,
            "Native reset/recovery,MCU,DSP effects and CLK2 retain independent absolute clocks");
  }

  McuMachine mcu;
  OwnDspMachine dsp{{.extal_hz = oscillator_hz,
                     .timebase_hz = common_hz,
                     .switch_timing = DspClockSwitch::reference_cycle_boundary}};
  std::unique_ptr<McuDspRuntime> runtime;
  bool failing;
  std::uint64_t recovered = 0;
  Ledger ledger;
  std::vector<CycleTrace> trace;
};

struct Saved {
  McuDspRuntimeCheckpoint checkpoint;
  Ledger ledger;
  NativeTrace native;
  std::uint64_t mcu_ticks;
};

Saved capture(Fixture &f) {
  return {.checkpoint = f.runtime->checkpoint(),
          .ledger = f.ledger,
          .native = f.native(),
          .mcu_ticks = f.mcu.run_clock().elapsed_ticks()};
}

void restore(Fixture &f, const Saved &saved) {
  f.runtime.reset();
  f.runtime = std::make_unique<McuDspRuntime>(saved.checkpoint, f.mcu, f.dsp);
  f.ledger = saved.ledger;
  f.trace.clear();
  f.attach();
  require(f.native() == saved.native && f.mcu.run_clock().elapsed_ticks() == saved.mcu_ticks &&
              f.mcu.bus().read(mcu_ram) == initial_byte &&
              f.dsp.bus().read({.space = DspSpace::x, .address = dsp_ram}) == initial_word &&
              !f.runtime->faulted(),
          "Joint restore recovers full native CPU/pending effects,clocks and deep bus contents");
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
  require(std::vector<CycleTrace>(suffix.begin() + static_cast<std::ptrdiff_t>(saved.ledger.cycles),
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
  f.runtime->begin_system_reset(McuResetSource::external, 3);
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
    f.runtime->end_system_reset();
    const auto expected = 50U + ((reset_edges + (hold ? 1U : 0U)) * 5U) - phase + 50U;
    require(f.mcu.run_clock().elapsed_ticks() == expected && f.mcu.cpu().state().pc == entry &&
                f.mcu.cpu().state().ccr == hc12::reset_ccr &&
                f.mcu.run_clock().native_epoch() == 2 && f.ledger.adds > saved.ledger.adds,
            "Native reset and five vector periods preserve SRAM/DSP with literal OSC phase");
    f.time();
  }
}

void check_fault_prefix(const Fixture &f, const std::string &failure) {
  constexpr unsigned reset_sr = 0xc00300;
  constexpr unsigned bypass_omr = 0x305;
  constexpr unsigned linear_modifier = 0xffffff;
  constexpr unsigned failed_pc = boot + 4 + fault_delay;
  constexpr std::uint64_t completed = (2 * fault_delay) + 9;
  constexpr unsigned attempt_ticks = 5310;
  constexpr unsigned mcu_bus_periods = 58;
  constexpr unsigned mcu_idle_periods = 37;
  constexpr unsigned mcu_ticks = 580;
  dsp56300::Registers expected{.x = 7, .pc = failed_pc, .sr = reset_sr, .omr = bypass_omr};
  expected.r[0] = stack;
  expected.m.fill(linear_modifier);
  const auto pending = f.dsp.cpu().pending_instruction();
  require(
      failure.contains("attempted-ticks=" + std::to_string(attempt_ticks)) &&
          failure.contains("pc=" + hex(failed_pc, 6)) &&
          failure.contains("Unmapped DSP memory write at Y:0x002000") &&
          f.dsp.cpu().registers() == expected &&
          f.dsp.cpu().progress() == dsp56300::Progress{.instructions = fault_delay + 2,
                                                       .estimated_cycles = fault_delay + 4} &&
          pending && pending->effect.kind == dsp56300::EffectKind::write &&
          pending->effect.space == dsp56300::Space::y && pending->effect.address == stack &&
          pending->effect.value == 7 && pending->effect.pc == failed_pc &&
          f.ledger.cycles == completed && f.runtime->completed_cycles() == completed &&
          f.dsp.clock().elapsed_ticks() == f.recovered + (completed * dsp_period) &&
          f.dsp.clock().timer_clk2_edges() == completed / 2 &&
          f.mcu.run_clock().bus_cycles() == mcu_bus_periods &&
          f.mcu.idle_bus_cycles() == mcu_idle_periods &&
          f.mcu.run_clock().elapsed_ticks() == mcu_ticks,
      "Fault replay retains independent complete native registers,actual store and both clocks");
}

void fault_recovery() {
  Fixture f({.failing = true});
  f.enter_wait();
  const auto saved = capture(f);
  std::string expected;
  std::vector<CycleTrace> suffix;
  for (unsigned repeat = 0; repeat < 2; ++repeat) {
    restore(f, saved);
    const auto failure = rejects([&] { f.idle(64); });
    require(
        failure.contains("MCU wait advance pc=0x800f") && f.runtime->faulted() &&
            !rejects([&] { static_cast<void>(f.runtime->checkpoint()); }).empty(),
        "Actual reserved-Y store remains a contextual native failure and quarantines capture: " +
            failure);
    check_fault_prefix(f, failure);
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

void fresh_owner() {
  constexpr unsigned idle_cut = 17;
  Fixture left;
  left.enter_wait();
  left.idle(idle_cut);
  const auto saved = capture(left);
  left.idle(7);
  left.wake();
  const std::vector<CycleTrace> expected(
      left.trace.begin() + static_cast<std::ptrdiff_t>(saved.ledger.cycles), left.trace.end());
  Fixture right;
  restore(right, saved);
  left.mcu.bus().write(mcu_ram, mutation);
  left.dsp.bus().write({.space = DspSpace::x, .address = dsp_ram, .value = mutation});
  right.idle(7);
  right.wake();
  require(right.trace == expected && right.mcu.bus().read(mcu_ram) == initial_byte &&
              right.dsp.bus().read({.space = DspSpace::x, .address = dsp_ram}) == initial_word &&
              left.mcu.bus().read(mcu_ram) == mutation &&
              left.dsp.bus().read({.space = DspSpace::x, .address = dsp_ram}) == mutation,
          "Fresh native owners retain the complete suffix and deep independent MCU/DSP memories");
}

void external_contract() {
  Fixture f;
  const auto saved = capture(f);
  const auto before = f.native();
  const auto cpu = registers(f.mcu.cpu().state());
  unsigned queries = 0;
  DspRuntimeEvents different{
      .identity = std::make_shared<const bool>(false),
      .next_ticks = [&]() -> std::optional<std::uint64_t> {
        ++queries;
        return std::nullopt;
      },
      .observe =
          [](std::uint64_t) { throw Fault("Unexpected event on rejected checkpoint owner"); },
      .order = DspPeerOrder::dsp_first,
      .maximum_events_per_period = 1};
  require(rejects([&] {
            const McuDspRuntime rejected(saved.checkpoint, f.mcu, f.dsp, different);
          }).contains("matching external identity and contract") &&
              queries == 0 && f.native() == before && registers(f.mcu.cpu().state()) == cpu &&
              f.mcu.run_clock().elapsed_ticks() == saved.mcu_ticks && !f.runtime->faulted(),
          "A different external owner rejects before queries or either native machine changes");
}

void active_owner() {
  Fixture f;
  const auto saved = capture(f);
  unsigned rejected = 0;
  f.mcu.set_cycle_observer([&](const auto period) {
    f.runtime->begin_cycle(period, false, cycle_inputs, [&](const auto &result) {
      const auto before = f.native();
      const auto cpu = registers(f.mcu.cpu().state());
      require(rejects([&] { static_cast<void>(f.runtime->checkpoint()); }).contains("inactive") &&
                  rejects([&] {
                    const McuDspRuntime replacement(saved.checkpoint, f.mcu, f.dsp);
                  }).contains("active MCU execution") &&
                  f.native() == before && registers(f.mcu.cpu().state()) == cpu,
              "Active capture/replacement rejects before either native machine changes");
      ++rejected;
      f.record(result);
    });
  });
  static_cast<void>(f.step());
  require(rejected > 0 && !f.runtime->faulted(),
          "Caught reentrant requests preserve actual outer execution");
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
  require(rejects([&] {
            static_cast<void>(f.runtime->checkpoint());
          }).contains("consumed operation boundary"),
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
  f.runtime.reset();
  f.runtime = std::make_unique<McuDspRuntime>(saved.checkpoint, f.mcu, f.dsp);
  require(f.mcu.advance_wait(1) == 1 && external_calls == 1,
          "Native restore drops external wiring and observer callbacks");
  require(rejects([&] {
            static_cast<void>(f.runtime->checkpoint());
          }).contains("consumed native period"),
          "Missing reattachment cannot silently relabel an unobserved native idle period");
  restore(f, saved);
  f.wake();
}

void unknown_reset_pad() {
  Fixture f;
  f.runtime->begin_system_reset(McuResetSource::external, 3);
  f.attach();
  f.mcu.advance_system_reset(reset_edges - 1, McuResetPad::unknown);
  const auto saved = capture(f);
  require(
      rejects([&] { f.mcu.advance_system_reset(1, McuResetPad::unknown); }).contains("RESET pad") &&
          f.runtime->faulted() && f.native() == saved.native &&
          f.mcu.run_clock().elapsed_ticks() == saved.mcu_ticks,
      "Unknown RESET sampling rejects before either native clock advances");
  restore(f, saved);
  require(f.mcu.advance_system_reset(1, McuResetPad::high) == 1,
          "Trusted restore permits a later explicitly known reset-pad observation");
  f.runtime->end_system_reset();
  require(f.mcu.cpu().state().pc == entry, "Recovered reset fetches the actual original vector");
  f.time();
}

void input_contract() {
  Fixture source;
  const auto saved = capture(source);
  Fixture different({.phase = 1});
  const auto native = different.native();
  const auto cpu = registers(different.mcu.cpu().state());
  const auto ticks = different.mcu.run_clock().elapsed_ticks();
  require(rejects([&] {
            const McuDspRuntime rejected(saved.checkpoint, different.mcu, different.dsp);
          }).contains("incompatible reference run contract") &&
              different.native() == native && registers(different.mcu.cpu().state()) == cpu &&
              different.mcu.run_clock().elapsed_ticks() == ticks && !different.runtime->faulted(),
          "An incompatible MCU clock contract rejects before either native target changes");
  static_cast<void>(different.step());
}

void backend_failure(const std::filesystem::path &temporary) {
  const auto backend = temporary / "joint-checkpoint.sla";
  std::filesystem::copy_file(BLOFELD_DEFAULT_SLA, backend);
  Fixture f({.backend = backend});
  f.enter_wait();
  const auto saved = capture(f);
  f.idle(1);
  const auto before = f.native();
  const auto cpu = registers(f.mcu.cpu().state());
  {
    std::ofstream changed(backend, std::ios::binary | std::ios::app);
    changed.exceptions(std::ios::failbit | std::ios::badbit);
    changed.put('x');
  }
  f.runtime.reset();
  require(!rejects([&] {
             f.runtime = std::make_unique<McuDspRuntime>(saved.checkpoint, f.mcu, f.dsp);
           }).empty() &&
              !f.runtime && f.native() == before && registers(f.mcu.cpu().state()) == cpu,
          "Changed backend refuses a fresh joint owner before either native target changes");
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
    fresh_owner();
    external_contract();
    active_owner();
    incomplete_offer(false);
    incomplete_offer(true);
    external_observers();
    unknown_reset_pad();
    input_contract();
    backend_failure(arguments[1]);
    std::cout
        << "45 dsp56300 joint checkpoint scenarios passed:6 run,4 WAI,25 reset,1 clone,9 guards\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
