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
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
constexpr unsigned oscillator_hz = 12000000;
constexpr unsigned common_hz = 540000000;
constexpr unsigned boot = 0xff0000;
constexpr unsigned recovery_end = 540;
constexpr unsigned dsp_period = 45;
constexpr unsigned entry = 0x8000;
constexpr unsigned resume = 0x800f;
constexpr unsigned handler = 0x9000;
constexpr unsigned stack = 0x2000;
constexpr unsigned wait_stack = stack - 9;
constexpr unsigned unmasked = 0xc0;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned enabled = 0x80;
constexpr unsigned timer_gate = 0x40;
constexpr unsigned rti_vector = 0xfff0;
constexpr unsigned irq_vector = 0xfff2;
constexpr unsigned initial_cycles = 21;
constexpr unsigned initial_ticks = 210;
constexpr unsigned rti_endpoint = 47970;
constexpr unsigned counter = 0x44;
constexpr unsigned capture_edges = 0x4b;
constexpr unsigned capture_interrupts = 0x4c;
constexpr unsigned final_idle_ticks = 5330;
constexpr unsigned foreground_d = 0x1234;
constexpr unsigned foreground_x = 0x4567;
constexpr unsigned foreground_y = 0x6789;
constexpr unsigned foreground_periods = 34;
constexpr unsigned wake_periods = 18;
// Independent FM short BRA displacement-1 encoding,already qualified by the core.
constexpr unsigned branch_back = 0x050fdf;
constexpr std::uint64_t accumulator_unit = std::uint64_t{1} << 24U;

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected explicit native pipeline WAI rejection");
}

McuFlashImage image() {
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  constexpr unsigned nop = 0xa7;
  constexpr unsigned protection = 0x7f0d;
  constexpr unsigned security = 0x7f0f;
  constexpr unsigned unsecured = 0xfe;
  // Own LDS/LDD/LDX/LDY;ANDCC #ef;WAI;NOP. Independent costs2+2+2+2+1+7.
  constexpr std::array<std::uint8_t, 16> code{
      0xcf, 0x20, 0, 0xcc, 0x12, 0x34, 0xce, 0x45, 0x67, 0xcd, 0x67, 0x89, 0x10, 0xef, 0x3e, nop};
  // Own LDAA #80;STAA CRGFLG;RTI. Independent costs1+3+8.
  constexpr std::array<std::uint8_t, 6> isr{0x86, 0x80, 0x7a, 0, 0x37, 0x0b};
  bytes.fill(nop);
  for (unsigned i = 0; i < code.size(); ++i) { bytes.at(i) = code[i]; }
  for (unsigned i = 0; i < isr.size(); ++i) { bytes.at(handler - entry + i) = isr[i]; }
  for (const unsigned vector : {rti_vector, irq_vector}) {
    bytes.at(vector - entry) = handler >> 8U;
    bytes.at(vector - entry + 1) = 0;
  }
  bytes[protection] = byte_mask;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  return McuFlashImage::from_known(bytes);
}

McuRunContract mcu_contract() {
  return {.clock = {.oscillator_hz = oscillator_hz,
                    .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = McuRunMemory::unstretched_internal};
}

void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

McuDspRuntimeInputs inputs(const bool dsp_first) {
  return {.timing = OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts,
          .maximum_cycles_per_period = 16,
          .wait_dsp_first = dsp_first,
          .initial_reset = OwnDspResetInputs{
              .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}};
}

DspDmaInputs pins() {
  return {.known_requests = 15,
          .timing = DspDmaTiming::reference_sampled_cycles,
          .grant = DspDmaGrant::available};
}

void initialize(OwnDspMachine &dsp, const bool fault) {
  // DSP56300FM literal encodings: MOVE #>1,X1;CLR A;ADD X1,A;BRA *-1.
  constexpr unsigned immediate_x1 = 0x45f400;
  constexpr unsigned clear_a = 0x200013;
  constexpr unsigned add_x1 = 0x200060;
  constexpr unsigned immediate_r0 = 0x60f400;
  constexpr unsigned immediate_x0 = 0x44f400;
  constexpr unsigned store_x0_y_r0 = 0x4c6000;
  constexpr unsigned padding = 28;
  constexpr unsigned rom_words = 128;
  std::array<std::uint32_t, rom_words> code{};
  if (fault) {
    code[0] = immediate_r0;
    code[1] = stack;
    code[2] = immediate_x0;
    code[3] = 7;
    code[4 + padding] = store_x0_y_r0;
  } else {
    code[0] = immediate_x1;
    code[1] = 1;
    code[2] = clear_a;
    code[3] = add_x1;
    code[4] = branch_back;
  }
  dsp.bus().install_rom({.space = DspSpace::p, .start = boot, .words = code});
}

struct Observation {
  dsp56300::Registers registers;
  dsp56300::Progress progress;
  std::optional<dsp56300::InstructionEffect> pending;
  std::optional<std::uint32_t> issued;
  std::uint64_t completion;
  bool captured;
  bool retired;
  bool operator==(const Observation &) const = default;
};

struct Ledger {
  std::uint64_t cycles;
  std::uint64_t adds;
  unsigned expected_pc;
  unsigned cpu_periods;
  unsigned periods;
};

struct Fixture {
  explicit Fixture(const bool fault = false,
                   const bool enter_wait = true,
                   const bool dsp_first = true)
      : failing(fault) {
    initialize(dsp, fault);
    runtime = std::make_unique<McuDspRuntime>(inputs(dsp_first), mcu, dsp);
    runtime->release_dsp_reset();
    mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
    attach();
    if (!enter_wait) { return; }
    for (unsigned i = 0; i < 6; ++i) { static_cast<void>(step()); }
    require(mcu.cpu().waiting() && mcu.run_clock().bus_cycles() == initial_cycles &&
                mcu.run_clock().elapsed_ticks() == initial_ticks && cpu_periods == 16,
            "Literal native setup periods reach actual WAI and its frame");
  }

  void attach() {
    mcu.set_cycle_observer([this](const auto cycle) {
      ++cpu_periods;
      runtime->begin_cycle(cycle, false, pins, [this](const auto &result) { record(result); });
    });
    mcu.set_wait_period_observer([this](const auto period) {
      ++periods;
      runtime->begin_wait_period(period, pins, [this](const auto &result) { record(result); });
    });
  }

  void record(const DspRuntimeCycle &cycle) {
    const auto &result = cycle.result;
    ++cycles;
    require(cycle.cycle + 1 == cycles && cycle.completion_ticks - cycle.start_ticks == dsp_period &&
                cycle.completion_ticks == recovery_end + (cycles * dsp_period),
            "Every native DSP effect period follows actual held reset and recovery clocks");
    const auto pending = dsp.cpu().pending_instruction();
    trace.push_back({.registers = dsp.cpu().registers(),
                     .progress = dsp.cpu().progress(),
                     .pending = pending ? std::optional{pending->effect} : std::nullopt,
                     .issued = result.issued_address,
                     .completion = cycle.completion_ticks,
                     .captured = result.captured,
                     .retired = result.native && result.native->retired});
    const auto end = cycle.completion_ticks;
    if (edge && end == *edge) {
      require(!mcu.cpu().pending_interrupt(),
              "DSP cycle precedes the tied OSC-derived MCU IRQ edge");
      saw_edge = true;
    }
    if (assert_irq) {
      mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 1});
      assert_irq = false;
    }
    if (failing || !result.native || !result.native->retired) { return; }
    if (!result.native_pc) {
      throw Fault("Native retirement must retain its actual instruction PC");
    }
    const auto pc = *result.native_pc;
    require(pc == expected_pc,
            "Genuine native retirements follow own literal setup/loop addresses");
    if (pc == boot) {
      expected_pc = boot + 2;
    } else if (pc == boot + 2) {
      require(dsp.cpu().registers().a == 0,
              "Native CLR has the independent zero accumulator result");
      expected_pc = boot + 3;
    } else if (pc == boot + 3) {
      ++adds;
      require(dsp.cpu().registers().a == adds * accumulator_unit,
              "Each native ADD advances the independently expected full accumulator");
      expected_pc = boot + 4;
    } else {
      expected_pc = boot + 3;
    }
  }

  CpuStep step() {
    const auto result = mcu.cpu().step();
    runtime->end_instruction();
    return result;
  }

  void check_time() const {
    const auto common = mcu.run_clock().elapsed_ticks() * 9;
    require(runtime->end_ticks() == common && cycles == (common - recovery_end) / dsp_period &&
                runtime->completed_cycles() == cycles &&
                dsp.clock().elapsed_ticks() == recovery_end + (cycles * dsp_period) &&
                dsp.clock().hardware_reset_ticks() == recovery_end &&
                dsp.clock().timer_clk2_edges() == cycles / 2,
            "Real MCU,held/reset-recovery,DSP effect periods and CLK2 retain independent clocks");
  }

  [[nodiscard]] Ledger ledger() const {
    return {.cycles = cycles,
            .adds = adds,
            .expected_pc = expected_pc,
            .cpu_periods = cpu_periods,
            .periods = periods};
  }

  void restore(const McuDspRuntimeCheckpoint &saved, const Ledger saved_ledger) {
    runtime.reset();
    runtime = std::make_unique<McuDspRuntime>(saved, mcu, dsp);
    cycles = saved_ledger.cycles;
    adds = saved_ledger.adds;
    expected_pc = saved_ledger.expected_pc;
    cpu_periods = saved_ledger.cpu_periods;
    periods = saved_ledger.periods;
    trace.clear();
    saw_edge = false;
    attach();
  }

  McuMachine mcu{image(), BLOFELD_DEFAULT_SLA, mcu_contract()};
  OwnDspMachine dsp{{.extal_hz = oscillator_hz,
                     .timebase_hz = common_hz,
                     .switch_timing = DspClockSwitch::reference_cycle_boundary}};
  std::unique_ptr<McuDspRuntime> runtime;
  std::vector<Observation> trace;
  bool failing;
  unsigned cpu_periods = 0;
  unsigned periods = 0;
  bool assert_irq = false;
  bool saw_edge = false;
  std::optional<std::uint64_t> edge;
  std::uint64_t cycles = 0;
  std::uint64_t adds = 0;
  unsigned expected_pc = boot;
};

void check_frame(McuMachine &mcu) {
  constexpr std::array<unsigned, 9> frame{unmasked, 0x34, 0x12, 0x45, 0x67, 0x67, 0x89, 0x80, 0x0f};
  for (unsigned i = 0; i < frame.size(); ++i) {
    require(mcu.bus().read(static_cast<std::uint16_t>(wait_stack + i)) == frame[i],
            "Every actual native WAI stack byte matches the independent NXP expectation");
  }
}

void select_phy(McuBus &bus) {
  bus.write(hc12::clock_select, 0);
  bus.write(hc12::usb_control, timer_gate);
  bus.observe_phy({.edges = 15});
  bus.write(hc12::clock_select, enabled);
}

void rti_wake(const bool phy, const bool gated) {
  Fixture f;
  check_frame(f.mcu);
  if (phy) { select_phy(f.mcu.bus()); }
  f.mcu.bus().write(hc12::tscr1, enabled | (gated ? timer_gate : 0));
  f.mcu.bus().write(hc12::rti_control, 0x10);
  f.mcu.bus().write(hc12::crg_interrupts, enabled);
  const unsigned total = phy ? 2560 : 512;
  constexpr unsigned partial = 17;
  require(f.mcu.advance_wait(partial) == partial, "Actual bounded initial idle slice completes");
  f.runtime->end_wait();
  f.check_time();
  const auto cut = f.runtime->checkpoint();
  const auto ledger = f.ledger();
  const auto registers = f.dsp.cpu().registers();
  const auto progress = f.dsp.cpu().progress();
  std::vector<Observation> expected;
  for (unsigned suffix = 0; suffix < 2; ++suffix) {
    f.restore(cut, ledger);
    require(f.mcu.cpu().waiting() && f.dsp.cpu().registers() == registers &&
                f.dsp.cpu().progress() == progress,
            "Joint idle restoration lost actual WAI or continuing native computation");
    check_frame(f.mcu);
    f.edge = rti_endpoint;
    require(f.mcu.advance_wait(total) == total - partial,
            "OSC1024-edge RTI stops exactly at512OSC or2560PHY native idle periods");
    f.runtime->end_wait();
    require(f.saw_edge && f.periods == total && f.cpu_periods == 16 &&
                f.mcu.idle_bus_cycles() == total &&
                f.mcu.run_clock().bus_cycles() == initial_cycles + total &&
                f.mcu.run_clock().elapsed_ticks() == final_idle_ticks,
            "Both MCU clock routes reach the same literal RTI endpoint while DSP continues");
    f.check_time();
    const auto entered = f.step();
    require(entered.interrupt_vector == rti_vector && entered.stack_reused &&
                !entered.instruction_executed && f.mcu.cpu().state().sp == wait_stack,
            "Actual six-period native wake reuses the genuine WAI frame");
    check_frame(f.mcu);
    for (unsigned i = 0; i < 3; ++i) { static_cast<void>(f.step()); }
    const auto state = f.mcu.cpu().state();
    const auto timer = f.mcu.bus().read_memory({.address = counter, .size = 2});
    require(
        state.pc == resume && state.sp == stack && state.d == foreground_d &&
            state.x == foreground_x && state.y == foreground_y && state.ccr == unmasked &&
            f.cpu_periods == foreground_periods &&
            timer == (gated ? wake_periods : total + wake_periods) &&
            !f.mcu.cpu().pending_interrupt() && f.adds != 0,
        "Actual handler/RTI restore literal foreground registers and respect the native TIM gate");
    f.check_time();
    if (suffix == 0) {
      expected = f.trace;
    } else {
      require(f.trace == expected,
              "Restored WAI/RTI changed a full native register,flag,pending effect or fetch clock");
    }
  }
}

void cancelled_offers(const unsigned kind) {
  Fixture f;
  unsigned pre = 0;
  const auto irq = [&] { f.mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 1}); };
  if (kind == 2) {
    f.mcu.bus().set_pin_drive(McuPort::t, {.mask = 1});
    f.mcu.set_timer_inputs(1);
    f.mcu.bus().write(hc12::tscr1, enabled);
    f.mcu.bus().write(capture_edges, 1);
    f.mcu.bus().write(capture_interrupts, 1);
  }
  require(f.mcu.advance_wait(1) == 1, "One native idle period precedes boundary input changes");
  f.runtime->end_wait();
  const auto dsp = f.dsp.cpu().registers();
  const auto progress = f.dsp.cpu().progress();
  const auto pending = f.dsp.cpu().pending_instruction();
  const auto cycles = f.cycles;
  if (kind == 0) { irq(); }
  f.mcu.set_wait_observer([&](auto /*cycle*/) {
    ++pre;
    if (kind == 1) { irq(); }
    if (kind == 2) { f.mcu.bus().set_pin_drive(McuPort::t, {.mask = 1, .levels = 1}); }
  });
  require(f.mcu.advance_wait(10) == 0 && f.periods == 1 && pre == (kind == 0 ? 0U : 1U),
          "Existing,pre-observed or sampled timer IRQ cancels before the committed peer hook");
  f.runtime->end_wait();
  require(f.mcu.idle_bus_cycles() == 1 && f.dsp.cpu().registers() == dsp &&
              f.dsp.cpu().progress() == progress && f.cycles == cycles,
          "Cancelled offers change neither actual pipeline/native CPU state nor elapsed clocks");
  if (pending) { static_cast<void>(f.dsp.candidate_checkpoint(pending->ticket)); }
  f.check_time();
}

void committed_irq() {
  Fixture f;
  require(f.mcu.advance_wait(1) == 1, "Initial native idle period completes");
  f.runtime->end_wait();
  f.assert_irq = true;
  require(f.mcu.advance_wait(10) == 1,
          "A pin IRQ raised by a committed DSP period cannot cancel it");
  f.runtime->end_wait();
  require(f.periods == 2 && f.mcu.idle_bus_cycles() == 2 && f.mcu.cpu().waiting(),
          "The actual committed MCU period finishes before native wake selection");
  f.check_time();
  require(f.step().interrupt_vector == irq_vector,
          "Actual external pin IRQ selects the native wake vector");
  check_frame(f.mcu);
}

std::string check_idle_fault(Fixture &f) {
  const auto error = rejects([&] { static_cast<void>(f.mcu.advance_wait(64)); });
  const auto pending = f.dsp.cpu().pending_instruction();
  constexpr unsigned store_pc = boot + 32;
  constexpr unsigned completed = 65;
  constexpr unsigned retired_before_store = 30;
  constexpr unsigned attempted_ticks = 3510;
  constexpr unsigned reset_sr = 0xc00300;
  constexpr unsigned bypass_omr = 0x305;
  constexpr unsigned linear_modifier = 0xffffff;
  constexpr std::uint64_t idle_periods = 17;
  constexpr unsigned mcu_period_ticks = 10;
  require(error.contains("MCU wait advance pc=0x800f") && error.contains("pc=0xff0020") &&
              error.contains("attempted-ticks=" + std::to_string(attempted_ticks)) &&
              error.contains("Unmapped DSP memory write at Y:0x002000") && f.runtime->faulted() &&
              f.mcu.timing_faulted() && f.dsp.clock().faulted() && pending &&
              pending->effect.kind == dsp56300::EffectKind::write &&
              pending->effect.space == dsp56300::Space::y && pending->effect.address == stack &&
              pending->effect.value == 7 && pending->effect.pc == store_pc &&
              f.dsp.cpu().progress().instructions == retired_before_store &&
              f.cycles == completed && f.runtime->completed_cycles() == completed,
          "Real native idle store failure retains address/context and quarantines both peers: " +
              error);
  const auto consumed = f.mcu.run_clock().elapsed_ticks() * 9;
  const auto dsp_end = f.dsp.clock().elapsed_ticks();
  constexpr unsigned mcu_period = 90;
  require(f.runtime->end_ticks() == consumed + mcu_period && dsp_end >= consumed &&
              dsp_end < f.runtime->end_ticks() &&
              f.mcu.run_clock().bus_cycles() == initial_cycles + f.mcu.idle_bus_cycles(),
          "A partial failing idle interval charges only genuinely completed DSP and MCU periods");
  dsp56300::Registers expected{.x = 7, .pc = store_pc, .sr = reset_sr, .omr = bypass_omr};
  expected.r[0] = stack;
  expected.m.fill(linear_modifier);
  const auto state = f.mcu.cpu().state();
  require(
      f.dsp.cpu().registers() == expected &&
          f.dsp.cpu().progress() ==
              dsp56300::Progress{.instructions = retired_before_store, .estimated_cycles = 32} &&
          f.mcu.cpu().waiting() && state.pc == resume && state.sp == wait_stack &&
          state.d == foreground_d && state.x == foreground_x && state.y == foreground_y &&
          state.ccr == unmasked && f.mcu.idle_bus_cycles() == idle_periods &&
          f.mcu.run_clock().bus_cycles() == initial_cycles + idle_periods &&
          f.mcu.run_clock().elapsed_ticks() == initial_ticks + (idle_periods * mcu_period_ticks) &&
          f.mcu.run_clock().oscillator_cycles() == (initial_cycles + idle_periods) * 2 &&
          dsp_end == recovery_end + (completed * dsp_period) &&
          f.dsp.clock().hardware_reset_ticks() == recovery_end &&
          f.dsp.clock().timer_clk2_edges() == completed / 2 &&
          f.runtime->end_ticks() == attempted_ticks,
      "Failed native WAI prefix retains independent complete registers and literal clock costs");
  static_cast<void>(rejects([&] { f.runtime->end_wait(); }));
  static_cast<void>(rejects([&] { static_cast<void>(f.mcu.checkpoint()); }));
  static_cast<void>(rejects([&] { static_cast<void>(f.runtime->checkpoint()); }));
  // Chip snapshots retain faults for diagnosis; they cannot resume execution.
  const auto failed_chip = f.dsp.checkpoint();
  OwnDspMachine clone({.extal_hz = oscillator_hz,
                       .timebase_hz = common_hz,
                       .switch_timing = DspClockSwitch::reference_cycle_boundary});
  clone.restore(failed_chip);
  require(clone.clock().faulted() && clone.clock().elapsed_ticks() == dsp_end &&
              clone.cpu().registers() == f.dsp.cpu().registers() &&
              clone.cpu().progress() == f.dsp.cpu().progress(),
          "Diagnostic chip restoration lost the actual failed execution state or became healthy");
  require(rejects([&] {
            static_cast<void>(clone.begin_fetch());
          }).contains("requires completed hardware reset"),
          "A restored failed chip cannot acquire fresh native execution authority");
  check_frame(f.mcu);
  return error;
}

void idle_fault() {
  Fixture f(true);
  const auto saved = f.runtime->checkpoint();
  const auto ledger = f.ledger();
  std::string expected;
  std::vector<Observation> trace;
  for (unsigned suffix = 0; suffix < 2; ++suffix) {
    f.restore(saved, ledger);
    require(!f.runtime->faulted() && !f.mcu.timing_faulted() && !f.dsp.clock().faulted(),
            "Healthy joint idle checkpoint must clear both prior failed owners");
    const auto error = check_idle_fault(f);
    if (suffix == 0) {
      expected = error;
      trace = f.trace;
    }
    require(error == expected && f.trace == trace,
            "Joint idle restore changed a native fault,effect,register or fetch/clock prefix");
  }
}

void empty_wait_guards() {
  Fixture empty;
  const auto registers = empty.dsp.cpu().registers();
  const auto progress = empty.dsp.cpu().progress();
  const auto size = empty.trace.size();
  require(empty.mcu.advance_wait(0) == 0, "Zero wait budget cannot offer a native period");
  empty.runtime->end_wait();
  require(empty.periods == 0 && empty.mcu.idle_bus_cycles() == 0 && empty.trace.size() == size &&
              empty.dsp.cpu().registers() == registers && empty.dsp.cpu().progress() == progress &&
              empty.mcu.run_clock().elapsed_ticks() == initial_ticks,
          "Closing an empty actual wait budget changed CPU state or consumed either clock");
  empty.check_time();
  check_frame(empty.mcu);
  static_cast<void>(empty.runtime->checkpoint());

  Fixture unfinished(false, false);
  for (unsigned i = 0; i < 5; ++i) { static_cast<void>(unfinished.step()); }
  require(unfinished.mcu.cpu().step().instruction_executed && unfinished.mcu.cpu().waiting(),
          "Unfinished instruction control must actually execute WAI and build its stack");
  require(rejects([&] { unfinished.runtime->end_wait(); }).contains("idle boundary is missing"),
          "Empty idle completion cannot consume an unfinished WAI instruction boundary");
}

void boundary_guards() {
  Fixture before(false, false);
  const auto cpu = before.mcu.cpu().state();
  const auto error = rejects([&] {
    before.runtime->begin_wait_period({.context = {.pc = cpu.pc, .ccr = cpu.ccr}}, pins);
  });
  require(error.contains("requires native WAI") && before.runtime->faulted() && before.cycles == 0,
          "A fabricated idle offer cannot clock either peer before genuine WAI");
  Fixture omitted;
  require(omitted.mcu.advance_wait(1) == 1, "A native idle slice exists to finish");
  require(rejects([&] {
            static_cast<void>(omitted.runtime->checkpoint());
          }).contains("consumed operation boundary"),
          "An unfinished actual idle slice cannot become a trusted joint snapshot");
  require(
      rejects([&] { omitted.runtime->end_instruction(); }).contains("correct reset/wait boundary"),
      "An instruction boundary cannot consume an unfinished native idle slice");
  Fixture wrong_context;
  const auto state = wrong_context.mcu.cpu().state();
  require(rejects([&] {
            wrong_context.runtime->begin_wait_period(
                {.context = {.pc = resume + 1U, .ccr = state.ccr}}, pins);
          }).contains("requires native WAI"),
          "A mismatched native wait context cannot advance the peer");
  Fixture unqualified(false, false, false);
  for (unsigned i = 0; i < 5; ++i) { static_cast<void>(unqualified.step()); }
  require(
      rejects([&] {
        static_cast<void>(unqualified.step());
      }).contains("correct reset/wait boundary") &&
          unqualified.mcu.cpu().waiting() && unqualified.runtime->faulted() &&
          unqualified.mcu.run_clock().bus_cycles() == initial_cycles,
      "Default unqualified policy still refuses genuine WAI without inventing a clock contract");
}

void run() {
  for (const bool phy : {false, true}) {
    for (const bool gated : {false, true}) { rti_wake(phy, gated); }
  }
  for (unsigned kind = 0; kind < 3; ++kind) { cancelled_offers(kind); }
  committed_irq();
  idle_fault();
  empty_wait_guards();
  boundary_guards();
}
}

int main() {
  try {
    run();
    std::cout << "Native MCU WAI/RTI:15 cases,8 RTI and2 fault replay suffixes passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
