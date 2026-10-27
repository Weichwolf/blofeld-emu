// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_fetch_schedule.hpp"
#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_fetch_run_fixture.hpp"
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
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned oscillator_hz = 12000000;
constexpr unsigned common_hz = 540000000;
constexpr unsigned epoch = 450;
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

McuDspFetchInputs inputs(const McuDspWaitOrder order) {
  return {.dsp = {.extal_hz = oscillator_hz,
                  .timebase_hz = common_hz,
                  .timing = DspFetchTiming::reference_cycle_order,
                  .execution = DspReferenceExecution::staged_register_boundaries,
                  .pipeline = DspFetchPipeline::automatic_memory},
          .epoch_ticks = epoch,
          .shared_write_order = McuDspTieOrder::dsp_before_mcu,
          .maximum_dsp_cycles_per_period = 16,
          .wait_order = order};
}

DspMachine &initialize(DspMachine &dsp, const bool fault) {
  DspProgram code;
  if (fault) {
    code.immediate("r0", stack);
    code.immediate("x0", 7);
    constexpr unsigned padding = 28;
    for (unsigned i = 0; i < padding; ++i) { code.emit("nop"); }
    code.emit("move x0,y:(r0)");
  } else {
    code.immediate("x1", 1);
    code.emit("clr a");
    code.emit("add x1,a");
    code.word(branch_back);
  }
  constexpr unsigned fetch_padding = 32;
  for (unsigned i = 0; i < fetch_padding; ++i) { code.emit("nop"); }
  code.install(dsp.bus());
  dsp.bus().pll().observe_pinit(false);
  dsp.cpu().reset_spi_boot();
  static_cast<void>(
      dsp.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return dsp;
}

struct Fixture {
  explicit Fixture(const bool fault = false,
                   const bool enter_wait = true,
                   const McuDspWaitOrder order = McuDspWaitOrder::dsp_before_mcu)
      : schedule(inputs(order), mcu, initialize(dsp, fault)), failing(fault) {
    mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
    mcu.set_cycle_observer([this](const auto cycle) {
      ++cpu_periods;
      schedule.begin_cycle(
          cycle,
          false,
          [] { return fetch_clock::input(); },
          [this](const auto &result) { record(result); });
    });
    mcu.set_wait_period_observer([this](const auto period) {
      ++periods;
      schedule.begin_wait_period(
          period,
          [] { return fetch_clock::input(); },
          [this](const auto &result) { record(result); });
    });
    if (!enter_wait) { return; }
    for (unsigned i = 0; i < 6; ++i) { static_cast<void>(step()); }
    require(mcu.cpu().waiting() && mcu.run_clock().bus_cycles() == initial_cycles &&
                mcu.run_clock().elapsed_ticks() == initial_ticks && cpu_periods == 16,
            "Literal native setup periods reach actual WAI and its frame");
  }

  void record(const DspFetchCycleResult &result) {
    require(result.completed_cycles == cycles + 1 && result.period_ticks == dsp_period &&
                result.completion_ticks == result.completed_cycles * dsp_period,
            "Each automatic DSP pipeline period follows the independent reset PLL clock");
    ++cycles;
    const auto end = epoch + result.completion_ticks;
    if (edge && end == *edge) {
      require(!mcu.cpu().pending_interrupt(),
              "DSP cycle precedes the tied OSC-derived MCU IRQ edge");
      saw_edge = true;
    }
    if (assert_irq) {
      mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 1});
      assert_irq = false;
    }
    if (!result.pipeline) { throw Fault("Automatic owner must retain its actual pipeline result"); }
    const auto &phase = *result.pipeline;
    if (failing || !phase.retirement_published) { return; }
    if (!phase.execute2) { throw Fault("Native publication must retain its actual E2 word"); }
    const auto pc = phase.execute2->address;
    require(pc == expected_pc,
            "Genuine native retirements follow own literal setup/loop addresses");
    if (pc == boot) {
      expected_pc = boot + 2;
    } else if (pc == boot + 2) {
      require(dsp.cpu().state().a == 0, "Native CLR has the independent zero accumulator result");
      expected_pc = boot + 3;
    } else if (pc == boot + 3) {
      ++adds;
      require(dsp.cpu().state().a == adds * accumulator_unit,
              "Each native ADD advances the independently expected full accumulator");
      expected_pc = boot + 4;
    } else {
      expected_pc = boot + 3;
    }
  }

  CpuStep step() {
    const auto result = mcu.cpu().step();
    schedule.end_instruction();
    return result;
  }

  void check_time() const {
    const auto common = mcu.run_clock().elapsed_ticks() * 9;
    require(schedule.end_ticks() == common && cycles == (common - epoch) / dsp_period &&
                schedule.run().completed_cycles() == cycles &&
                schedule.run().elapsed_ticks() == cycles * dsp_period &&
                schedule.run().timer_clk2_edges() == cycles / 2,
            "All actual MCU and pipeline clocks agree with the independent integer ledger");
  }

  McuMachine mcu{image(), BLOFELD_DEFAULT_SLA, mcu_contract()};
  DspMachine dsp;
  McuDspFetchSchedule schedule;
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
  f.schedule.end_wait();
  f.check_time();
  f.edge = rti_endpoint;
  require(f.mcu.advance_wait(total) == total - partial,
          "OSC1024-edge RTI stops exactly at512OSC or2560PHY native idle periods");
  f.schedule.end_wait();
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
  f.schedule.end_wait();
  const auto dsp = f.dsp.cpu().debug_state();
  const auto fetch = f.dsp.reference_fetch();
  const auto cycles = f.cycles;
  if (kind == 0) { irq(); }
  f.mcu.set_wait_observer([&](auto /*cycle*/) {
    ++pre;
    if (kind == 1) { irq(); }
    if (kind == 2) { f.mcu.bus().set_pin_drive(McuPort::t, {.mask = 1, .levels = 1}); }
  });
  require(f.mcu.advance_wait(10) == 0 && f.periods == 1 && pre == (kind == 0 ? 0U : 1U),
          "Existing,pre-observed or sampled timer IRQ cancels before the committed peer hook");
  f.schedule.end_wait();
  require(f.mcu.idle_bus_cycles() == 1 && f.dsp.cpu().debug_state() == dsp &&
              f.dsp.reference_fetch() == fetch && f.cycles == cycles,
          "Cancelled offers change neither actual pipeline/native CPU state nor elapsed clocks");
  f.check_time();
}

void committed_irq() {
  Fixture f;
  require(f.mcu.advance_wait(1) == 1, "Initial native idle period completes");
  f.schedule.end_wait();
  f.assert_irq = true;
  require(f.mcu.advance_wait(10) == 1,
          "A pin IRQ raised by a committed DSP period cannot cancel it");
  f.schedule.end_wait();
  require(f.periods == 2 && f.mcu.idle_bus_cycles() == 2 && f.mcu.cpu().waiting(),
          "The actual committed MCU period finishes before native wake selection");
  f.check_time();
  require(f.step().interrupt_vector == irq_vector,
          "Actual external pin IRQ selects the native wake vector");
  check_frame(f.mcu);
}

void idle_fault() {
  Fixture f(true);
  const auto error = rejects([&] { static_cast<void>(f.mcu.advance_wait(64)); });
  require(error.contains("Scheduled pipeline idle peer mcu-pc=0x800f") &&
              error.contains("Unmapped DSP memory write at Y:0x002000") && f.schedule.faulted() &&
              f.mcu.timing_faulted() && f.schedule.run().faulted(),
          "Real native idle store failure retains address/context and quarantines both peers: " +
              error);
  const auto consumed = f.mcu.run_clock().elapsed_ticks() * 9;
  const auto dsp_end = epoch + f.schedule.run().elapsed_ticks();
  constexpr unsigned mcu_period = 90;
  require(f.schedule.end_ticks() == consumed + mcu_period && dsp_end >= consumed &&
              dsp_end < f.schedule.end_ticks() &&
              f.mcu.run_clock().bus_cycles() == initial_cycles + f.mcu.idle_bus_cycles(),
          "A partial failing idle interval charges only genuinely completed DSP and MCU periods");
  static_cast<void>(rejects([&] { f.schedule.end_wait(); }));
  static_cast<void>(rejects([&] { static_cast<void>(f.mcu.checkpoint()); }));
  static_cast<void>(rejects([&] { static_cast<void>(f.dsp.checkpoint()); }));
  static_cast<void>(rejects([&] {
    static_cast<void>(
        f.dsp.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  }));
  check_frame(f.mcu);
}

void boundary_guards() {
  Fixture before(false, false);
  const auto cpu = before.mcu.cpu().state();
  const auto error = rejects([&] {
    before.schedule.begin_wait_period({.context = {.pc = cpu.pc, .ccr = cpu.ccr}},
                                      [] { return fetch_clock::input(); });
  });
  require(error.contains("requires native WAI") && before.schedule.faulted() && before.cycles == 0,
          "A fabricated idle offer cannot clock either peer before genuine WAI");
  Fixture omitted;
  require(omitted.mcu.advance_wait(1) == 1, "A native idle slice exists to finish");
  require(rejects([&] { omitted.schedule.end_instruction(); }).contains("explicit idle completion"),
          "An instruction boundary cannot consume an unfinished native idle slice");
  Fixture wrong_context;
  const auto state = wrong_context.mcu.cpu().state();
  require(rejects([&] {
            wrong_context.schedule.begin_wait_period(
                {.context = {.pc = resume + 1U, .ccr = state.ccr}},
                [] { return fetch_clock::input(); });
          }).contains("requires native WAI"),
          "A mismatched native wait context cannot advance the peer");
  Fixture unqualified(false, false, McuDspWaitOrder::unqualified);
  for (unsigned i = 0; i < 5; ++i) { static_cast<void>(unqualified.step()); }
  require(
      rejects([&] {
        static_cast<void>(unqualified.step());
      }).contains("Unqualified MCU/DSP fetch WAI boundary") &&
          unqualified.mcu.cpu().waiting() && unqualified.schedule.faulted() &&
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
  boundary_guards();
}
}

int main() {
  try {
    run();
    std::cout << "Native WAI/RTI and automatic DSP pipeline timing/failure cases passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
