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
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_dsp_schedule.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/mcu_run_clock.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>

namespace {
constexpr unsigned oscillator_hz = 12000000;
constexpr unsigned common_hz = 540000000;
constexpr unsigned epoch = 450;
constexpr unsigned native_unit_ticks = 45;
constexpr unsigned loop_ticks = 225;
constexpr unsigned absolute_jump = 0x0af080;
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
constexpr unsigned timer_vector = 0xffee;
constexpr unsigned initial_cycles = 21;
constexpr unsigned initial_ticks = 210;
constexpr unsigned partial_idle = 17;
constexpr unsigned rti_endpoint = 47970;
constexpr unsigned counter = 0x44;
constexpr unsigned capture_edges = 0x4b;
constexpr unsigned capture_interrupts = 0x4c;
constexpr unsigned final_idle_ticks = 5330;
constexpr unsigned foreground_d = 0x1234;
constexpr unsigned foreground_x = 0x4567;
constexpr unsigned foreground_y = 0x6789;
constexpr unsigned final_cpu_periods = 34;
constexpr unsigned wake_handler_periods = 18;
constexpr unsigned failed_dsp_ticks = 1440;
constexpr unsigned unqualified_dsp_units = 31;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw blofeld::Fault(message); }
}

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const blofeld::Fault &fault) { return fault.what(); }
  throw blofeld::Fault("Expected native paired-WAI rejection");
}

blofeld::McuFlashImage image() {
  std::array<std::uint8_t, blofeld::McuFlashImage::byte_count> bytes{};
  constexpr unsigned nop = 0xa7;
  constexpr unsigned protection = 0x7f0d;
  constexpr unsigned security = 0x7f0f;
  constexpr unsigned unsecured = 0xfe;
  // Own LDS/LDD/LDX/LDY immediates; ANDCC #ef; WAI; NOP.
  constexpr std::array<std::uint8_t, 16> code{
      0xcf, 0x20, 0, 0xcc, 0x12, 0x34, 0xce, 0x45, 0x67, 0xcd, 0x67, 0x89, 0x10, 0xef, 0x3e, nop};
  // Own LDAA #80; STAA CRGFLG; RTI. Independent costs: 1+3+8.
  constexpr std::array<std::uint8_t, 6> isr{0x86, 0x80, 0x7a, 0, 0x37, 0x0b};
  bytes.fill(nop);
  for (unsigned i = 0; i < code.size(); ++i) { bytes.at(i) = code[i]; }
  for (unsigned i = 0; i < isr.size(); ++i) { bytes.at(handler - entry + i) = isr[i]; }
  for (const unsigned vector : {rti_vector, irq_vector, timer_vector}) {
    bytes.at(vector - entry) = handler >> 8U;
    bytes.at(vector - entry + 1) = 0;
  }
  bytes[protection] = byte_mask;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  return blofeld::McuFlashImage::from_known(bytes);
}

blofeld::McuRunContract mcu_contract() {
  return {.clock = {.oscillator_hz = oscillator_hz,
                    .initial_oscillator_phase_ticks = 0,
                    .switch_timing = blofeld::McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = blofeld::McuRunMemory::unstretched_internal};
}

blofeld::DspRunContract dsp_contract() {
  return {.clock = {.extal_hz = oscillator_hz,
                    .timebase_hz = common_hz,
                    .switch_timing = blofeld::DspClockSwitch::reference_instruction_boundary},
          .timing = blofeld::DspRunTiming::native_internal_estimates};
}

blofeld::McuDspScheduleInputs inputs(const bool qualified) {
  return {.dsp = {.epoch_ticks = epoch,
                  .observation = blofeld::DspStepObservation::reference_completion_boundary},
          .shared_write_order = blofeld::McuDspTieOrder::dsp_before_mcu,
          .maximum_dsp_steps_per_cycle = 16,
          .wait_order = qualified ? blofeld::McuDspWaitOrder::dsp_before_mcu
                                  : blofeld::McuDspWaitOrder::unqualified};
}

blofeld::DspMachine &initialize(blofeld::DspMachine &dsp, const bool fault) {
  blofeld::test::DspProgram program;
  if (fault) {
    program.immediate("r0", stack); // Reserved Y address, unrelated to the MCU stack.
    program.immediate("x0", 7);
    constexpr unsigned padding = 28;
    for (unsigned i = 0; i < padding; ++i) { program.emit("nop"); }
    program.emit("move x0,y:(r0)");
  } else {
    program.emit("nop");
    // FM long absolute JMP: 3 + lab1 = four nominal units.
    program.word(absolute_jump);
    program.word(blofeld::test::boot);
  }
  program.install(dsp.bus());
  dsp.bus().pll().observe_pinit(false);
  dsp.cpu().reset_spi_boot();
  return dsp;
}

struct Ledger {
  std::uint64_t units = 0;
  std::uint64_t steps = 0;
};

struct Fixture {
  explicit Fixture(const bool fail = false, const bool qualified = true)
      : schedule(inputs(qualified), mcu, initialize(dsp, fail)), failing(fail) {
    mcu.bus().set_pin_drive(blofeld::McuPort::e, {.mask = 3, .levels = 3});
    attach();
    for (unsigned i = 0; i < 6; ++i) { static_cast<void>(step()); }
    require(mcu.cpu().waiting() && mcu.run_clock().bus_cycles() == initial_cycles &&
                mcu.run_clock().elapsed_ticks() == initial_ticks && cpu_periods == 16,
            "Five reset periods plus literal 2+2+2+2+1+7 setup cycles reach native WAI");
  }

  void record(const blofeld::DspScheduledStep &event) {
    std::uint64_t cost = event.plan.before.cpu.pc == blofeld::test::boot ? 1U : 4U;
    if (failing) { cost = ledger.steps < 2 ? 2U : 1U; }
    require(event.plan.native_cycle_units == cost &&
                event.plan.duration_ticks == cost * native_unit_ticks &&
                event.plan.start_ticks == ledger.units * native_unit_ticks &&
                event.completion_ticks == epoch + (ledger.units + cost) * native_unit_ticks,
            "Every DSP attempt follows independent immediate/NOP/JMP nominal costs");
    if (edge && event.completion_ticks == *edge) {
      require(!mcu.cpu().pending_interrupt(), "DSP completion precedes the tied MCU RTI edge");
      saw_edge = true;
    }
    if (assert_irq) {
      mcu.bus().set_pin_drive(blofeld::McuPort::e, {.mask = 3, .levels = 1});
      assert_irq = false;
    }
    ledger.units += cost;
    ++ledger.steps;
    trace << "DSP end=" << event.completion_ticks << " units=" << cost << '\n';
  }

  void attach() {
    mcu.set_cycle_observer([this](const auto cycle) {
      ++cpu_periods;
      schedule.begin_cycle(cycle, false, [this](const auto &event) { record(event); });
    });
    mcu.set_wait_period_observer([this](const auto period) {
      ++periods;
      trace << "IDLE prior=" << mcu.idle_bus_cycles() << '\n';
      schedule.begin_wait_period(period, [this](const auto &event) { record(event); });
    });
  }

  blofeld::CpuStep step() {
    const auto result = mcu.cpu().step();
    schedule.end_instruction();
    return result;
  }

  blofeld::McuMachine mcu{image(), BLOFELD_DEFAULT_SLA, mcu_contract()};
  blofeld::DspMachine dsp{dsp_contract()};
  blofeld::McuDspSchedule schedule;
  bool failing;
  Ledger ledger;
  unsigned cpu_periods = 0;
  unsigned periods = 0;
  bool assert_irq = false;
  bool saw_edge = false;
  std::optional<std::uint64_t> edge;
  std::ostringstream trace;
};

void check_frame(blofeld::McuMachine &mcu) {
  constexpr std::array<unsigned, 9> frame{unmasked, 0x34, 0x12, 0x45, 0x67, 0x67, 0x89, 0x80, 0x0f};
  for (unsigned i = 0; i < frame.size(); ++i) {
    require(mcu.bus().read(static_cast<std::uint16_t>(wait_stack + i)) == frame[i],
            "Actual native WAI frame matches every independently expected NXP byte");
  }
}

void check_loop(const Fixture &f) {
  const auto horizon = (f.mcu.run_clock().elapsed_ticks() * 9) - epoch;
  const auto groups = horizon / loop_ticks;
  const auto tail = horizon % loop_ticks >= native_unit_ticks ? 1U : 0U;
  const auto units = (groups * 5) + tail;
  const auto steps = (groups * 2) + tail;
  require(f.dsp.run_clock().native_cycle_units() == units &&
              f.dsp.run_clock().elapsed_ticks() == units * native_unit_ticks &&
              f.schedule.completed_steps() == steps && f.ledger.units == units &&
              f.ledger.steps == steps &&
              f.schedule.end_ticks() == f.mcu.run_clock().elapsed_ticks() * 9,
          "Finite native NOP/JMP completions match an independent five-unit loop ledger");
}

void select_phy(blofeld::McuBus &bus) {
  bus.write(blofeld::hc12::clock_select, 0);
  bus.write(blofeld::hc12::usb_control, timer_gate);
  bus.observe_phy({.edges = 15});
  bus.write(blofeld::hc12::clock_select, enabled);
}

void test_rti_wake_replay(const bool phy, const bool gated) {
  Fixture f;
  check_frame(f.mcu);
  if (phy) { select_phy(f.mcu.bus()); }
  f.mcu.bus().write(blofeld::hc12::tscr1, enabled | (gated ? timer_gate : 0));
  f.mcu.bus().write(blofeld::hc12::rti_control, 0x10); // Independent period: 1024 OSC edges.
  f.mcu.bus().write(blofeld::hc12::crg_interrupts, enabled);
  const unsigned total = phy ? 2560 : 512;
  require(f.mcu.advance_wait(partial_idle) == partial_idle, "First bounded idle slice completes");
  f.schedule.end_wait();
  check_loop(f);
  const auto mcu = f.mcu.checkpoint();
  const auto dsp = f.dsp.checkpoint();
  const auto schedule = f.schedule.checkpoint();
  const auto ledger = f.ledger;
  const auto periods = f.periods;
  const auto cpu_periods = f.cpu_periods;
  std::string expected;
  for (unsigned replay = 0; replay < 2; ++replay) {
    f.mcu.restore(mcu);
    f.dsp.restore(dsp);
    f.schedule.restore(schedule);
    f.ledger = ledger;
    f.periods = periods;
    f.cpu_periods = cpu_periods;
    f.saw_edge = false;
    f.edge = rti_endpoint;
    f.trace.str({});
    f.attach();
    require(f.mcu.advance_wait(total) == total - partial_idle,
            "Only actual remaining OSC-derived periods run before native RTI becomes pending");
    f.schedule.end_wait();
    check_loop(f);
    require(f.saw_edge && f.periods == total && f.cpu_periods == 16 &&
                f.mcu.idle_bus_cycles() == total &&
                f.mcu.run_clock().bus_cycles() == initial_cycles + total &&
                f.mcu.run_clock().elapsed_ticks() == final_idle_ticks,
            "Both source routes and TIM gates reach the same exact common-time RTI endpoint");
    const auto entered = f.step();
    require(entered.interrupt_vector == rti_vector && entered.stack_reused &&
                !entered.instruction_executed && f.mcu.cpu().state().sp == wait_stack,
            "Native six-cycle WAI wake reuses the actual frame without another push");
    check_frame(f.mcu);
    for (unsigned i = 0; i < 3; ++i) { static_cast<void>(f.step()); }
    const auto state = f.mcu.cpu().state();
    const auto timer = f.mcu.bus().read_memory({.address = counter, .size = 2});
    require(state.pc == resume && state.sp == stack && state.d == foreground_d &&
                state.x == foreground_x && state.y == foreground_y && state.ccr == unmasked &&
                f.cpu_periods == final_cpu_periods &&
                timer == (gated ? wake_handler_periods : total + wake_handler_periods) &&
                !f.mcu.cpu().pending_interrupt(),
            "Actual handler acknowledges RTI; independent frame/registers and TIM gate restore");
    check_loop(f);
    if (replay == 0) { expected = f.trace.str(); }
    require(f.trace.str() == expected,
            "Both native owners and pending DSP plan replay every event");
  }
}

void test_cancelled_wait_offers() {
  for (unsigned kind = 0; kind < 3; ++kind) {
    Fixture f;
    unsigned pre = 0;
    const auto irq = [&] {
      f.mcu.bus().set_pin_drive(blofeld::McuPort::e, {.mask = 3, .levels = 1});
    };
    if (kind == 2) {
      f.mcu.bus().set_pin_drive(blofeld::McuPort::t, {.mask = 1});
      f.mcu.set_timer_inputs(1);
      f.mcu.bus().write(blofeld::hc12::tscr1, enabled);
      f.mcu.bus().write(capture_edges, 1);
      f.mcu.bus().write(capture_interrupts, 1);
    }
    require(f.mcu.advance_wait(1) == 1, "One real period separates timer input observations");
    f.schedule.end_wait();
    const auto dsp = f.dsp.cpu().debug_state();
    const auto dsp_ticks = f.dsp.run_clock().elapsed_ticks();
    if (kind == 0) { irq(); }
    f.mcu.set_wait_observer([&](auto /*cycle*/) {
      ++pre;
      if (kind == 1) { irq(); }
      if (kind == 2) { f.mcu.bus().set_pin_drive(blofeld::McuPort::t, {.mask = 1, .levels = 1}); }
    });
    require(f.mcu.advance_wait(10) == 0 && f.periods == 1 && pre == (kind == 0 ? 0U : 1U),
            "Existing, pre-observed and sampled IRQs cancel before a committed idle hook");
    f.schedule.end_wait();
    require(f.mcu.run_clock().bus_cycles() == initial_cycles + 1 && f.mcu.idle_bus_cycles() == 1 &&
                f.dsp.cpu().debug_state() == dsp && f.dsp.run_clock().elapsed_ticks() == dsp_ticks,
            "Cancelled MCU offers execute no DSP instruction or clock on either owner");
    check_loop(f);
  }
}

void test_interrupt_from_committed_period() {
  Fixture f;
  require(f.mcu.advance_wait(1) == 1, "The preceding bounded period leaves two DSP steps due next");
  f.schedule.end_wait();
  f.assert_irq = true;
  require(f.mcu.advance_wait(10) == 1,
          "An IRQ supplied within a committed period cannot cancel it");
  f.schedule.end_wait();
  require(f.periods == 2 && f.mcu.idle_bus_cycles() == 2 &&
              f.mcu.run_clock().bus_cycles() == initial_cycles + 2 && f.mcu.cpu().waiting(),
          "The one actual MCU period completes before native wake is selected");
  check_loop(f);
  require(f.step().interrupt_vector == irq_vector, "A real pending external IRQ wakes next");
}

void test_idle_fault_and_restore() {
  Fixture f(true);
  const auto mcu = f.mcu.checkpoint();
  const auto dsp = f.dsp.checkpoint();
  const auto schedule = f.schedule.checkpoint();
  const auto ledger = f.ledger;
  std::string expected;
  for (unsigned replay = 0; replay < 2; ++replay) {
    f.mcu.restore(mcu);
    f.dsp.restore(dsp);
    f.schedule.restore(schedule);
    f.ledger = ledger;
    f.attach();
    const auto error = rejects([&] { f.mcu.advance_wait(1); });
    require(error.contains("Scheduled idle peer execution mcu-pc=0x800f") &&
                error.contains("attempted-completion=1935") &&
                error.contains("Unmapped DSP memory write at Y:0x002000") &&
                f.mcu.run_clock().bus_cycles() == initial_cycles && f.mcu.idle_bus_cycles() == 0 &&
                f.dsp.run_clock().native_cycle_units() == 32 &&
                f.dsp.run_clock().elapsed_ticks() == failed_dsp_ticks && f.schedule.faulted(),
            "Actual idle operand failure charges no unconsumed MCU period or failed DSP word");
    static_cast<void>(rejects([&] { static_cast<void>(f.mcu.checkpoint()); }));
    static_cast<void>(rejects([&] { static_cast<void>(f.schedule.checkpoint()); }));
    check_frame(f.mcu);
    if (replay == 0) { expected = error; }
    require(error == expected, "Coordinated restore repeats the same actual idle operand fault");
  }
  Fixture unqualified(false, false);
  static_cast<void>(rejects([&] { unqualified.mcu.advance_wait(1); }));
  require(unqualified.mcu.run_clock().bus_cycles() == initial_cycles &&
              unqualified.dsp.run_clock().native_cycle_units() == unqualified_dsp_units,
          "Unqualified idle observation order cannot advance native MCU or DSP time");
}
}

int main() {
  try {
    for (const bool phy : {false, true}) {
      for (const bool gated : {false, true}) { test_rti_wake_replay(phy, gated); }
    }
    test_cancelled_wait_offers();
    test_interrupt_from_committed_period();
    test_idle_fault_and_restore();
    std::cout
        << "Paired native WAI periods, cancelled offers, RTI wake and joint restore passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
