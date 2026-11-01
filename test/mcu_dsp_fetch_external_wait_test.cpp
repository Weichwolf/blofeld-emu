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
constexpr unsigned oscillator = 12000000, common = 540000000, entry = 0x8000, handler = 0x9000;
constexpr unsigned flags = 0xc0, stack = 0x2000, wait_stack = 0x1ff7, vector = 0xfff2,
                   byte_mask = 0xff;
constexpr std::uint64_t initial_ticks = 450, recovery_end = 540, period = 45;
constexpr std::uint64_t waiting = 1350, irq = 1395, idle_end = 1440, deassert = 2000;
constexpr std::uint64_t mcu_period = 90;
constexpr unsigned return_periods = 29;

constexpr unsigned boot = 0xff0000;

void require(const bool condition, const char *message) {
  if (!condition) { throw Fault(message); }
}

DspDmaInputs cycle_inputs() {
  return {.known_requests = 15,
          .timing = DspDmaTiming::reference_sampled_cycles,
          .grant = DspDmaGrant::available};
}

dsp56300::Registers nop_registers(const std::uint64_t cycles) {
  constexpr unsigned reset_status = 0xc00300;
  constexpr unsigned reset_omr = 0x305;
  dsp56300::Registers expected{};
  expected.pc = boot;
  expected.sr = reset_status;
  expected.omr = reset_omr;
  expected.m.fill(dsp56300::word_mask);
  if (cycles >= 3) { expected.pc += static_cast<unsigned>((cycles - 1) / 2); }
  return expected;
}

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

McuDspRuntimeInputs inputs() {
  return {.timing = OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts,
          .maximum_cycles_per_period = 16,
          .writes = McuWriteOrder::before_mcu,
          .wait_dsp_first = true,
          .reset_dsp_first = true,
          .initial_reset = OwnDspResetInputs{
              .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}};
}

void prepare(OwnDspMachine &dsp) {
  constexpr unsigned nops = 128;
  const std::array<std::uint32_t, nops> program{};
  dsp.bus().install_rom({.space = DspSpace::p, .start = boot, .words = program});
}

struct Fixture {
  Fixture(const DspPeerOrder policy, const std::uint64_t at) : irq_tick(at), order(policy) {
    prepare(dsp);
    runtime = std::make_unique<McuDspRuntime>(inputs(), mcu, dsp, events());
    require(runtime->end_ticks() == initial_ticks &&
                dsp.clock().hardware_reset_ticks() == initial_ticks,
            "Five actual MCU reset-vector periods retain DSP hold from zero");
    runtime->release_dsp_reset();
    mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
    attach();
  }

  ~Fixture() {
    mcu.set_cycle_observer({});
    mcu.set_wait_period_observer({});
  }

  Fixture(const Fixture &) = delete;
  Fixture &operator=(const Fixture &) = delete;

  DspRuntimeEvents events() {
    return {.identity = identity,
            .next_ticks = [this]() -> std::optional<std::uint64_t> {
              if (cursor == 0) { return irq_tick; }
              if (cursor == 1) { return deassert; }
              return std::nullopt;
            },
            .observe =
                [this](const std::uint64_t at) {
                  require(at == (cursor == 0 ? irq_tick : deassert),
                          "Actual IRQ input time and cursor match");
                  mcu.bus().set_pin_drive(
                      McuPort::e,
                      {.mask = 3, .levels = static_cast<std::uint8_t>(cursor == 0 ? 1U : 3U)});
                  trace.push_back({.actor = 'E', .ticks = at});
                  ++cursor;
                },
            .order = order,
            .maximum_events_per_period = 2};
  }

  void restore(const McuDspRuntimeCheckpoint &saved) {
    runtime.reset();
    cursor = saved_cursor;
    trace.clear();
    runtime = std::make_unique<McuDspRuntime>(saved, mcu, dsp, events());
    attach();
  }

  void attach() {
    const auto observed = [this](const DspRuntimeCycle &cycle) {
      const auto count = cycle.cycle + 1;
      const auto retired = count < 2 ? 0 : (count - 2) / 2;
      require(cycle.completion_ticks - cycle.start_ticks == period &&
                  cycle.completion_ticks == recovery_end + (count * period) &&
                  runtime->completed_cycles() == count &&
                  dsp.cpu().registers() == nop_registers(count) &&
                  dsp.cpu().progress() ==
                      dsp56300::Progress{.instructions = retired, .estimated_cycles = retired},
              "Native NOP periods,PCU,flags and full registers retain independent expectations");
      trace.push_back({.actor = 'D', .ticks = cycle.completion_ticks});
    };
    mcu.set_cycle_observer([this, observed](const auto cycle) {
      runtime->begin_cycle(cycle, false, cycle_inputs, observed);
    });
    mcu.set_wait_period_observer([this, observed](const auto slice) {
      runtime->begin_wait_period(slice, cycle_inputs, observed);
    });
  }

  CpuStep step() {
    const auto result = mcu.cpu().step();
    runtime->end_instruction();
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
  OwnDspMachine dsp{{.extal_hz = oscillator,
                     .timebase_hz = common,
                     .switch_timing = DspClockSwitch::reference_cycle_boundary}};
  std::shared_ptr<const bool> identity = std::make_shared<const bool>(true);
  unsigned cursor = 0;
  unsigned saved_cursor = 0;
  const std::uint64_t irq_tick;
  const DspPeerOrder order;
  std::vector<Event> trace;
  std::unique_ptr<McuDspRuntime> runtime;
};

void run(const DspPeerOrder order, const std::uint64_t irq_tick) {
  Fixture f(order, irq_tick);
  for (unsigned i = 0; i < 3; ++i) {
    require(f.step().instruction_executed, "Native setup reaches actual WAI");
  }
  require(f.mcu.cpu().waiting() && f.mcu.cpu().state().pc == entry + 6 &&
              f.mcu.cpu().state().sp == wait_stack && f.runtime->end_ticks() == waiting &&
              f.mcu.run_clock().bus_cycles() == 15 && f.cursor == (irq_tick == waiting ? 1U : 0U),
          "Independent2+1+7 instruction periods and genuine reset establish the WAI cut");
  f.frame();
  const auto saved = f.runtime->checkpoint();
  f.saved_cursor = f.cursor;
  const auto prefix = f.trace;
  const auto idle_periods = (irq_tick - waiting + mcu_period - 1) / mcu_period;
  const auto wait_end = waiting + (idle_periods * mcu_period);
  const auto wake_end = wait_end + (6 * mcu_period);
  const auto final_end = wake_end + (8 * mcu_period);
  const auto native_periods = (final_end - recovery_end) / period;
  const auto retired = (native_periods - 2) / 2;
  std::vector<Event> suffix;
  for (unsigned replay = 0; replay < 2; ++replay) {
    f.restore(saved);
    const auto first = f.trace.size();
    require(f.mcu.advance_wait(10) == idle_periods,
            "Timed IRQ stops after its already committed idle period");
    f.runtime->end_wait();
    f.frame();
    require(f.cursor == 1 && f.mcu.cpu().waiting() && f.mcu.cpu().pending_interrupt() &&
                f.mcu.idle_bus_cycles() == idle_periods &&
                f.mcu.run_clock().bus_cycles() == 15 + idle_periods &&
                f.runtime->end_ticks() == wait_end &&
                f.runtime->completed_cycles() == (wait_end - recovery_end) / period,
            "Asynchronous input preserves real idle completion and continuing native DSP clocks");
    const auto wake = f.step();
    require(!wake.instruction_executed && wake.stack_reused && wake.interrupt_vector == vector &&
                f.mcu.cpu().state().sp == wait_stack && f.mcu.cpu().state().pc == handler &&
                f.runtime->end_ticks() == wake_end,
            "Native six-period IRQ wake reuses the genuine WAI frame");
    f.frame();
    require(f.step().instruction_executed, "Actual eight-period RTI executes");
    const auto cpu = f.mcu.cpu().state();
    require(cpu.pc == entry + 6 && cpu.sp == stack && cpu.d == 0 && cpu.x == 0 && cpu.y == 0 &&
                cpu.ccr == flags && !f.mcu.cpu().waiting() && !f.mcu.cpu().pending_interrupt() &&
                f.mcu.run_clock().bus_cycles() == return_periods + idle_periods &&
                f.runtime->end_ticks() == final_end &&
                f.runtime->completed_cycles() == native_periods &&
                f.dsp.clock().timer_clk2_edges() == native_periods / 2 && f.cursor == 2 &&
                f.dsp.cpu().registers() == nop_registers(native_periods) &&
                f.dsp.cpu().progress() ==
                    dsp56300::Progress{.instructions = retired, .estimated_cycles = retired} &&
                f.dsp.clock().hardware_reset_ticks() == recovery_end &&
                f.dsp.clock().elapsed_ticks() == final_end,
            "Literal CPU/stack/flags/time endpoints and actual native NOP publications agree");
    std::vector<Event> tied;
    const auto &at_irq = irq_tick == waiting ? prefix : f.trace;
    for (const auto event : at_irq) {
      if (event.ticks == irq_tick) { tied.push_back(event); }
    }
    std::vector<Event> wanted{{.actor = 'E', .ticks = irq_tick}};
    if ((irq_tick - recovery_end) % period == 0) {
      const Event native{.actor = 'D', .ticks = irq_tick};
      if (order == DspPeerOrder::peer_first) {
        wanted.push_back(native);
      } else {
        wanted.insert(wanted.begin(), native);
      }
    }
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
    for (const auto at : {waiting, waiting + 1, irq, idle_end - 1, idle_end, idle_end + 1}) {
      run(DspPeerOrder::peer_first, at);
      run(DspPeerOrder::dsp_first, at);
    }
    std::cout
        << "Native MCU/dsp56300 WAI events:12 profiles,24 restored IRQ/RTI suffixes passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
