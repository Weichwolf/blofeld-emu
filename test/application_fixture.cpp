// SPDX-License-Identifier: GPL-3.0-only
#include "application_fixture.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "lcd_fixture.hpp"
#include "parallel_update_fixture.hpp"

#include <array>
#include <cstdint>
#include <ostream>
#include <string_view>

namespace blofeld::test {
namespace {
constexpr unsigned e_fixture_mask = 0xeb;
constexpr unsigned enable_pad = 0x10;
constexpr unsigned other_serial_inputs = 0xe8;
constexpr unsigned external_u_mask = 0x3f;
constexpr unsigned ready_poll = 0xb98d;
constexpr unsigned display_initialization = 0x8b94;
constexpr unsigned release_inputs = 0x820e;
constexpr unsigned released_b_inputs = 0x1e;
constexpr unsigned startup_poll = 0x869d;
constexpr unsigned countdown_poll = 0x82f6;
constexpr unsigned pm3_read = 0x8301;
constexpr unsigned application_wait = 0x8314;
constexpr unsigned application_resume = 0x8315;
constexpr unsigned main_stack = 0x25f3;
constexpr unsigned wai_stack = 0x25ea;
constexpr unsigned pm3_mask = 8;
constexpr unsigned saved_pm3 = 0x182f;
constexpr unsigned pm3_pull = 0x24c;
constexpr unsigned rti_vector = 0xfff0;
constexpr unsigned step_limit = 12000000;
constexpr unsigned checkpoint_period = 100000;
constexpr unsigned byte_mask = 255;
constexpr unsigned application_instructions = 11979871;
constexpr unsigned fault_instructions = 10708295;
constexpr unsigned periods_to_wait = 6384;

void require(const bool condition, const char *message) {
  if (!condition) { throw Fault(message); }
}

bool same_state(const CpuState &left, const CpuState &right) {
  return left.pc == right.pc && left.d == right.d && left.x == right.x && left.y == right.y &&
         left.sp == right.sp && left.ccr == right.ccr;
}

void check_nvm(const McuBus &bus, const McuFlashImage &initial) {
  const auto current = bus.power_off_flash_image();
  require(current.bytes == initial.bytes && current.known == initial.known &&
              current.fresh == initial.fresh,
          "Native application preserves all MCU NVM bytes, knownness and erase history");
}

void observe_startup(McuBus &bus, const unsigned pc) {
  if (pc == ready_poll) { bus.set_pin_drive(McuPort::u, {.mask = external_u_mask, .levels = 1}); }
  if (pc == display_initialization) { bus.set_pin_drive(McuPort::b, {.mask = 8}); }
  if (pc == release_inputs) { bus.set_pin_drive(McuPort::b, {.mask = released_b_inputs}); }
}

void qualify_pm3(
    McuBus &bus, Hc12 &cpu, const ApplicationRun &run, const bool high, std::ostream &trace) {
  require(run.instructions == fault_instructions && run.interrupts == periods_to_wait,
          "Original countdown finishes before the first PM3 input observation");
  const auto before = cpu.state();
  const auto revision = bus.revision();
  const auto initial = bus.power_off_flash_image();
  bool rejected = false;
  try {
    cpu.step();
  } catch (const Fault &fault) {
    const std::string_view message(fault.what());
    require(message.contains("MCU read at 0x0248") && message.contains("pc=0x8301") &&
                message.contains("ccr=") && message.contains("mask=0x08"),
            "Unobserved PM3 must fail at its original read with address, PC and CCR");
    trace << "missing-pm3 " << message << '\n';
    rejected = true;
  }
  require(rejected && same_state(cpu.state(), before) && bus.revision() == revision,
          "An unknown plain input read has no visible register or bus mutation before retry");
  check_nvm(bus, initial);
  require((bus.read(hc12::ddrm) & pm3_mask) == 0 && (bus.read(pm3_pull) & pm3_mask) == 0,
          "Native PM3 is an input with its pull device disabled");
  bus.set_pin_drive(McuPort::m,
                    {.mask = pm3_mask, .levels = static_cast<std::uint8_t>(high ? pm3_mask : 0)});
}

void check_wait(McuBus &bus, const Hc12 &cpu, const CpuState &before) {
  const auto after = cpu.state();
  require(before.pc == application_wait && before.sp == main_stack &&
              (before.ccr & hc12::interrupt_mask) == 0 && cpu.waiting() &&
              after.pc == application_resume && after.sp == wai_stack && after.d == before.d &&
              after.x == before.x && after.y == before.y && after.ccr == before.ccr &&
              !cpu.pending_interrupt(),
          "Original main WAI preserves registers, keeps IRQs enabled and saves its native frame");
  // S12CPUV2 WAI definition: CCR,B,A,X,Y,next-PC at increasing stack addresses.
  const std::array<unsigned, 9> expected{before.ccr,
                                         before.d & byte_mask,
                                         static_cast<unsigned>(before.d) >> 8U,
                                         static_cast<unsigned>(before.x) >> 8U,
                                         before.x & byte_mask,
                                         static_cast<unsigned>(before.y) >> 8U,
                                         before.y & byte_mask,
                                         application_resume >> 8U,
                                         application_resume & byte_mask};
  for (unsigned index = 0; index < expected.size(); ++index) {
    require(bus.read(static_cast<std::uint16_t>(wai_stack + index)) == expected[index],
            "Original main's saved WAI frame matches the independent NXP byte layout");
  }
}
}

ApplicationRun start_application(
    McuBus &bus, Hc12 &cpu, LcdFixture &display, std::ostream &trace, const bool pm3_high) {
  const auto initial = bus.power_off_flash_image();
  boot(bus, cpu, trace);
  bus.set_pin_drive(McuPort::q, {.floating = byte_mask});
  bus.observe_usb_decoder({});
  bus.set_pin_drive(McuPort::e, {.mask = e_fixture_mask, .levels = 3, .floating = enable_pad});
  bus.set_pin_drive(McuPort::b, {.mask = other_serial_inputs});
  bus.set_observer([&display](const BusEvent &event) { display.observe(event); });
  ApplicationRun run;
  bool deliver_rti = true;
  bool observed_pm3 = false;
  CpuState before_wait;
  for (unsigned steps = 0; !cpu.waiting() || cpu.pending_interrupt(); ++steps) {
    require(steps < step_limit, "Original application must reach a bounded wait or visible fault");
    const auto before = cpu.state();
    observe_startup(bus, before.pc);
    if (before.pc == pm3_read) {
      require(!observed_pm3, "Native startup observes PM3 once without host redirects");
      qualify_pm3(bus, cpu, run, pm3_high, trace);
      observed_pm3 = true;
    }
    const bool clock_slot = before.pc == startup_poll || before.pc == countdown_poll;
    if (clock_slot && deliver_rti && !cpu.pending_interrupt()) {
      const auto cycles = bus.oscillator_cycles_until_rti();
      require(cycles != 0, "Original RTI configuration supplies a known oscillator period");
      bus.advance_oscillator(cycles);
      ++run.oscillator_periods;
      deliver_rti = false;
    }
    if (before.pc == application_wait) { before_wait = before; }
    const auto result = cpu.step();
    if (result.instruction_executed) {
      ++run.instructions;
      if (clock_slot) { deliver_rti = true; }
    }
    if (result.interrupt_vector) {
      require(*result.interrupt_vector == rti_vector && !result.instruction_executed &&
                  !result.stack_reused,
              "Startup enters real RTI handlers through the native interrupt controller");
      ++run.interrupts;
    }
    if (steps % checkpoint_period == 0) {
      trace << "startup instructions=" << run.instructions << " pc=" << hex(cpu.state().pc)
            << " interrupts=" << run.interrupts << '\n';
      trace.flush();
    }
  }
  check_wait(bus, cpu, before_wait);
  const auto counts = display.counts();
  constexpr unsigned commands = 302;
  constexpr unsigned writes = 2402;
  constexpr unsigned reads = 2244;
  require(observed_pm3 && run.instructions == application_instructions &&
              run.interrupts == periods_to_wait && run.oscillator_periods == periods_to_wait &&
              bus.read(saved_pm3) == (pm3_high ? pm3_mask : 0) && counts.commands == commands &&
              counts.writes == writes && counts.reads == reads,
          "Original application initializes its own PM3 state and display before main WAI");
  check_nvm(bus, initial);
  trace << "native-main-wai pm3=" << (pm3_high ? 1 : 0) << " instructions=" << run.instructions
        << " interrupts=" << run.interrupts << " sp=" << hex(cpu.state().sp)
        << " ccr=" << hex(cpu.state().ccr, 2) << " reads=" << counts.reads << '\n';
  return run;
}
}
