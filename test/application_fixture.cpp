// SPDX-License-Identifier: GPL-3.0-only
#include "application_fixture.hpp"

#include "common/bus.hpp"
#include "lcd_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "parallel_update_fixture.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>
#include <memory>
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
constexpr unsigned checkpoint_period = 100000;
constexpr unsigned byte_mask = 255;
constexpr unsigned application_instructions = 12046302;
constexpr unsigned fault_instructions = 10705413;
constexpr unsigned periods_to_wait = 6384;
// Native retired instructions plus real interrupt entries, with one boundary slot.
constexpr unsigned step_limit = application_instructions + periods_to_wait + 1;

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

void qualify_pm3(McuMachine &machine,
                 std::unique_ptr<LcdFixture> &display,
                 const bool high,
                 std::ostream &trace) {
  const auto checkpoint = machine.checkpoint();
  const LcdFixture saved_display(machine.bus(), *display);
  auto &bus = machine.bus();
  auto &cpu = machine.cpu();
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
          "An unknown plain input read has no visible register or bus mutation");
  bool stopped = false;
  try {
    cpu.step();
  } catch (const Fault &fault) {
    stopped = std::string_view(fault.what()).contains("requires coordinated recovery");
  }
  require(stopped && same_state(cpu.state(), before) && bus.revision() == revision,
          "A failed PM3 read cannot implicitly resume even with unchanged visible registers");
  check_nvm(bus, initial);
  require((bus.read(hc12::ddrm) & pm3_mask) == 0 && (bus.read(pm3_pull) & pm3_mask) == 0,
          "Native PM3 is an input with its pull device disabled");
  machine.restore(checkpoint);
  display = std::make_unique<LcdFixture>(machine.bus(), saved_display);
  machine.bus().set_observer([&display](const BusEvent &event) { display->observe(event); });
  require(same_state(machine.cpu().state(), before),
          "Joint recovery restores the complete pre-read CPU boundary");
  machine.bus().set_pin_drive(
      McuPort::m, {.mask = pm3_mask, .levels = static_cast<std::uint8_t>(high ? pm3_mask : 0)});
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

void observe_boundary(const ApplicationBoundary &observer,
                      const McuMachine &machine,
                      const LcdFixture &display) {
  if (observer) { observer(machine, display); }
}
}

ApplicationRun start_application_reference(McuMachine &machine,
                                           std::unique_ptr<LcdFixture> &display,
                                           std::ostream &trace,
                                           const bool pm3_high,
                                           const unsigned maximum_steps,
                                           const ApplicationBoundary &observer) {
  const auto initial = machine.bus().power_off_flash_image();
  boot(machine.bus(), machine.cpu(), trace);
  machine.bus().set_pin_drive(McuPort::q, {.floating = byte_mask});
  machine.bus().observe_usb_decoder({});
  machine.bus().set_pin_drive(McuPort::e,
                              {.mask = e_fixture_mask, .levels = 3, .floating = enable_pad});
  machine.bus().set_pin_drive(McuPort::b, {.mask = other_serial_inputs});
  machine.bus().set_observer([&display](const BusEvent &event) { display->observe(event); });
  ApplicationRun run;
  bool deliver_rti = true;
  bool observed_pm3 = false;
  CpuState before_wait;
  for (unsigned steps = 0; !machine.cpu().waiting() || machine.cpu().pending_interrupt(); ++steps) {
    require(steps < maximum_steps,
            "Original application must reach a bounded wait or visible fault");
    const auto before = machine.cpu().state();
    observe_startup(machine.bus(), before.pc);
    if (before.pc == pm3_read) {
      require(!observed_pm3, "Native startup observes PM3 once without host redirects");
      run.before_input_instructions = run.instructions;
      run.before_input_interrupts = run.interrupts;
      qualify_pm3(machine, display, pm3_high, trace);
      observed_pm3 = true;
    }
    auto &bus = machine.bus();
    auto &cpu = machine.cpu();
    const bool clock_slot = before.pc == startup_poll || before.pc == countdown_poll;
    if (clock_slot && deliver_rti && !cpu.pending_interrupt()) {
      const auto cycles = bus.oscillator_cycles_until_rti();
      require(cycles != 0, "Original RTI configuration supplies a known oscillator period");
      bus.advance_oscillator(cycles);
      ++run.oscillator_periods;
      deliver_rti = false;
    }
    if (before.pc == application_wait) { before_wait = before; }
    observe_boundary(observer, machine, *display);
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
  auto &bus = machine.bus();
  const auto &cpu = machine.cpu();
  check_wait(bus, cpu, before_wait);
  const auto counts = display->counts();
  require(observed_pm3 && bus.read(saved_pm3) == (pm3_high ? pm3_mask : 0),
          "Original application initializes its own PM3 state before main WAI");
  check_nvm(bus, initial);
  trace << "native-main-wai pm3=" << (pm3_high ? 1 : 0) << " instructions=" << run.instructions
        << " interrupts=" << run.interrupts << " sp=" << hex(cpu.state().sp)
        << " ccr=" << hex(cpu.state().ccr, 2) << " reads=" << counts.reads << '\n';
  return run;
}

ApplicationRun start_application(McuMachine &machine,
                                 std::unique_ptr<LcdFixture> &display,
                                 std::ostream &trace,
                                 const bool pm3_high) {
  const auto run = start_application_reference(machine, display, trace, pm3_high, step_limit);
  const auto counts = display->counts();
  constexpr unsigned commands = 344;
  constexpr unsigned writes = 4194;
  constexpr unsigned reads = 2244;
  require(run.before_input_instructions == fault_instructions &&
              run.before_input_interrupts == periods_to_wait &&
              run.instructions == application_instructions && run.interrupts == periods_to_wait &&
              run.oscillator_periods == periods_to_wait && counts.commands == commands &&
              counts.writes == writes && counts.reads == reads,
          "Original erased-unit countdown, instruction and LCD comparison contract is unchanged");
  return run;
}
}
