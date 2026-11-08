// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "cpu12/memory.hpp"
#include "isa.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_memory.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "mcu_checkpoint_fixture.hpp"
#include "runtime/gpio_pads.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using blofeld::test::require;
namespace isa = cpu12::isa;
using cpu12::Byte;
using isa::Opcode;
constexpr unsigned entry = blofeld::test::entry;
constexpr unsigned stack = blofeld::test::stack;
constexpr unsigned horizon = 1000000;
constexpr unsigned oscillator_hz = 12000000;
constexpr std::uint64_t period_ticks = 10;
constexpr Byte unmasked = 0xc0;
constexpr Byte timer_on = 0x80;
constexpr Byte loop_back = 0xfd;
constexpr Byte sample_high = 0x12, sample_low = 0x34;
constexpr unsigned sample_value = 0x1234;
constexpr unsigned read_byte_result = 0x3400;

McuRunContract contract() {
  return {.clock = {.oscillator_hz = oscillator_hz,
                    .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = McuRunMemory::internal_flash_reads};
}

void prepare(McuMachine &machine) {
  blofeld::test::drive_interrupts(machine.bus());
  machine.cpu().set_state({.pc = entry, .sp = stack, .ccr = unmasked});
}

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected a visible native MCU failure");
}

void resident_loop() {
  constexpr auto program = isa::program(
      isa::dex, isa::opcode<Opcode::bne_relative>(), std::array<Byte, 1>{loop_back}, isa::wai);
  McuMachine machine(blofeld::test::checkpoint_image(program), BLOFELD_DEFAULT_SLA, contract());
  prepare(machine);
  constexpr unsigned trips = 10000;
  machine.cpu().set_state({.pc = entry, .x = trips, .sp = stack, .ccr = unmasked});
  std::vector<McuSpan> spans;
  machine.set_span_observer([&](const McuSpan span) { spans.push_back(span); });
  const auto result = machine.run_until(horizon);
  constexpr unsigned cycles = 5 + (4 * trips) + 5; // Reset + DEX/BNE + WAI.
  require(result.retired == 2 * trips + 1 && machine.cpu().waiting() &&
              machine.cpu().state().x == 0 && machine.cpu().state().sp == stack - 9 &&
              machine.run_clock().bus_cycles() == cycles &&
              machine.run_clock().elapsed_ticks() == period_ticks * cycles && spans.size() == 1 &&
              spans.front().cycles == cycles - 5 && !spans.front().waiting,
          "A complete native loop and WAI use one completed span, without opcode/bus callbacks");
  machine.validate_run_boundary();
}

void io_boundary() {
  constexpr auto program = isa::program(isa::opcode<Opcode::ldaa_immediate>(),
                                        std::array<Byte, 1>{timer_on},
                                        isa::opcode<Opcode::staa_extended>(),
                                        std::array<Byte, 2>{0, hc12::tscr1},
                                        isa::opcode<Opcode::ldd_immediate>(),
                                        std::array<Byte, 2>{sample_high, sample_low},
                                        isa::opcode<Opcode::std_extended>(),
                                        std::array<Byte, 2>{sample_high, 0},
                                        isa::wai);
  McuMachine machine(blofeld::test::checkpoint_image(program), BLOFELD_DEFAULT_SLA, contract());
  prepare(machine);
  unsigned writes = 0;
  machine.bus().set_observer([&](const BusEvent &event) {
    if (!event.write) { return; }
    ++writes;
    require(event.address == hc12::tscr1 && event.value == timer_on &&
                machine.run_clock().bus_cycles() == 6,
            "The actual register write observes completed preceding native time");
  });
  const auto first = machine.run_until(horizon);
  require(first.retired == 2 && machine.cpu().state().pc == entry + 5 && writes == 1 &&
              machine.run_clock().bus_cycles() == 9,
          "I/O yields after its whole instruction so changed clocks and mappings are admitted");
  const auto second = machine.run_until(horizon);
  constexpr unsigned completed_cycles = 21;
  constexpr unsigned timer_counter = 0x44;
  require(second.retired == 3 && machine.cpu().waiting() && writes == 1 &&
              machine.bus().read_memory({.address = blofeld::test::ram, .size = 2}) ==
                  sample_value &&
              machine.bus().read_memory({.address = timer_counter, .size = 2}) == 15 &&
              machine.run_clock().bus_cycles() == completed_cycles,
          "Native stores publish borrowed RAM and timer time without emitting RAM I/O events");
  machine.validate_run_boundary();
}

void gpio_reads_continue() {
  constexpr Byte selected_bit = 4;
  constexpr unsigned doubled_sample = 0x3434;
  constexpr auto program = isa::program(
      isa::opcode<Opcode::ldaa_extended>(),
      std::array<Byte, 2>{hc12::ptq >> 8U, hc12::ptq & blofeld::test::byte_mask},
      isa::opcode<Opcode::ldd_extended>(),
      std::array<Byte, 2>{hc12::ptq >> 8U, hc12::ptq & blofeld::test::byte_mask},
      isa::opcode<Opcode::brclr_extended>(),
      std::array<Byte, 4>{hc12::ptq >> 8U, hc12::ptq & blofeld::test::byte_mask, selected_bit, 1},
      isa::inx,
      isa::wai);
  McuMachine machine(blofeld::test::checkpoint_image(program), BLOFELD_DEFAULT_SLA, contract());
  prepare(machine);
  std::vector<std::uint32_t> masks;
  std::vector<std::uint64_t> times;
  machine.bus().attach_gpio_pad_source(McuPort::q, [&](const std::uint32_t mask) {
    masks.push_back(mask);
    times.push_back(machine.run_clock().bus_cycles());
    return GpioPadSample{.known = mask, .levels = sample_low & mask};
  });
  const auto result = machine.run_until(horizon);
  constexpr unsigned completed_cycles = 24; // Reset + LDAA/LDD/BRCLR/INX/WAI.
  require(result.retired == 5 && machine.cpu().waiting() &&
              machine.cpu().state().pc == entry + program.size() &&
              machine.cpu().state().d == doubled_sample && machine.cpu().state().x == 1 &&
              machine.run_clock().bus_cycles() == completed_cycles &&
              masks == std::vector<std::uint32_t>{blofeld::test::byte_mask,
                                                  blofeld::test::byte_mask,
                                                  blofeld::test::byte_mask,
                                                  selected_bit} &&
              times == std::vector<std::uint64_t>{5, 8, 8, 11},
          "Byte, word and masked GPIO reads keep their exact input times and continue in one "
          "native run");
  machine.validate_run_boundary();
}

enum class ReadIrq : std::uint8_t { before, during, masked };

void gpio_read_irq(const ReadIrq arrival) {
  constexpr auto program =
      isa::program(isa::nop,
                   isa::opcode<Opcode::ldaa_extended>(),
                   std::array<Byte, 2>{hc12::ptq >> 8U, hc12::ptq & blofeld::test::byte_mask},
                   isa::inx,
                   isa::wai);
  McuMachine machine(blofeld::test::checkpoint_image(program), BLOFELD_DEFAULT_SLA, contract());
  prepare(machine);
  constexpr Byte all_masked = 0xd0;
  if (arrival == ReadIrq::masked) {
    machine.cpu().set_state({.pc = entry, .sp = stack, .ccr = all_masked});
  }
  constexpr Byte edge_control = 0xc0;
  machine.bus().write(hc12::irq_control, edge_control);
  bool raised = false;
  const auto raise = [&] {
    require(!raised, "The external source supplies only one actual IRQ edge");
    raised = true;
    machine.bus().update_pin_drive(McuPort::e, 2, {.mask = 2, .levels = 0});
    machine.bus().update_pin_drive(McuPort::e, 2, {.mask = 2, .levels = 2});
  };
  machine.set_span_observer([&](McuSpan /*span*/) {
    if (arrival == ReadIrq::before && !raised) { raise(); }
  });
  machine.bus().attach_gpio_pad_source(McuPort::q, [&](const std::uint32_t mask) {
    if (arrival != ReadIrq::before && !raised) { raise(); }
    return GpioPadSample{.known = mask, .levels = sample_low & mask};
  });
  const auto first = machine.run_until(horizon);
  require(raised, "The actual GPIO read or its preceding device span supplies the IRQ edge");
  if (arrival == ReadIrq::masked) {
    const auto pending = machine.bus().interrupt_request(unmasked);
    require(first.retired == 4 && machine.cpu().waiting() && machine.cpu().state().x == 1 &&
                !machine.cpu().pending_interrupt() && pending &&
                pending->vector == blofeld::test::irq_vector,
            "A masked IRQ cannot break read continuation or acknowledge its latched edge");
    return;
  }
  require(first.retired == 2 && !machine.cpu().waiting() && machine.cpu().state().x == 0 &&
              machine.cpu().state().pc == entry + 4 &&
              machine.cpu().state().d == read_byte_result && machine.cpu().cycles() == 9,
          "A new IRQ returns after the whole real read, before the next foreground instruction");
  constexpr unsigned interrupt_cycles = 18; // Reset + NOP/LDAA + IRQ entry.
  const auto interrupt = machine.run_until(horizon);
  require(!interrupt.instruction_executed &&
              interrupt.interrupt_vector == blofeld::test::irq_vector && !interrupt.stack_reused &&
              machine.cpu().state().pc == blofeld::test::handler && machine.cpu().state().x == 0 &&
              machine.cpu().cycles() == interrupt_cycles,
          "The pending source enters its actual vector before foreground continuation");
  require(machine.run_until(horizon).retired == 1 && machine.cpu().state().pc == entry + 4 &&
              machine.cpu().state().sp == stack && machine.cpu().state().x == 0,
          "RTI restores the real read continuation exactly once");
  require(machine.run_until(horizon).retired == 2 && machine.cpu().waiting() &&
              machine.cpu().state().x == 1,
          "Foreground execution resumes only after the real interrupt return");
  machine.validate_run_boundary();
}

void gpio_read_revision(const bool during_read) {
  constexpr auto program =
      isa::program(isa::nop,
                   isa::opcode<Opcode::ldaa_extended>(),
                   std::array<Byte, 2>{hc12::ptq >> 8U, hc12::ptq & blofeld::test::byte_mask},
                   isa::inx,
                   isa::wai);
  McuMachine machine(blofeld::test::checkpoint_image(program), BLOFELD_DEFAULT_SLA, contract());
  prepare(machine);
  bool changed = false;
  const auto enable_timer = [&] {
    require(!changed, "The owner changes local scheduling only once");
    changed = true;
    machine.bus().write(hc12::tscr1, timer_on);
  };
  machine.set_span_observer([&](McuSpan /*span*/) {
    if (!during_read && !changed) { enable_timer(); }
  });
  machine.bus().attach_gpio_pad_source(McuPort::q, [&](const std::uint32_t mask) {
    if (during_read && !changed) { enable_timer(); }
    return GpioPadSample{.known = mask, .levels = sample_low & mask};
  });
  const auto first = machine.run_until(horizon);
  require(changed && first.retired == 2 && machine.cpu().state().pc == entry + 4 &&
              machine.cpu().state().x == 0 && machine.cpu().cycles() == 9,
          "A bus mutation during prefix synchronization or input consumption retains its owner "
          "boundary");
  require(machine.run_until(horizon).retired == 2 && machine.cpu().waiting() &&
              machine.cpu().state().x == 1,
          "The next native run uses the actual updated scheduling contract");
  machine.validate_run_boundary();
}

void status_reads_keep_boundary() {
  constexpr unsigned counter = 0x44;
  constexpr auto program = isa::program(
      isa::opcode<Opcode::ldd_extended>(), std::array<Byte, 2>{0, counter}, isa::inx, isa::wai);
  McuMachine machine(blofeld::test::checkpoint_image(program), BLOFELD_DEFAULT_SLA, contract());
  prepare(machine);
  machine.bus().write(hc12::tscr1, timer_on);
  require(
      machine.run_until(horizon).retired == 1 && machine.cpu().state().pc == entry + 3 &&
          machine.cpu().state().d == 0 && machine.cpu().state().x == 0,
      "Counter/status reads retain an owner boundary because they can acknowledge device flags");
  require(machine.run_until(horizon).retired == 2 && machine.cpu().waiting() &&
              machine.cpu().state().x == 1 && machine.cpu().cycles() == 16,
          "Status-read rescheduling retains exact subsequent instruction and clock progress");
  machine.validate_run_boundary();
}

void mapping_retirement() {
  constexpr auto program = isa::program(isa::opcode<Opcode::ldaa_immediate>(),
                                        std::array<Byte, 1>{0x10},
                                        isa::opcode<Opcode::staa_extended>(),
                                        std::array<Byte, 2>{0, hc12::initrg},
                                        isa::opcode<Opcode::ldaa_immediate>(),
                                        std::array<Byte, 1>{timer_on},
                                        isa::opcode<Opcode::staa_extended>(),
                                        std::array<Byte, 2>{0x10, hc12::tscr1},
                                        isa::wai);
  McuMachine machine(blofeld::test::checkpoint_image(program), BLOFELD_DEFAULT_SLA, contract());
  prepare(machine);
  require(machine.run_until(horizon).retired == 2 &&
              machine.bus().inspect_memory(0x1000).space == McuSpace::registers,
          "Mapping writes commit at the complete native instruction boundary");
  require(machine.run_until(horizon).retired == 2 &&
              machine.bus().read(0x1000 + hc12::tscr1) == timer_on,
          "The next native run withdraws overlapping RAM and reaches the relocated peripheral");
}

void unknown_ram() {
  constexpr auto program = isa::program(
      isa::opcode<Opcode::ldd_extended>(), std::array<Byte, 2>{sample_high, 0}, isa::wai);
  McuMachine machine(blofeld::test::checkpoint_image(program), BLOFELD_DEFAULT_SLA, contract());
  prepare(machine);
  const auto fault = rejects([&] { static_cast<void>(machine.run_until(horizon)); });
  if (!fault.contains("read") || !fault.contains("1200") || !fault.contains("8000")) {
    throw Fault("Unknown RAM diagnostic lacks operation/address/PC: " + fault);
  }
  require(rejects([&] { static_cast<void>(machine.run_until(horizon)); }).contains("incomplete"),
          "A failed native instruction cannot silently resume its partial prefix");
}

void flash_grants_follow_controller() {
  McuBus bus(blofeld::test::checkpoint_image(isa::wai));
  const auto readable = [](const std::vector<McuMemoryWindow> &windows) {
    return std::ranges::any_of(windows, [](const auto &window) { return !window.writable; });
  };
  require(readable(bus.memory_windows()), "An idle flash controller grants known program storage");
  const auto before = bus.code_revision();
  constexpr unsigned flash_configuration = 0x103;
  constexpr unsigned key_access = 0x20;
  bus.write(flash_configuration, key_access);
  const auto blocked = bus.memory_windows();
  require(!readable(blocked) && !blocked.empty() && bus.code_revision() != before,
          "KEYACC withdraws every flash alias while keeping actual RAM grants available");
  bus.reset();
  require(readable(bus.memory_windows()),
          "Reset restores program access through the actual controller");
}

void local_event_horizon() {
  constexpr Byte backedge = 0xfe;
  constexpr auto program =
      isa::program(isa::opcode<Opcode::bra_relative>(), std::array<Byte, 1>{backedge});
  auto image = blofeld::test::checkpoint_image(program);
  constexpr unsigned vector = 0xfff0;
  image.bytes.at(vector - entry) = blofeld::test::handler >> 8U;
  image.bytes.at(vector - entry + 1) = 0;
  McuMachine machine(image, BLOFELD_DEFAULT_SLA, contract());
  prepare(machine);
  machine.bus().write(hc12::rti_control, 0x10);
  machine.bus().write(hc12::crg_interrupts, timer_on);
  std::vector<McuSpan> spans;
  machine.set_span_observer([&](const McuSpan span) { spans.push_back(span); });
  const auto result = machine.run_until(horizon);
  constexpr unsigned oscillator_edges = 1024;
  constexpr unsigned branch_cycles = 3;
  constexpr unsigned branches = (oscillator_edges / 2 + branch_cycles - 1) / branch_cycles;
  constexpr unsigned executed_cycles = branches * branch_cycles;
  const auto pending = machine.cpu().pending_interrupt();
  require(result.retired == branches && spans.size() == 2 &&
              spans.front().cycles == oscillator_edges / 2 && spans.back().cycles == 1 &&
              machine.cpu().cycles() == 5 + executed_cycles && pending && pending->vector == vector,
          "A local device event shortens native admission; only the last atomic branch crosses it");
  const auto interrupt = machine.run_until(horizon);
  require(interrupt.interrupt_vector == vector && !interrupt.instruction_executed &&
              !interrupt.stack_reused && machine.cpu().state().pc == blofeld::test::handler &&
              machine.cpu().cycles() == 5 + executed_cycles + 9,
          "The actual timer IRQ enters before another foreground instruction through mapped ROM");
  machine.validate_run_boundary();
}

void wait_and_irq() {
  McuMachine machine(blofeld::test::checkpoint_image(isa::wai), BLOFELD_DEFAULT_SLA, contract());
  prepare(machine);
  require(machine.run_until(horizon).retired == 1 && machine.cpu().waiting(),
          "Native WAI enters its architectural wait");
  std::vector<McuSpan> spans;
  machine.set_span_observer([&](const McuSpan span) { spans.push_back(span); });
  const auto before = machine.run_clock().elapsed_ticks();
  constexpr unsigned idle = 10000;
  require(!machine.run_until(before + (period_ticks * idle)).instruction_executed &&
              spans.size() == 1 && spans.front().waiting && spans.front().cycles == idle &&
              machine.idle_bus_cycles() == idle && machine.cpu().cycles() == 12,
          "An event-free wait advances once without executing or charging guest instructions");
  constexpr Byte edge_irq_enable = 0xc0; // IRQEN | IRQE.
  machine.bus().write(hc12::irq_control, edge_irq_enable);
  machine.bus().update_pin_drive(McuPort::e, 2, {.mask = 2, .levels = 0});
  machine.bus().update_pin_drive(McuPort::e, 2, {.mask = 2, .levels = 2});
  const auto entered = machine.run_until(horizon);
  constexpr unsigned entered_cycles = 18;
  require(entered.interrupt_vector == blofeld::test::irq_vector && entered.stack_reused &&
              !entered.instruction_executed && machine.cpu().cycles() == entered_cycles &&
              machine.cpu().state().pc == blofeld::test::handler,
          "A real external IRQ wakes through the mapped vector and reuses the WAI frame");
  require(machine.run_until(horizon).retired == 1 && machine.cpu().state().sp == stack &&
              machine.cpu().state().pc == entry + 1 && !machine.cpu().pending_interrupt(),
          "Mapped-vector entry acknowledges the latched edge; RTI restores its real frame once");
  machine.validate_run_boundary();
}

void observer_ownership_and_recovery() {
  McuMachine machine(blofeld::test::checkpoint_image(isa::wai), BLOFELD_DEFAULT_SLA, contract());
  prepare(machine);
  static_cast<void>(machine.run_until(horizon));
  unsigned callbacks = 0;
  machine.set_span_observer([&](const McuSpan span) {
    require(span.waiting && span.cycles == 10,
            "The observer sees the completed interval rather than a forecast bus slot");
    require(
        rejects([&] { machine.set_span_observer({}); }).contains("inactive") &&
            rejects([&] { static_cast<void>(machine.run_until(horizon)); }).contains("inactive") &&
            rejects([&] { machine.cpu().step(); }).contains("Recursive"),
        "A device observer cannot replace or recursively enter its active execution owner");
    ++callbacks;
  });
  require(machine.advance_wait(10) == 10 && callbacks == 1 && !machine.timing_faulted(),
          "Caught ownership violations do not damage a completed device-time span");
  machine.set_span_observer([](McuSpan /*span*/) { throw Fault("Own span observer failure"); });
  constexpr unsigned failed_idle = 20;
  require(rejects([&] { machine.advance_wait(10); }).contains("Own span observer failure") &&
              machine.timing_faulted() && machine.idle_bus_cycles() == failed_idle,
          "An observer failure retains actual completed idle time and quarantines execution");
  require(rejects([&] { static_cast<void>(machine.run_until(horizon)); }).contains("healthy"),
          "A failed device-time boundary requires coordinated recovery");
  constexpr unsigned reset_edges = 128 + 3 + 64;
  machine.begin_system_reset(McuResetSource::external, 3);
  require(machine.advance_system_reset(reset_edges, McuResetPad::high) == reset_edges &&
              !machine.system_reset_active() && !machine.timing_faulted() &&
              machine.cpu().state().pc == entry && callbacks == 1,
          "A real system reset replaces native state and releases captured external observers");
  machine.validate_run_boundary();
}
}

int main() {
  try {
    resident_loop();
    io_boundary();
    gpio_reads_continue();
    for (const auto arrival : {ReadIrq::before, ReadIrq::during, ReadIrq::masked}) {
      gpio_read_irq(arrival);
    }
    for (const bool during_read : {false, true}) { gpio_read_revision(during_read); }
    status_reads_keep_boundary();
    mapping_retirement();
    unknown_ram();
    flash_grants_follow_controller();
    local_event_horizon();
    wait_and_irq();
    observer_ownership_and_recovery();
    std::cout << "Native MCU spans, direct RAM, I/O, remapping, unknown reads and IRQ/WAI passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
