// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "cpu12/memory.hpp"
#include "isa.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld;
namespace isa = cpu12::isa;
namespace reg = hc12;
using cpu12::Byte;
using isa::Opcode;
constexpr unsigned entry = 0x8000, handler = 0x8100, stack = 0x2000;
constexpr unsigned irq_vector = 0xfff2, compare_vector = 0xffee;
constexpr unsigned protection = 0x7f0d, security = 0x7f0f, unsecured = 0xfe;
constexpr unsigned oscillator = 12000000, horizon = 1000000;
constexpr unsigned tcnt = 0x44, tie = 0x4c, tscr2 = 0x4d, tflg2 = 0x4f;
constexpr unsigned pacnt = 0x62, sample = 0x1234, word_wrap = 65536;
constexpr Byte byte_mask = 0xff, masked = 0xd0, unmasked = 0xc0, timer_on = 0x80;
constexpr Byte fast_clear = 0x90, sample_high = 0x12, sample_low = 0x34;

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw std::runtime_error(std::string(message)); }
}

constexpr std::array<Byte, 2> address(const unsigned value) {
  return {static_cast<Byte>(value >> 8U), static_cast<Byte>(value & byte_mask)};
}

McuFlashImage image(const std::span<const Byte> program) {
  std::array<Byte, McuFlashImage::byte_count> bytes;
  bytes.fill(isa::nop.front());
  std::ranges::copy(program, bytes.begin());
  bytes[handler - entry] = isa::rti.front();
  for (const auto vector : {irq_vector, compare_vector}) {
    bytes[vector - entry] = handler >> 8U;
    bytes[vector - entry + 1] = 0;
  }
  bytes[protection] = byte_mask;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  return McuFlashImage::from_known(bytes);
}

McuRunContract contract() {
  return {.clock = {.oscillator_hz = oscillator,
                    .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = McuRunMemory::internal_flash_reads};
}

struct Fixture {
  explicit Fixture(const std::span<const Byte> program) : machine(image(program), {}, contract()) {
    machine.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
    machine.cpu().set_state({.pc = entry, .d = sample, .sp = stack, .ccr = masked});
  }

  [[nodiscard]] unsigned word(const unsigned location) {
    return static_cast<unsigned>(
        machine.bus().read_memory({.address = static_cast<std::uint16_t>(location), .size = 2}));
  }

  void overflow() {
    machine.bus().write(reg::tscr1, fast_clear);
    machine.bus().write(tscr2, timer_on);
    machine.bus().advance_timer(word_wrap);
    require(machine.bus().read(tflg2) == timer_on && machine.bus().interrupt_request(unmasked),
            "Establish a real overflow flag and enabled request while the CPU masks IRQ");
  }

  McuMachine machine;
};

struct Write {
  std::uint16_t address;
  Byte value;
  std::uint64_t cycles;
  bool operator==(const Write &) const = default;
};

void counter_storage(const bool bytewise) {
  constexpr auto bytes = isa::program(isa::opcode<Opcode::staa_extended>(),
                                      address(pacnt),
                                      isa::opcode<Opcode::stab_extended>(),
                                      address(pacnt + 1));
  constexpr auto word = isa::program(isa::opcode<Opcode::std_extended>(), address(pacnt));
  constexpr auto suffix =
      isa::program(isa::opcode<Opcode::ldd_extended>(), address(pacnt), isa::inx, isa::wai);
  constexpr auto byte_program = isa::program(bytes, suffix);
  constexpr auto word_program = isa::program(word, suffix);
  Fixture fixture(bytewise ? std::span<const Byte>{byte_program}
                           : std::span<const Byte>{word_program});
  std::vector<Write> observed;
  fixture.machine.bus().set_observer([&](const BusEvent &event) {
    observed.push_back({.address = event.address,
                        .value = event.value,
                        .cycles = fixture.machine.run_clock().bus_cycles()});
  });
  const auto result = fixture.machine.run_until(horizon);
  const auto cpu = fixture.machine.cpu().state();
  const std::array wanted{
      Write{.address = pacnt, .value = sample_high, .cycles = 5},
      Write{.address = pacnt + 1, .value = sample_low, .cycles = bytewise ? 8U : 5U}};
  const auto cycles = bytewise ? 22U : 19U;
  require(
      result.retired == (bytewise ? 5U : 4U) && fixture.machine.cpu().waiting() &&
          cpu.d == sample && cpu.x == 1 && cpu.sp == stack - 9 && cpu.ccr == masked &&
          fixture.word(pacnt) == sample && std::ranges::equal(observed, wanted) &&
          fixture.machine.cpu().cycles() == cycles &&
          fixture.machine.run_clock().bus_cycles() == cycles,
      "Stored timer byte/word accesses continue natively with exact writes, prefixes and state");
}

void counter_acknowledgements(const bool write) {
  constexpr auto read_program =
      isa::program(isa::opcode<Opcode::ldd_extended>(), address(tcnt), isa::inx, isa::wai);
  constexpr auto write_program = isa::program(isa::opcode<Opcode::std_extended>(),
                                              address(tcnt),
                                              isa::opcode<Opcode::ldd_extended>(),
                                              address(tcnt),
                                              isa::inx,
                                              isa::wai);
  Fixture fixture(write ? std::span<const Byte>{write_program}
                        : std::span<const Byte>{read_program});
  fixture.overflow();
  const auto result = fixture.machine.run_until(horizon);
  require(result.retired == (write ? 4U : 3U) && fixture.machine.cpu().waiting() &&
              fixture.machine.cpu().state().d == (write ? 3U : 0U) &&
              fixture.machine.cpu().state().x == 1 && fixture.machine.bus().read(tflg2) == 0 &&
              !fixture.machine.bus().interrupt_request(unmasked) &&
              fixture.word(tcnt) == (write ? 14U : 11U),
          "Counter reads/ignored writes clear TOF, retain counter time and do not need a scheduler "
          "exit");
}

void write_observer_irq(const bool masked_irq, const bool prefix_irq) {
  constexpr auto program = isa::program(
      isa::nop, isa::opcode<Opcode::std_extended>(), address(pacnt), isa::inx, isa::wai);
  Fixture fixture(program);
  fixture.machine.cpu().set_state(
      {.pc = entry, .d = sample, .sp = stack, .ccr = masked_irq ? masked : unmasked});
  fixture.machine.bus().write(reg::irq_control, unmasked);
  bool raised = false;
  const auto raise = [&] {
    if (raised) { return; }
    raised = true;
    fixture.machine.bus().update_pin_drive(McuPort::e, 2, {.mask = 2, .levels = 0});
    fixture.machine.bus().update_pin_drive(McuPort::e, 2, {.mask = 2, .levels = 2});
  };
  fixture.machine.set_span_observer([&](McuSpan /*span*/) {
    if (prefix_irq) { raise(); }
  });
  fixture.machine.bus().set_observer([&](const BusEvent &event) {
    if (!prefix_irq && event.address == pacnt) { raise(); }
  });
  const auto result = fixture.machine.run_until(horizon);
  require(raised && fixture.word(pacnt) == sample,
          "A prefix/write event supplies a real IRQ without splitting the atomic word write");
  if (masked_irq) {
    require(
        result.retired == 4 && fixture.machine.cpu().waiting() &&
            fixture.machine.cpu().state().x == 1 &&
            fixture.machine.bus().interrupt_request(unmasked),
        "A masked IRQ remains pending without forcing a harmless write out of native execution");
    return;
  }
  require(result.retired == 2 && fixture.machine.cpu().state().pc == entry + 4 &&
              fixture.machine.cpu().state().x == 0 && fixture.machine.cpu().cycles() == 9,
          "An eligible IRQ exits after the complete native write, before foreground INX");
  const auto interrupt = fixture.machine.run_until(horizon);
  require(interrupt.interrupt_vector == irq_vector && fixture.machine.cpu().state().pc == handler &&
              fixture.machine.cpu().state().x == 0,
          "The actual interrupt vector runs before foreground continuation");
}

void prefix_mutation() {
  constexpr auto program = isa::program(
      isa::nop, isa::opcode<Opcode::std_extended>(), address(pacnt), isa::inx, isa::wai);
  Fixture fixture(program);
  bool changed = false;
  fixture.machine.set_span_observer([&](McuSpan /*span*/) {
    if (!changed) {
      changed = true;
      fixture.machine.bus().write(reg::tscr1, timer_on);
    }
  });
  const auto result = fixture.machine.run_until(horizon);
  require(changed && result.retired == 2 && fixture.machine.cpu().state().x == 0 &&
              fixture.machine.cpu().state().pc == entry + 4 && fixture.word(pacnt) == sample,
          "A new deadline during prefix synchronization still ends a classified counter write");
  require(fixture.machine.run_until(horizon).retired == 2 && fixture.machine.cpu().waiting(),
          "Continuation adopts the actual newly enabled timer horizon");
}

void compare_write_reschedules() {
  constexpr auto program = isa::program(isa::opcode<Opcode::ldd_immediate>(),
                                        address(4),
                                        isa::opcode<Opcode::std_extended>(),
                                        address(reg::timer_channel0),
                                        isa::inx,
                                        isa::wai);
  Fixture fixture(program);
  fixture.machine.cpu().set_state({.pc = entry, .sp = stack, .ccr = unmasked});
  fixture.machine.bus().write(reg::timer_base, 1);
  constexpr unsigned old_compare = 100;
  fixture.machine.bus().write_memory(
      {.address = reg::timer_channel0, .size = 2, .value = old_compare});
  fixture.machine.bus().write(reg::tscr1, timer_on);
  fixture.machine.bus().write(tie, 1);
  fixture.machine.bus().advance_timer(1);
  const auto result = fixture.machine.run_until(horizon);
  require(result.retired == 2 && fixture.machine.cpu().state().x == 0 &&
              fixture.machine.cpu().state().pc == entry + 6,
          "A compare write cannot continue under its obsolete later horizon");
  const auto interrupt = fixture.machine.run_until(horizon);
  require(interrupt.interrupt_vector == compare_vector &&
              fixture.machine.cpu().state().pc == handler && fixture.machine.cpu().state().x == 0,
          "The newly earlier compare event preempts the next foreground instruction");
}

void write_classification() {
  Fixture fixture(isa::wai);
  auto &bus = fixture.machine.bus();
  const MemoryWrite write{.address = pacnt, .size = 2, .value = sample};
  const auto revision = bus.revision();
  bus.write_memory(write);
  require(!bus.write_requires_boundary(write, revision),
          "A real completed PACNT word is classified");
  bus.write(reg::tscr1, timer_on);
  require(bus.write_requires_boundary(write, revision),
          "An extra mutation invalidates continuation");
  require(bus.write_requires_boundary(write, std::numeric_limits<std::uint64_t>::max()) &&
              bus.write_requires_boundary({.address = pacnt, .size = 0}, bus.revision()) &&
              bus.write_requires_boundary({.address = pacnt, .size = 3}, bus.revision()),
          "Invalid widths and exhausted revision arithmetic remain conservative");
}
}

int main() {
  try {
    for (const bool option : {false, true}) {
      counter_storage(option);
      counter_acknowledgements(option);
      write_observer_irq(option, false);
      write_observer_irq(option, true);
    }
    prefix_mutation();
    compare_write_reschedules();
    write_classification();
    std::cout << "Native timer I/O preserves prefixes, effects, IRQs and rescheduling\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
