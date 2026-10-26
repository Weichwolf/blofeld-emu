// SPDX-License-Identifier: GPL-3.0-only
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "subroutine_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <vector>

namespace blofeld::test::subroutine {
void Machine::prepare(const Input input, const Hcs08PrefetchContract selected) {
  memory = Memory{};
  memory.bytes.fill(nop);
  memory.known.set();
  memory.writable.set();
  const std::array code{static_cast<std::uint8_t>(input.opcode)};
  memory.install(handler, code);
  const std::array<std::uint8_t, 2> destination{handler >> 8U, handler & byte_mask};
  memory.install(vector, destination);
  constexpr unsigned preserved_h = 0x1200;
  cpu.set_state({.pc = entry,
                 .hx = static_cast<std::uint16_t>(preserved_h | input.value),
                 .sp = static_cast<std::uint16_t>(input.sp + 5),
                 .a = static_cast<std::uint8_t>(input.value),
                 .ccr_known = static_cast<std::uint8_t>(input.known | flag_i),
                 .ccr = static_cast<std::uint8_t>(input.flags & ~flag_i),
                 .a_known = static_cast<std::uint8_t>(input.a_known),
                 .hx_known = static_cast<std::uint16_t>(input.hx_known)});
  memory.interrupt = {.vector = vector};
  require(cpu.start_irq_bus(Hcs08IrqBusContract::reference_unstretched_sequence), "Own IRQ queue");
  for (unsigned period = 0; period < 8; ++period) { cpu.advance_irq_bus(); }
  // Own RAM code may overlap the synthetic entry frame; install before actual program reads.
  memory.install(handler, code);
  for (unsigned period = 0; period < 3; ++period) { cpu.advance_irq_bus(); }
  cpu.start_irq_program(selected);
  if (input.opcode == rts) {
    memory.bytes[static_cast<std::uint16_t>(input.sp + 1U)] =
        static_cast<std::uint8_t>(input.target >> 8U);
    memory.bytes[static_cast<std::uint16_t>(input.sp + 2U)] =
        static_cast<std::uint8_t>(input.target);
    memory.known.set(static_cast<std::uint16_t>(input.sp + 1U));
    memory.known.set(static_cast<std::uint16_t>(input.sp + 2U));
  }
  memory.events.clear();
  memory.accesses.clear();
  memory.native_starts = 0;
  memory.retired = 0;
}
}

namespace {
using namespace blofeld;
using namespace blofeld::test::subroutine;
unsigned cases = 0, copies = 0, refusals = 0;
constexpr unsigned negative = 0x80;

Hcs08State result(const Hcs08State before, const Input input) {
  auto expected = before;
  expected.pc = static_cast<std::uint16_t>(input.opcode == rts ? input.target : handler + 1);
  if (input.opcode == rts) {
    expected.sp = static_cast<std::uint16_t>(input.sp + 2U);
    return expected;
  }
  const unsigned value =
      input.opcode == decrement_a ? (input.value + byte_mask) & byte_mask : input.value;
  const unsigned flags = ((value & negative) != 0 ? flag_n : 0U) | (value == 0 ? flag_z : 0U) |
                         (input.opcode == decrement_a && input.value == negative ? flag_v : 0U);
  expected.ccr = static_cast<std::uint8_t>((before.ccr & ~vnz) | flags);
  expected.ccr_known |= vnz;
  if (input.opcode == decrement_a) { expected.a = static_cast<std::uint8_t>(value); }
  return expected;
}

void check(Machine &machine, const Input input) {
  machine.prepare(input);
  const auto before = machine.cpu.state();
  std::vector<Event> events;
  for (unsigned period = 1; period <= periods(input); ++period) {
    if (input.opcode != rts) {
      events.push_back(
          {.period = period, .access = {.address = handler + 3, .value = nop, .write = false}});
    } else if (period <= 2) {
      const unsigned value = period == 1 ? input.target >> 8U : input.target & byte_mask;
      events.push_back(
          {.period = period,
           .access = {.address = (input.sp + period) & word_mask, .value = value, .write = false}});
    } else if (period >= 4) {
      const unsigned address = (input.target + period - 4) & word_mask;
      events.push_back(
          {.period = period,
           .access = {.address = address, .value = machine.memory.bytes[address], .write = false}});
    }
    machine.tick();
    const bool executed = input.opcode != rts || period >= 3;
    const auto queue = owned(machine.cpu);
    require(machine.memory.events == events &&
                machine.cpu.state() == (executed ? result(before, input) : before) &&
                machine.memory.native_starts == static_cast<unsigned>(executed) &&
                machine.memory.retired == static_cast<unsigned>(period == periods(input)) &&
                queue.native_executed == executed && queue.completed_periods == period &&
                queue.attempted_periods == period,
            "Independent literal p/u/u/f/p/p/p references and native result/flags/SP/PC");
  }
  ++cases;
}

void vectors(Machine &machine) {
  constexpr unsigned high = 0x9100;
  constexpr unsigned low = 0xa5;
  for (unsigned value = 0; value < byte_values; ++value) {
    check(machine, {.target = (value << 8U) | low});
    check(machine, {.target = high | value});
    check(machine, {.flags = value});
    check(machine, {.known = value});
    for (const unsigned opcode : {decrement_a, test_x}) {
      check(machine, {.opcode = opcode, .value = value});
      check(machine, {.opcode = opcode, .value = negative, .flags = value});
      check(machine, {.opcode = opcode, .known = value});
    }
  }
  for (const unsigned sp : {0U, word_mask - 2, word_mask - 1, word_mask}) {
    for (const unsigned destination : {0U, 1U, word_mask - 1, word_mask}) {
      check(machine, {.target = destination, .sp = sp});
    }
  }
  for (const unsigned a_mask : {0U, flag_n, byte_mask}) {
    for (const unsigned hx_mask : {0U, byte_mask, word_mask}) {
      check(machine, {.a_known = a_mask, .hx_known = hx_mask});
    }
    for (const unsigned hx_mask : {byte_mask, word_mask}) {
      check(machine, {.opcode = test_x, .a_known = a_mask, .hx_known = hx_mask});
    }
    check(machine, {.opcode = decrement_a, .hx_known = a_mask});
  }
}

void snapshots(Machine &parent, Machine &fresh, const Input input) {
  constexpr std::uint8_t poison = 0x55;
  for (unsigned cut = 0; cut <= periods(input); ++cut) {
    for (unsigned repeat = 0; repeat < 2; ++repeat) {
      parent.prepare(input);
      const auto expected = result(parent.cpu.state(), input);
      while (owned(parent.cpu).completed_periods < cut) { parent.tick(); }
      const auto saved = parent.cpu.checkpoint();
      fresh.memory = parent.memory;
      fresh.cpu.restore_checkpoint(saved);
      if (input.opcode == rts && input.sp == stack && cut > 0 && cut < 3) {
        for (unsigned index = 1; index <= cut; ++index) {
          parent.memory.bytes[input.sp + index] = poison;
          fresh.memory.bytes[input.sp + index] = poison;
        }
      }
      finish(parent, input);
      finish(fresh, input);
      require(parent.cpu.state() == expected && fresh.cpu.state() == expected &&
                  parent.cpu.prefetch_state() == fresh.cpu.prefetch_state() &&
                  parent.memory.events == fresh.memory.events &&
                  parent.memory.bytes == fresh.memory.bytes,
              "Fresh return resumes actual byte latches without rereading poisoned RAM or "
              "substituting PC");
      ++copies;
    }
  }
}

void strict(Machine &machine) {
  for (const unsigned opcode : {rts, decrement_a, test_x}) {
    machine.prepare({.opcode = opcode}, Hcs08PrefetchContract::reference_call_program_cycles);
    const auto before = machine.cpu.state();
    const auto error = rejected([&] { machine.tick(); });
    require(error.contains("unqualified") && machine.cpu.faulted() &&
                machine.cpu.state() == before && machine.memory.events.empty(),
            "Old call contract rejects new consumers before bus effects");
    ++refusals;
  }
  for (const unsigned mask : {0U, flag_n}) {
    for (const unsigned opcode : {decrement_a, test_x}) {
      machine.prepare({.opcode = opcode,
                       .a_known = opcode == decrement_a ? mask : byte_mask,
                       .hx_known = opcode == test_x ? mask : word_mask});
      const auto error = rejected([&] { machine.tick(); });
      require(error.contains("unknown operand") && error.contains("address=") &&
                  error.contains("pc=") && machine.cpu.faulted(),
              "Unknown consumed A/X cannot manufacture flags/results");
      ++refusals;
    }
  }
}

void failures(Machine &machine, Machine &fresh) {
  for (unsigned index = 1; index <= 2; ++index) {
    for (const bool after : {false, true}) {
      const Input input;
      machine.prepare(input);
      while (owned(machine.cpu).completed_periods < index - 1) { machine.tick(); }
      const auto saved = machine.cpu.checkpoint();
      const auto memory = machine.memory;
      machine.memory.refused_read = stack + index;
      machine.memory.after = after;
      const auto error = rejected([&] { machine.tick(); });
      require(
          error.contains("return read") && error.contains("pc=") && error.contains("address=") &&
              machine.cpu.faulted() &&
              machine.memory.events.size() == memory.events.size() + static_cast<unsigned>(after),
          "Before/after actual return-byte failure retains exact bus prefix and quarantines CPU");
      static_cast<void>(rejected([&] { machine.tick(); }));
      static_cast<void>(rejected([&] { static_cast<void>(machine.cpu.checkpoint()); }));
      fresh.memory = memory;
      fresh.cpu.restore_checkpoint(saved);
      finish(fresh, input);
      require(fresh.cpu.state().pc == input.target && fresh.cpu.state().sp == stack + 2,
              "Trusted joint restore consumes actual retained/read bytes once");
      ++refusals;
    }
  }
}
}

int main() {
  try {
    Machine parent;
    Machine fresh;
    vectors(parent);
    for (const unsigned sp : {0U, stack, word_mask - 1, word_mask, handler - 1}) {
      snapshots(parent, fresh, {.sp = sp});
    }
    for (const unsigned opcode : {decrement_a, test_x}) {
      snapshots(parent, fresh, {.opcode = opcode});
    }
    strict(parent);
    failures(parent, fresh);
    std::cout << "Native subroutine periods: " << cases << " vectors, " << copies
              << " fresh continuations, " << refusals << " strict/failure controls\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
