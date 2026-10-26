// SPDX-License-Identifier: GPL-3.0-only
#include "call_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <vector>

namespace blofeld::test::call {
void Machine::prepare(const Input input, const Hcs08PrefetchContract selected) {
  memory = Memory{};
  memory.bytes.fill(nop);
  memory.known.set();
  memory.writable.set();
  const std::array<std::uint8_t, 3> code{
      static_cast<std::uint8_t>(input.spec.opcode),
      static_cast<std::uint8_t>(input.spec.bytes == 3 ? input.offset >> 8U : input.offset),
      static_cast<std::uint8_t>(input.offset)};
  memory.install(handler, std::span(code).first(input.spec.bytes));
  const std::array<std::uint8_t, 2> destination{handler >> 8U, handler & byte_mask};
  memory.install(vector, destination);
  constexpr unsigned preserved_a = 0xa5;
  cpu.set_state({.pc = entry,
                 .hx = static_cast<std::uint16_t>(input.hx),
                 .sp = static_cast<std::uint16_t>(input.sp + 5),
                 .a = preserved_a,
                 .ccr_known = static_cast<std::uint8_t>(input.known | flag_i),
                 .ccr = static_cast<std::uint8_t>(input.flags & ~flag_i),
                 .a_known = static_cast<std::uint8_t>(input.a_known),
                 .hx_known = static_cast<std::uint16_t>(input.hx_known)});
  memory.interrupt = {.vector = vector};
  require(cpu.start_irq_bus(Hcs08IrqBusContract::reference_unstretched_sequence), "Own IRQ queue");
  for (unsigned period = 0; period < 11; ++period) { cpu.advance_irq_bus(); }
  cpu.start_irq_program(selected);
  memory.events.clear();
  memory.native_starts = 0;
  memory.retired = 0;
}
}

namespace {
using namespace blofeld;
using namespace blofeld::test::call;
unsigned cases = 0, copies = 0, refusals = 0;

void check(Machine &machine, const Input input) {
  machine.prepare(input);
  const auto before = machine.cpu.state();
  auto expected = before;
  std::vector<Event> events;
  const unsigned next = handler + input.spec.bytes;
  unsigned next_read = handler + 3;
  for (unsigned period = 1; period <= input.spec.periods; ++period) {
    if (period == input.spec.first || period == input.spec.first + 1) {
      const unsigned index = period - input.spec.first;
      const unsigned value = index == 0 ? next & byte_mask : next >> 8U;
      events.push_back({.period = period,
                        .access = {.address = (input.sp + space_bytes - index) & word_mask,
                                   .value = value,
                                   .write = true}});
      expected.sp = static_cast<std::uint16_t>(input.sp + space_bytes - index - 1);
      if (index == 1) {
        expected.pc = static_cast<std::uint16_t>(target(input));
        next_read = target(input);
      }
    } else {
      events.push_back({.period = period,
                        .access = {.address = next_read,
                                   .value = machine.memory.bytes[next_read],
                                   .write = false}});
      next_read = (next_read + 1) & word_mask;
    }
    machine.tick();
    const auto queue = owned(machine.cpu);
    require(machine.memory.events == events && machine.cpu.state() == expected &&
                machine.memory.native_starts == static_cast<unsigned>(period >= input.spec.first) &&
                machine.memory.retired == static_cast<unsigned>(period == input.spec.periods) &&
                queue.native_executed == (period > input.spec.first) &&
                queue.completed_periods == period && queue.attempted_periods == period,
            "Literal call periods, PCL then PCH, native SP/PC and preserved register knowledge");
  }
  const auto queue = owned(machine.cpu);
  require(queue.queued_bytes == 3 && queue.ready_bytes == 2 && queue.program_byte_in_flight &&
              queue.head == target(input) && queue.discarded && queue.discarded->address == next &&
              (input.spec.bytes != 1 ||
               (queue.jump_discarded && queue.jump_discarded->address == next + 1)),
          "Actual sequential successors discarded; native target owns three refill reads");
  ++cases;
}

void profiles(Machine &machine, const Spec spec) {
  for (unsigned value = 0; value < byte_values; ++value) {
    check(machine, {.spec = spec, .flags = value});
    check(machine, {.spec = spec, .known = value});
  }
  for (const unsigned sp : {0U, 1U, stack, word_mask}) { check(machine, {.spec = spec, .sp = sp}); }
  for (const unsigned offset : {0U, 1U, 127U, 128U, byte_mask}) {
    check(machine, {.spec = spec, .offset = offset});
  }
  if (spec.bytes == 3) { check(machine, {.spec = spec, .offset = word_mask}); }
  for (const unsigned mask : {0U, flag_n, byte_mask}) {
    check(machine, {.spec = spec, .a_known = mask});
    if (spec.opcode < first_indexed_call) { check(machine, {.spec = spec, .hx_known = mask}); }
  }
  for (const unsigned hx : {0U, word_mask}) {
    for (const unsigned offset : {0U, 1U, byte_mask}) {
      check(machine, {.spec = spec, .offset = offset, .hx = hx});
    }
  }
}

void snapshots(Machine &parent, Machine &fresh, const Input input) {
  for (unsigned cut = 0; cut <= input.spec.periods; ++cut) {
    for (unsigned repeat = 0; repeat < 2; ++repeat) {
      parent.prepare(input);
      while (owned(parent.cpu).completed_periods < cut) { parent.tick(); }
      const auto saved = parent.cpu.checkpoint();
      fresh.memory = parent.memory;
      fresh.cpu.restore_checkpoint(saved);
      finish(parent, input);
      finish(fresh, input);
      require(parent.cpu.state() == fresh.cpu.state() &&
                  parent.cpu.prefetch_state() == fresh.cpu.prefetch_state() &&
                  parent.memory.events == fresh.memory.events &&
                  parent.memory.bytes == fresh.memory.bytes &&
                  fresh.cpu.state().pc == target(input) &&
                  fresh.cpu.state().sp == static_cast<std::uint16_t>(input.sp + space_bytes - 2) &&
                  fresh.memory.bytes[input.sp] == ((handler + input.spec.bytes) & byte_mask) &&
                  fresh.memory.bytes[(input.sp + word_mask) & word_mask] == (handler >> 8U),
              "Fresh captured native call resumes its real remaining stack byte and target");
      ++copies;
    }
  }
}

void changed_code(Machine &parent, Machine &fresh, const Spec spec) {
  const Input input{.spec = spec};
  parent.prepare(input);
  while (owned(parent.cpu).completed_periods < spec.first) { parent.tick(); }
  const auto saved = parent.cpu.checkpoint();
  fresh.memory = parent.memory;
  const std::array<std::uint8_t, 3> replacement{nop, nop, nop};
  fresh.memory.install(handler, replacement);
  fresh.cpu.restore_checkpoint(saved);
  finish(fresh, input);
  require(fresh.cpu.state().pc == target(input) && fresh.cpu.state().sp == stack - 2 &&
              fresh.memory.bytes[stack] == ((handler + spec.bytes) & byte_mask) &&
              fresh.memory.bytes[stack - 1] == (handler >> 8U),
          "Captured native call retains its exact opcode, return temporary and target after RAM "
          "code changes");
  fresh.cpu.set_state({.pc = handler, .sp = stack, .ccr = flags_fixed | flag_i});
  fresh.cpu.step();
  require(fresh.cpu.state().pc == handler + 1 && fresh.cpu.state().sp == stack,
          "Later instruction admission decodes the actual replacement NOP");
  ++cases;
}

void strict(Machine &machine, const Spec spec) {
  machine.prepare({.spec = spec}, Hcs08PrefetchContract::reference_modify_program_cycles);
  const auto before = machine.cpu.state();
  const auto error = rejected([&] { machine.tick(); });
  require(error.contains("unqualified") && machine.cpu.faulted() && machine.memory.events.empty() &&
              machine.cpu.state() == before,
          "Prior RMW contract rejects calls without stack effects");
  ++refusals;
  if (spec.opcode < first_indexed_call) { return; }
  for (const unsigned mask : {0U, byte_mask}) {
    machine.prepare({.spec = spec, .hx_known = mask});
    while (owned(machine.cpu).completed_periods < spec.first - 1) { machine.tick(); }
    const auto events = machine.memory.events;
    const auto bad = rejected([&] { machine.tick(); });
    require(bad.contains("unknown operand") && bad.contains("pc=") && bad.contains("address=") &&
                machine.memory.events == events && machine.cpu.faulted(),
            "Unknown H:X cannot supply a native call target or commit return bytes");
    ++refusals;
  }
}

void failures(Machine &machine, Machine &fresh, const Spec spec) {
  const Input input{.spec = spec};
  for (unsigned index = 0; index < 2; ++index) {
    for (const bool after : {false, true}) {
      machine.prepare(input);
      while (owned(machine.cpu).completed_periods < spec.first + index - 1) { machine.tick(); }
      const auto saved = machine.cpu.checkpoint();
      const auto memory = machine.memory;
      machine.memory.refused_write = stack - index;
      machine.memory.after = after;
      const auto error = rejected([&] { machine.tick(); });
      require(error.contains("call write") && error.contains("address=") && error.contains("pc=") &&
                  machine.cpu.faulted() &&
                  machine.memory.events.size() ==
                      memory.events.size() + static_cast<unsigned>(after),
              "Before/after PCL/PCH failure preserves exact bus prefix and quarantines CPU");
      static_cast<void>(rejected([&] { machine.tick(); }));
      static_cast<void>(rejected([&] { static_cast<void>(machine.cpu.checkpoint()); }));
      fresh.memory = memory;
      fresh.cpu.restore_checkpoint(saved);
      finish(fresh, input);
      require(fresh.cpu.state().pc == target(input) && fresh.cpu.state().sp == stack - 2,
              "Trusted joint restore completes native call after bus failure");
      ++refusals;
    }
  }
}
}

int main() {
  try {
    Machine parent;
    Machine fresh;
    for (const auto spec : specs) {
      profiles(parent, spec);
      snapshots(parent, fresh, {.spec = spec});
      snapshots(parent, fresh, {.spec = spec, .sp = 0});
      changed_code(parent, fresh, spec);
      strict(parent, spec);
      failures(parent, fresh, spec);
      // Real stack writes may overlap already captured instruction bytes.
      snapshots(parent, fresh, {.spec = spec, .sp = handler + spec.bytes - 1});
    }
    std::cout << "Native call periods: " << cases << " vectors, " << copies
              << " fresh continuations, " << refusals << " strict/failure controls\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
