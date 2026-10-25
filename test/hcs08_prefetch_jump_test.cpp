// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
constexpr unsigned handler = 0x9000, vector = 0xffe0, stack = 0x240;
constexpr unsigned nop = 0x9d, accumulator = 0x5a, index = 0xff80, partial = 0xef;
constexpr unsigned direct = 0xbc, extended = 0xcc, indexed16 = 0xdc;
constexpr unsigned indexed8 = 0xec, indexed = 0xfc;
constexpr auto contract = Hcs08PrefetchContract::reference_jump_program_cycles;

struct Form {
  unsigned opcode, bytes, periods;
};

constexpr std::array<Form, 5> forms{{{.opcode = direct, .bytes = 2, .periods = 3},
                                     {.opcode = extended, .bytes = 3, .periods = 4},
                                     {.opcode = indexed16, .bytes = 3, .periods = 4},
                                     {.opcode = indexed8, .bytes = 2, .periods = 3},
                                     {.opcode = indexed, .bytes = 1, .periods = 3}}};
constexpr std::array<unsigned, 13> offsets{
    0, 1, 0x7f, 0x80, 0xff, 0x100, 0x7fff, 0x8000, 0xff00, 0xff7f, 0xff80, 0xfffe, 0xffff};

struct Input {
  Form form;
  unsigned offset = 0, flags = flags_fixed, known = byte_mask, location = handler;
  unsigned a_known = byte_mask, hx_known = word_mask;
};

bool uses_index(const Input input) {
  return input.form.opcode == indexed || input.form.opcode == indexed8 ||
         input.form.opcode == indexed16;
}

unsigned execution_phase(const Input input) {
  return input.form.bytes == 3 ? 2U : 1U;
}

std::uint16_t destination(const Input input) {
  const auto opcode = input.form.opcode;
  unsigned offset = input.form.bytes == 2 ? input.offset & byte_mask : input.offset;
  if (opcode == indexed) { offset = 0; }
  return static_cast<std::uint16_t>(offset + (uses_index(input) ? index : 0U));
}

unsigned read_address(const Input input, const unsigned phase) {
  if (phase < execution_phase(input)) {
    return static_cast<std::uint16_t>(input.location + input.form.bytes);
  }
  return static_cast<std::uint16_t>(destination(input) + phase - execution_phase(input));
}

class MemoryOwner final : public MaskedMemory {
public:
  std::optional<unsigned> refused;
  bool after = false;

  std::uint8_t read(const std::uint16_t address) override {
    if (refused == address && !after) { fail(address, "own before-JMP-read refusal"); }
    const auto value = Memory::read(address);
    if (refused == address && after) { fail(address, "own after-JMP-read refusal"); }
    return value;
  }
};

Hcs08PrefetchState owned(const Hcs08 &cpu) {
  const auto q = cpu.prefetch_state();
  if (!q) { throw Fault("Missing actual JMP program queue"); }
  return *q;
}

struct Machine {
  MemoryOwner memory;
  Hcs08 cpu{memory, BLOFELD_HCS08_SLA};

  void prepare(const Input input, const Hcs08PrefetchContract selected = contract) {
    memory.refused.reset();
    memory.bytes.fill(nop);
    memory.partial_known.fill(0);
    memory.known.set();
    memory.writable.set();
    const std::array<std::uint8_t, 2> target{static_cast<std::uint8_t>(input.location >> 8U),
                                             static_cast<std::uint8_t>(input.location)};
    const unsigned first = input.form.bytes == 3 ? input.offset >> 8U : input.offset & byte_mask;
    const std::array<std::uint8_t, 3> code{static_cast<std::uint8_t>(input.form.opcode),
                                           static_cast<std::uint8_t>(first),
                                           static_cast<std::uint8_t>(input.offset)};
    for (unsigned n = 0; n < input.form.bytes; ++n) {
      const std::array<std::uint8_t, 1> byte{code[n]};
      memory.install(static_cast<std::uint16_t>(input.location + n), byte);
    }
    memory.install(vector, target);
    memory.accesses.clear();
    memory.retired = 0;
    cpu.set_state({.pc = entry,
                   .hx = index,
                   .sp = stack,
                   .a = accumulator,
                   .ccr_known = static_cast<std::uint8_t>(input.known),
                   .ccr = static_cast<std::uint8_t>(input.flags & ~flag_i),
                   .a_known = static_cast<std::uint8_t>(input.a_known),
                   .hx_known = static_cast<std::uint16_t>(input.hx_known)});
    memory.interrupt = {.vector = vector};
    require(cpu.start_irq_bus(Hcs08IrqBusContract::reference_unstretched_sequence),
            "Genuine source supplies the actual JMP frame, vector and three fetched bytes");
    for (unsigned n = 0; n < 11; ++n) { cpu.advance_irq_bus(); }
    cpu.start_irq_program(selected);
  }
};

void check_case(Machine &m, const Input input) {
  m.prepare(input);
  const auto before = m.cpu.state();
  const auto start = m.memory.accesses.size();
  for (unsigned phase = 1; phase <= input.form.periods; ++phase) {
    m.cpu.advance_irq_program();
    const bool executed = phase >= execution_phase(input);
    auto expected = before;
    if (executed) { expected.pc = destination(input); }
    const auto q = owned(m.cpu);
    const unsigned fetched = executed ? phase - execution_phase(input) + 1 : 1U;
    require(m.cpu.state() == expected && q.native_executed == executed &&
                q.completed_periods == phase && q.attempted_periods == phase &&
                q.retired_instructions == static_cast<unsigned>(phase == input.form.periods) &&
                m.memory.retired == q.retired_instructions && q.queued_bytes == fetched &&
                q.ready_bytes == fetched - 1 && q.program_byte_in_flight && !q.operand.value &&
                q.next_read == static_cast<std::uint16_t>(read_address(input, phase) + 1U) &&
                m.memory.accesses.size() == start + phase,
            "Independent JMP address/CCR/register/period oracle opcode=" + hex(input.form.opcode));
    for (unsigned n = 1; n <= phase; ++n) {
      const auto address = read_address(input, n);
      require(m.memory.accesses[start + n - 1] ==
                  Access{.address = address, .value = m.memory.bytes[address], .write = false},
              "Every JMP p period performs its actual sequential/selected-PC read exactly once");
    }
    if (executed) {
      require(q.head == expected.pc && q.discarded &&
                  q.discarded->address ==
                      static_cast<std::uint16_t>(input.location + input.form.bytes) &&
                  q.discarded->value == nop && !q.discarded->in_flight &&
                  q.jump_discarded.has_value() == (input.form.bytes == 1),
              "Actual already fetched successors remain discarded observations");
      if (q.jump_discarded) {
        require(q.jump_discarded->address == static_cast<std::uint16_t>(input.location + 2U) &&
                    q.jump_discarded->value == nop && !q.jump_discarded->in_flight,
                "JMP ,X retains both actual sequential successors");
      }
    }
  }
}

struct Snapshot {
  MemoryOwner memory;
  Hcs08Checkpoint cpu;
};

Snapshot capture(const Input input, const unsigned cut) {
  Machine original;
  original.prepare(input);
  for (unsigned n = 0; n < cut; ++n) { original.cpu.advance_irq_program(); }
  return {.memory = original.memory, .cpu = original.cpu.checkpoint()};
}

void restore(Machine &m, const Snapshot &saved) {
  m.memory = saved.memory;
  m.cpu.restore_checkpoint(saved.cpu);
}

void failed_read(const Input input,
                 const Snapshot &saved,
                 const Machine &baseline,
                 const unsigned cut,
                 const bool after) {
  Machine fault;
  restore(fault, saved);
  fault.memory.refused = read_address(input, cut + 1);
  fault.memory.after = after;
  const auto count = fault.memory.accesses.size();
  auto expected = fault.cpu.state();
  if (cut + 1 >= execution_phase(input)) { expected.pc = destination(input); }
  require(rejected([&] { fault.cpu.advance_irq_program(); }).contains("JMP-read refusal") &&
              fault.cpu.faulted() && fault.cpu.state() == expected &&
              owned(fault.cpu).completed_periods == cut &&
              fault.memory.accesses.size() == count + static_cast<unsigned>(after),
          "Failed JMP reads retain committed native PC and only actual access prefixes");
  require(rejected([&] { fault.cpu.advance_irq_program(); }).contains("coordinated recovery"),
          "JMP access faults cannot silently retry");
  restore(fault, saved);
  for (unsigned n = cut; n < input.form.periods; ++n) { fault.cpu.advance_irq_program(); }
  require(fault.cpu.prefetch_state() == baseline.cpu.prefetch_state() &&
              fault.memory.accesses == baseline.memory.accesses,
          "Coordinated JMP recovery performs each reference read once");
}

void continuations_and_failures() {
  for (const auto form : forms) {
    const Input input{.form = form, .offset = word_mask, .known = partial};
    Machine baseline;
    baseline.prepare(input);
    for (unsigned n = 0; n < form.periods; ++n) { baseline.cpu.advance_irq_program(); }
    for (unsigned cut = 0; cut <= form.periods; ++cut) {
      const auto saved = capture(input, cut);
      for (unsigned replay = 0; replay < 2; ++replay) {
        Machine fresh;
        restore(fresh, saved);
        for (unsigned n = cut; n < form.periods; ++n) { fresh.cpu.advance_irq_program(); }
        require(
            fresh.cpu.state() == baseline.cpu.state() &&
                fresh.cpu.prefetch_state() == baseline.cpu.prefetch_state() &&
                fresh.memory.bytes == baseline.memory.bytes &&
                fresh.memory.accesses == baseline.memory.accesses,
            "Fresh joint JMP owners retain native effects, successors and pending target reads");
      }
      if (cut == form.periods) { continue; }
      for (const bool after : {false, true}) { failed_read(input, saved, baseline, cut, after); }
    }
  }
}

void strict_and_captured() {
  Machine m;
  for (const auto form : forms) {
    const Input input{.form = form, .offset = word_mask};
    m.prepare(input, Hcs08PrefetchContract::reference_peripheral_program_cycles);
    const auto before = m.cpu.state();
    const auto accesses = m.memory.accesses;
    require(rejected([&] { m.cpu.advance_irq_program(); }).contains("unqualified") &&
                m.cpu.state() == before && m.memory.accesses == accesses,
            "The retained peripheral contract cannot silently acquire JMP timing");
    m.prepare(input);
    constexpr std::array<std::uint8_t, 3> changed{nop, 0, 0};
    m.memory.install(handler, changed);
    for (unsigned n = 0; n < form.periods; ++n) { m.cpu.advance_irq_program(); }
    require(m.cpu.state().pc == destination(input),
            "Native JMP consumes genuinely fetched opcode/operands despite changed backing code");
    if (!uses_index(input)) { continue; }
    for (const unsigned mask : {0U, byte_mask, 0xff00U, 0xfffeU}) {
      m.prepare({.form = form, .offset = word_mask, .hx_known = mask});
      const auto state = m.cpu.state();
      for (unsigned n = 1; n < execution_phase(input); ++n) { m.cpu.advance_irq_program(); }
      const auto count = m.memory.accesses.size();
      require(rejected([&] { m.cpu.advance_irq_program(); }).contains("partial-register") &&
                  m.cpu.state() == state && m.memory.accesses.size() == count,
              "Unknown H:X refuses native indexed JMP before a target read or invented PC");
    }
  }
  m.prepare({.form = forms[1], .location = word_mask - 1});
  const auto before = m.cpu.state();
  const auto accesses = m.memory.accesses;
  require(rejected([&] { m.cpu.advance_irq_program(); }).contains("wrap") &&
              m.cpu.state() == before && m.memory.accesses == accesses,
          "An unqualified wrapping instruction refuses before consuming captured code");
}
}

int main() {
  try {
    Machine m;
    for (const auto form : forms) {
      for (const auto offset : offsets) {
        for (unsigned flags = 0; flags < 32; ++flags) {
          const unsigned ccr =
              flags_fixed | (flags & 7U) | ((flags & 8U) << 1U) | ((flags & 16U) << 3U);
          for (const unsigned known : {byte_mask, partial}) {
            check_case(m, {.form = form, .offset = offset, .flags = ccr, .known = known});
          }
        }
      }
      check_case(m,
                 {.form = form,
                  .offset = word_mask,
                  .known = partial,
                  .a_known = 0,
                  .hx_known = uses_index({.form = form}) ? word_mask : 0U});
    }
    continuations_and_failures();
    strict_and_captured();
    std::cout
        << "HCS08 JMP:4160 independent cases,44 fresh suffixes,34 failed access prefixes pass.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
