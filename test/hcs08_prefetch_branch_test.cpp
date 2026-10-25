// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
constexpr auto contract = Hcs08PrefetchContract::reference_short_branch_refill;
constexpr unsigned handler = 0x9000, vector = 0xffe0, stack = 0x240;
constexpr unsigned accumulator = 0x5c, index = 0x1234, nop = 0x9d;
constexpr unsigned bra = 0x20, brn = 0x21, bhi = 0x22, bls = 0x23;
constexpr unsigned bcc = 0x24, bcs = 0x25, bne = 0x26, beq = 0x27;
constexpr unsigned bhcc = 0x28, bhcs = 0x29, bpl = 0x2a, bmi = 0x2b;
constexpr unsigned bge = 0x90, blt = 0x91, bgt = 0x92, ble = 0x93;
constexpr unsigned partial = 0xef, unknown_zero = 0xfd;
constexpr std::array<unsigned, 16> opcodes{
    bra, brn, bhi, bls, bcc, bcs, bne, beq, bhcc, bhcs, bpl, bmi, bge, blt, bgt, ble};

struct Input {
  unsigned opcode;
  std::uint8_t flags;
  int offset;
  std::uint8_t known = byte_mask;
  unsigned location = handler;
};

bool condition(const Input input) {
  const bool c = (input.flags & flag_c) != 0;
  const bool z = (input.flags & flag_z) != 0;
  const bool n = (input.flags & flag_n) != 0;
  const bool h = (input.flags & flag_h) != 0;
  const bool v = (input.flags & flag_v) != 0;
  switch (input.opcode) {
    case bra: return true;
    case brn: return false;
    case bhi: return !c && !z;
    case bls: return c || z;
    case bcc: return !c;
    case bcs: return c;
    case bne: return !z;
    case beq: return z;
    case bhcc: return !h;
    case bhcs: return h;
    case bpl: return !n;
    case bmi: return n;
    case bge: return n == v;
    case blt: return n != v;
    case bgt: return !z && n == v;
    case ble: return z || n != v;
    default: throw Fault("Independent branch oracle needs an explicit condition");
  }
}

std::uint16_t destination(const Input input) {
  return static_cast<std::uint16_t>(static_cast<int>(input.location) + 2 +
                                    (condition(input) ? input.offset : 0));
}

Hcs08PrefetchState owned(const Hcs08 &cpu) {
  const auto q = cpu.prefetch_state();
  if (!q) { throw Fault("Expected owned short-branch queue"); }
  return *q;
}

struct Machine {
  MaskedMemory memory;
  Hcs08 cpu{memory, BLOFELD_HCS08_SLA};

  void prepare(const Input input) {
    const std::array<std::uint8_t, 2> target{static_cast<std::uint8_t>(input.location >> 8U),
                                             static_cast<std::uint8_t>(input.location)};
    constexpr std::array<std::uint8_t, 1> fill{nop};
    const auto taken =
        static_cast<std::uint16_t>(static_cast<int>(input.location) + 2 + input.offset);
    for (unsigned n = 0; n < 8; ++n) {
      memory.install(static_cast<std::uint16_t>(taken + n), fill);
      memory.install(static_cast<std::uint16_t>(input.location + 2 + n), fill);
      memory.install(static_cast<std::uint16_t>(input.location + n), fill);
    }
    const std::array<std::uint8_t, 3> code{
        static_cast<std::uint8_t>(input.opcode), static_cast<std::uint8_t>(input.offset), nop};
    memory.install(input.location, code);
    memory.install(vector, target);
    memory.writable.set();
    memory.accesses.clear();
    memory.retired = 0;
    cpu.set_state({.pc = entry,
                   .hx = index,
                   .sp = stack,
                   .a = accumulator,
                   .ccr_known = input.known,
                   .ccr = static_cast<std::uint8_t>(flags_fixed | (input.flags & ~flag_i))});
    memory.interrupt = {.vector = vector};
    require(cpu.start_irq_bus(Hcs08IrqBusContract::reference_unstretched_sequence),
            "A genuine pending source supplies the frame, vector and instruction queue");
    for (unsigned n = 0; n < 11; ++n) { cpu.advance_irq_bus(); }
    cpu.start_irq_program(contract);
  }
};

void check_case(Machine &machine, const Input input) {
  machine.prepare(input);
  const auto before = machine.cpu.state();
  auto expected = before;
  expected.pc = destination(input);
  const auto accesses = machine.memory.accesses.size();
  for (unsigned phase = 1; phase <= 3; ++phase) {
    machine.cpu.advance_irq_program();
    const auto q = owned(machine.cpu);
    if (!q.discarded) { throw Fault("Missing discarded actual sequential successor"); }
    require(machine.cpu.state() == expected && q.native_executed &&
                q.discarded->address == input.location + 2 && q.discarded->value == nop &&
                !q.discarded->in_flight && !q.operand.value && q.head == expected.pc &&
                q.next_read == static_cast<std::uint16_t>(expected.pc + phase) &&
                q.queued_bytes == phase && q.ready_bytes == phase - 1 && q.program_byte_in_flight &&
                q.completed_periods == phase && q.attempted_periods == phase &&
                q.retired_instructions == static_cast<unsigned>(phase == 3) &&
                machine.memory.retired == static_cast<unsigned>(phase == 3),
            "Independent native short-branch condition/PC and three-period refill opcode=" +
                hex(input.opcode) + " flags=" + hex(input.flags) +
                " offset=" + std::to_string(input.offset));
    require(machine.memory.accesses.size() == accesses + phase,
            "Only one actual program read belongs to each p period");
    for (unsigned n = 0; n < phase; ++n) {
      const auto address = static_cast<std::uint16_t>(expected.pc + n);
      require(machine.memory.accesses[accesses + n] ==
                      Access{.address = address,
                             .value = machine.memory.bytes[address],
                             .write = false} &&
                  q.queued[n] == machine.memory.bytes[address],
              "Actual selected-PC refill bytes, including sixteen-bit wrap, remain authoritative");
    }
  }
}

struct Snapshot {
  MaskedMemory memory;
  Hcs08Checkpoint cpu;
};

Snapshot capture(const Input input, const unsigned cut) {
  Machine original;
  original.prepare(input);
  for (unsigned n = 0; n < cut; ++n) { original.cpu.advance_irq_program(); }
  return {.memory = original.memory, .cpu = original.cpu.checkpoint()};
}

void fresh_continuations() {
  constexpr std::array<Input, 4> inputs{
      {{.opcode = beq, .flags = flag_z, .offset = -128, .known = partial},
       {.opcode = beq, .flags = flag_c, .offset = 127, .known = partial},
       {.opcode = ble, .flags = flag_v, .offset = -1},
       {.opcode = bge, .flags = flag_n, .offset = 1}}};
  for (const auto input : inputs) {
    Machine baseline;
    baseline.prepare(input);
    for (unsigned n = 0; n < 3; ++n) { baseline.cpu.advance_irq_program(); }
    for (unsigned cut = 0; cut <= 3; ++cut) {
      const auto saved = capture(input, cut);
      for (unsigned replay = 0; replay < 2; ++replay) {
        Machine fresh;
        fresh.memory = saved.memory;
        fresh.cpu.restore_checkpoint(saved.cpu);
        for (unsigned n = cut; n < 3; ++n) { fresh.cpu.advance_irq_program(); }
        require(
            fresh.cpu.state() == baseline.cpu.state() &&
                fresh.cpu.prefetch_state() == baseline.cpu.prefetch_state() &&
                fresh.memory.accesses == baseline.memory.accesses &&
                fresh.memory.bytes == baseline.memory.bytes,
            "Fresh joint owners preserve native decision, discarded byte and each pending refill");
      }
    }
  }
}

void changed_program() {
  constexpr Input input{.opcode = beq, .flags = flag_c, .offset = 127, .known = partial};
  Machine machine;
  machine.prepare(input);
  constexpr std::array<std::uint8_t, 3> changed{0x20, 1, 0x98}; // Own BRA/+1/CLC backing bytes.
  machine.memory.install(handler, changed);
  for (unsigned n = 0; n < 3; ++n) { machine.cpu.advance_irq_program(); }
  const auto q = owned(machine.cpu);
  if (!q.discarded) { throw Fault("Missing original captured successor"); }
  require(machine.cpu.state().pc == handler + 2 && q.discarded->value == nop &&
              q.queued[0] == changed[2],
          "Captured BEQ/offset decide execution; the new p1 observes changed successor code");
  machine.cpu.advance_irq_program();
  require(machine.cpu.state().pc == handler + 3 && (machine.cpu.state().ccr & flag_c) == 0,
          "Native CLC executes the actual replenished byte, never stale queued NOP or backing BRA");
}

void failed_reads() {
  constexpr Input input{.opcode = beq, .flags = flag_z, .offset = 127, .known = partial};
  for (unsigned phase = 1; phase <= 3; ++phase) {
    Machine machine;
    machine.prepare(input);
    for (unsigned n = 1; n < phase; ++n) { machine.cpu.advance_irq_program(); }
    const auto memory = machine.memory;
    const auto saved = machine.cpu.checkpoint();
    machine.memory.known.reset(destination(input) + phase - 1);
    const auto accesses = machine.memory.accesses;
    require(
        rejected([&] { machine.cpu.advance_irq_program(); }).contains("pc=0x9000") &&
            machine.cpu.faulted() && owned(machine.cpu).native_executed &&
            machine.cpu.state().pc == destination(input) && machine.memory.accesses == accesses &&
            owned(machine.cpu).completed_periods == phase - 1,
        "Failed refill keeps native PC and all committed prior read arrivals without retirement");
    machine.memory = memory;
    machine.cpu.restore_checkpoint(saved);
    for (unsigned n = phase; n <= 3; ++n) { machine.cpu.advance_irq_program(); }
    require(machine.memory.retired == 1 && owned(machine.cpu).retired_instructions == 1,
            "Coordinated CPU/memory recovery completes exactly one native branch retirement");
  }
}

void unknown_conditions() {
  constexpr std::array<Input, 3> inputs{
      {{.opcode = beq, .flags = flag_z, .offset = 1, .known = unknown_zero},
       {.opcode = bhcc, .flags = 0, .offset = 1, .known = partial},
       {.opcode = brn, .flags = 0, .offset = 1, .known = partial}}};
  for (const auto input : inputs) {
    Machine machine;
    machine.prepare(input);
    const auto before = machine.cpu.state();
    const auto accesses = machine.memory.accesses;
    require(rejected([&] { machine.cpu.advance_irq_program(); }).contains("partial-CCR") &&
                machine.cpu.faulted() && machine.cpu.state() == before &&
                machine.memory.accesses == accesses && !owned(machine.cpu).native_executed &&
                !owned(machine.cpu).discarded && owned(machine.cpu).queued_bytes == 1,
            "Unknown flag or unqualified partial semantics refuse before speculative reads or PC "
            "effects");
  }
}

void unsupported_forms() {
  constexpr std::array<unsigned, 6> unsupported{0x2c, 0x2d, 0x2e, 0x2f, 0xc7, 0x80};
  for (const auto opcode : unsupported) {
    Machine machine;
    machine.prepare({.opcode = opcode, .flags = 0, .offset = 0});
    const auto before = machine.cpu.state();
    const auto accesses = machine.memory.accesses;
    require(
        rejected([&] {
          machine.cpu.advance_irq_program();
        }).contains("unqualified program/data/flow") &&
            machine.cpu.faulted() && machine.cpu.state() == before &&
            machine.memory.accesses == accesses && owned(machine.cpu).queued_bytes == 3 &&
            owned(machine.cpu).ready_bytes == 3 && !owned(machine.cpu).discarded,
        "Mask/pin branches and unsupported write/return forms never invent inputs or consume code");
  }
}
}

int main() {
  try {
    Machine machine;
    constexpr std::array<int, 5> offsets{-128, -1, 0, 1, 127};
    for (const auto opcode : opcodes) {
      for (unsigned bits = 0; bits < 32; ++bits) {
        const auto flags =
            static_cast<std::uint8_t>((bits & 7U) | ((bits & 8U) << 1U) | ((bits & 16U) << 3U));
        for (const auto offset : offsets) {
          check_case(machine, {.opcode = opcode, .flags = flags, .offset = offset});
          if (opcode == bra || opcode == bhi || opcode == bls || opcode == bcs || opcode == bne ||
              opcode == beq) {
            check_case(machine,
                       {.opcode = opcode, .flags = flags, .offset = offset, .known = partial});
          }
        }
      }
    }
    constexpr std::array<Input, 2> wraps{
        {{.opcode = beq, .flags = flag_z, .offset = 127, .location = 0xfff0},
         {.opcode = beq, .flags = flag_z, .offset = -128, .location = 0x10}}};
    for (const auto input : wraps) { check_case(machine, input); }
    fresh_continuations();
    changed_program();
    failed_reads();
    unknown_conditions();
    unsupported_forms();
    std::cout << "HCS08 short-branch reference:3520 independent conditions/offsets,2 address wraps,"
                 "32 fresh continuations and strict failed refill/unknown flags pass.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
