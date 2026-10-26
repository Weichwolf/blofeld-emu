// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace blofeld::test::modify {
using namespace blofeld::test::hcs08;
constexpr unsigned handler = 0x9000, vector = 0xffe0, stack = 0x400, nop = 0x9d;
constexpr unsigned base_hx = 0x1234, vnz = flag_v | flag_n | flag_z;
constexpr auto contract = Hcs08PrefetchContract::reference_modify_program_cycles;

void inherited_stores();

enum class Operation : std::uint8_t { increment, decrement };

enum class Form : std::uint8_t { direct, ix, ix1, sp1 };

struct Spec {
  Form form;
  unsigned increment, decrement, bytes, periods, read, write;
};

// Independent literal ISA encodings/access periods, Family Rev2 printed249/252.
constexpr std::array specs{Spec{.form = Form::direct,
                                .increment = 0x3c,
                                .decrement = 0x3a,
                                .bytes = 2,
                                .periods = 5,
                                .read = 1,
                                .write = 3},
                           Spec{.form = Form::ix,
                                .increment = 0x7c,
                                .decrement = 0x7a,
                                .bytes = 1,
                                .periods = 4,
                                .read = 1,
                                .write = 3},
                           Spec{.form = Form::ix1,
                                .increment = 0x6c,
                                .decrement = 0x6a,
                                .bytes = 2,
                                .periods = 5,
                                .read = 1,
                                .write = 3},
                           Spec{.form = Form::sp1,
                                .increment = 0x6c,
                                .decrement = 0x6a,
                                .bytes = 3,
                                .periods = 6,
                                .read = 2,
                                .write = 4}};
constexpr std::array operations{Operation::increment, Operation::decrement};

struct Input {
  Spec spec = specs[0];
  Operation operation = Operation::increment;
  unsigned offset = operand, hx = base_hx, sp = stack, value = byte_mask;
  unsigned flags = byte_mask, known = byte_mask, a_known = byte_mask, hx_known = word_mask;
  unsigned memory_known = byte_mask;
};

constexpr unsigned target(const Input input) {
  switch (input.spec.form) {
    case Form::direct: return input.offset;
    case Form::ix: return input.hx;
    case Form::ix1: return (input.hx + input.offset) & word_mask;
    case Form::sp1: return (input.sp + input.offset) & word_mask;
    default: throw Fault("Invalid own INC/DEC test form");
  }
}

struct Event {
  unsigned period;
  Access access;
  bool operator==(const Event &) const = default;
};

class Memory final : public MaskedMemory {
public:
  unsigned period = 0, native_starts = 0;
  std::optional<unsigned> refused_read, refused_write;
  bool after = false;
  std::vector<Event> events;
  std::vector<unsigned> widths;

  std::uint8_t read(const std::uint16_t address) override {
    if (refused_read == address && !after) { fail(address, "before INC/DEC read"); }
    const auto value = hcs08::Memory::read(address);
    events.push_back(
        {.period = period, .access = {.address = address, .value = value, .write = false}});
    if (refused_read == address && after) { fail(address, "after INC/DEC read"); }
    return value;
  }

  Hcs08KnownByte read_known_byte(const std::uint16_t address) override {
    if (known[address] || !writable[address]) { return MaskedMemory::read_known_byte(address); }
    if (refused_read == address && !after) { fail(address, "before INC/DEC raw read"); }
    const auto byte = MaskedMemory::read_known_byte(address);
    events.push_back(
        {.period = period,
         .access = {.address = address, .value = byte.value, .write = false, .known = byte.known}});
    if (refused_read == address && after) { fail(address, "after INC/DEC raw read"); }
    return byte;
  }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    if (refused_write == address && !after) { fail(address, "before INC/DEC write"); }
    hcs08::Memory::write(address, value);
    events.push_back(
        {.period = period, .access = {.address = address, .value = value, .write = true}});
    if (refused_write == address && after) { fail(address, "after INC/DEC write"); }
  }

  void write_memory(const MemoryWrite access) override {
    widths.push_back(access.size);
    require(access.size == 1 && access.value <= byte_mask,
            "Native INC/DEC writes one actual result byte");
    Bus::write_memory(access);
  }

  void begin_execution(InstructionFetch /*fetch*/) override { ++native_starts; }
};

inline Hcs08PrefetchState owned(const Hcs08 &cpu) {
  const auto state = cpu.prefetch_state();
  if (!state) { throw Fault("Missing INC/DEC program owner"); }
  return *state;
}

struct Machine {
  Memory memory;
  Hcs08 cpu{memory, BLOFELD_HCS08_SLA};

  void prepare(const Input input, const Hcs08PrefetchContract selected = contract) {
    memory = Memory{};
    memory.bytes.fill(nop);
    memory.known.set();
    memory.writable.set();
    constexpr unsigned prefix = 0x9e;
    const auto opcode =
        input.operation == Operation::increment ? input.spec.increment : input.spec.decrement;
    const bool stack_relative = input.spec.form == Form::sp1;
    const std::array<std::uint8_t, 3> code{
        static_cast<std::uint8_t>(stack_relative ? prefix : opcode),
        static_cast<std::uint8_t>(stack_relative ? opcode : input.offset),
        static_cast<std::uint8_t>(input.offset)};
    memory.install(handler, std::span(code).first(input.spec.bytes));
    memory.bytes[target(input)] = static_cast<std::uint8_t>(input.value);
    const std::array<std::uint8_t, 2> destination{handler >> 8U, handler & byte_mask};
    memory.install(vector, destination);
    cpu.set_state({.pc = entry,
                   .hx = static_cast<std::uint16_t>(input.hx),
                   .sp = static_cast<std::uint16_t>(input.sp + 5),
                   .a = static_cast<std::uint8_t>(input.value),
                   .ccr_known = static_cast<std::uint8_t>(input.known | flag_i),
                   .ccr = static_cast<std::uint8_t>(input.flags & ~flag_i),
                   .a_known = static_cast<std::uint8_t>(input.a_known),
                   .hx_known = static_cast<std::uint16_t>(input.hx_known)});
    memory.interrupt = {.vector = vector};
    require(cpu.start_irq_bus(Hcs08IrqBusContract::reference_unstretched_sequence),
            "Own IRQ queue");
    for (unsigned period = 0; period < 11; ++period) { cpu.advance_irq_bus(); }
    cpu.start_irq_program(selected);
    memory.events.clear();
    memory.widths.clear();
    memory.native_starts = 0;
    memory.retired = 0;
    memory.bytes[target(input)] = static_cast<std::uint8_t>(input.value);
    memory.known[target(input)] = input.memory_known == byte_mask;
    memory.partial_known[target(input)] = static_cast<std::uint8_t>(input.memory_known);
  }

  void tick() {
    memory.period = static_cast<unsigned>(owned(cpu).attempted_periods + 1);
    cpu.advance_irq_program();
  }
};

constexpr unsigned result(const Input input) {
  return (input.value + (input.operation == Operation::increment ? 1U : byte_mask)) & byte_mask;
}

constexpr unsigned flags(const Input input) {
  constexpr unsigned sign = 0x80;
  constexpr unsigned positive_limit = 0x7f;
  const bool overflow =
      input.value == (input.operation == Operation::increment ? positive_limit : sign);
  return (overflow ? flag_v : 0U) | ((result(input) & sign) != 0 ? flag_n : 0U) |
         (result(input) == 0 ? flag_z : 0U);
}

inline void finish(Machine &m, const Input input) {
  while (owned(m.cpu).completed_periods < input.spec.periods) { m.tick(); }
}
}
