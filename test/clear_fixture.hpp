// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace blofeld::test::clear {
using namespace blofeld::test::hcs08;
constexpr unsigned handler = 0x9000, vector = 0xffe0, stack = 0x400, nop = 0x9d;
constexpr unsigned base_hx = 0x1234, vnz = flag_v | flag_n | flag_z;
constexpr auto contract = Hcs08PrefetchContract::reference_clear_program_cycles;

void inherited_stores();

enum class Form : std::uint8_t { direct, ix, ix1, sp1 };

struct Spec {
  Form form;
  unsigned opcode, bytes, periods, read, write;
};

// Independent literal access periods from Family Rev2 printed241.
constexpr std::array specs{
    Spec{.form = Form::direct, .opcode = 0x3f, .bytes = 2, .periods = 5, .read = 1, .write = 3},
    Spec{.form = Form::ix, .opcode = 0x7f, .bytes = 1, .periods = 4, .read = 1, .write = 3},
    Spec{.form = Form::ix1, .opcode = 0x6f, .bytes = 2, .periods = 5, .read = 1, .write = 3},
    Spec{.form = Form::sp1, .opcode = 0x9e, .bytes = 3, .periods = 6, .read = 2, .write = 4}};

struct Input {
  Spec spec = specs[0];
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
    default: throw Fault("Invalid own CLR test form");
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
    if (refused_read == address && !after) { fail(address, "before CLR read"); }
    const auto value = hcs08::Memory::read(address);
    events.push_back(
        {.period = period, .access = {.address = address, .value = value, .write = false}});
    if (refused_read == address && after) { fail(address, "after CLR read"); }
    return value;
  }

  Hcs08KnownByte read_known_byte(const std::uint16_t address) override {
    if (known[address] || !writable[address]) { return MaskedMemory::read_known_byte(address); }
    if (refused_read == address && !after) { fail(address, "before CLR raw read"); }
    const auto byte = MaskedMemory::read_known_byte(address);
    events.push_back(
        {.period = period,
         .access = {.address = address, .value = byte.value, .write = false, .known = byte.known}});
    if (refused_read == address && after) { fail(address, "after CLR raw read"); }
    return byte;
  }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    if (refused_write == address && !after) { fail(address, "before CLR write"); }
    hcs08::Memory::write(address, value);
    events.push_back(
        {.period = period, .access = {.address = address, .value = value, .write = true}});
    if (refused_write == address && after) { fail(address, "after CLR write"); }
  }

  void write_memory(const MemoryWrite access) override {
    widths.push_back(access.size);
    require(access.size == 1 && access.value == 0, "Native CLR writes one actual zero byte");
    Bus::write_memory(access);
  }

  void begin_execution(InstructionFetch /*fetch*/) override { ++native_starts; }
};

inline Hcs08PrefetchState owned(const Hcs08 &cpu) {
  const auto state = cpu.prefetch_state();
  if (!state) { throw Fault("Missing CLR program owner"); }
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
    const std::array<std::uint8_t, 3> code{
        static_cast<std::uint8_t>(input.spec.opcode),
        static_cast<std::uint8_t>(input.spec.form == Form::sp1 ? specs[2].opcode : input.offset),
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

inline void finish(Machine &m, const Input input) {
  while (owned(m.cpu).completed_periods < input.spec.periods) { m.tick(); }
}
}
