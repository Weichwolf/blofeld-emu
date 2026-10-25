// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace blofeld::test::word {
using namespace blofeld::test::hcs08;
constexpr unsigned handler = 0x9000, vector = 0xffe0, stack = 0x240;
constexpr unsigned nop = 0x9d, direct = 0x35, extended = 0x96, prefix = 0x9e, stack_store = 0xff;
constexpr unsigned default_store_value = 0x1234;
constexpr unsigned changed = flag_v | flag_n | flag_z;
constexpr auto contract = Hcs08PrefetchContract::reference_scanner_word_cycles;
enum class Form : std::uint8_t { dir, ext, sp };
constexpr std::array forms{Form::dir, Form::ext, Form::sp};

struct Input {
  Form form;
  unsigned value = default_store_value, argument = operand, pc = handler, initial_stack = stack;
  unsigned flags = byte_mask, known = byte_mask, hx_known = word_mask;
};

constexpr unsigned periods(const Input input) {
  return input.form == Form::dir ? 4U : 5U;
}

constexpr unsigned first_write(const Input input) {
  return input.form == Form::dir ? 1U : 2U;
}

constexpr unsigned size(const Input input) {
  return input.form == Form::dir ? 2U : 3U;
}

constexpr unsigned target(const Input input) {
  return input.form == Form::sp
             ? static_cast<std::uint16_t>(input.initial_stack + space_bytes - 5U + input.argument)
             : input.argument;
}

struct Event {
  unsigned period;
  Access access;
  bool operator==(const Event &) const = default;
};

class Memory final : public MaskedMemory {
public:
  unsigned period = 0, native_starts = 0;
  std::optional<unsigned> refused;
  bool after = false;
  std::vector<Event> events;
  std::vector<unsigned> write_widths;

  std::uint8_t read(const std::uint16_t address) override {
    const auto value = hcs08::Memory::read(address);
    events.push_back(
        {.period = period, .access = {.address = address, .value = value, .write = false}});
    return value;
  }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    if (refused == address && !after) { fail(address, "own before-write failure"); }
    hcs08::Memory::write(address, value);
    events.push_back(
        {.period = period, .access = {.address = address, .value = value, .write = true}});
    if (refused == address && after) { fail(address, "own after-write failure"); }
  }

  void write_memory(const MemoryWrite access) override {
    write_widths.push_back(access.size);
    require(access.size == 1, "Each native word-store bus period submits one actual byte");
    Bus::write_memory(access);
  }

  void begin_execution(InstructionFetch /*fetch*/) override { ++native_starts; }
};

inline Hcs08PrefetchState owned(const Hcs08 &cpu) {
  const auto state = cpu.prefetch_state();
  if (!state) { throw Fault("Missing native word-store program owner"); }
  return *state;
}

struct Machine {
  Memory memory;
  Hcs08 cpu{memory, BLOFELD_HCS08_SLA};

  void prepare(const Input input, const Hcs08PrefetchContract selected = contract) {
    memory = Memory{};
    memory.known.set();
    memory.writable.set();
    memory.bytes.fill(nop);
    constexpr unsigned byte_bits = 8;
    std::array<std::uint8_t, 3> code{direct, static_cast<std::uint8_t>(input.argument), nop};
    if (input.form == Form::ext) {
      code = {extended,
              static_cast<std::uint8_t>(input.argument >> byte_bits),
              static_cast<std::uint8_t>(input.argument)};
    } else if (input.form == Form::sp) {
      code = {prefix, stack_store, static_cast<std::uint8_t>(input.argument)};
    }
    memory.install(input.pc, code);
    const std::array<std::uint8_t, 2> destination{static_cast<std::uint8_t>(input.pc >> byte_bits),
                                                  static_cast<std::uint8_t>(input.pc)};
    memory.install(vector, destination);
    cpu.set_state({.pc = entry,
                   .hx = static_cast<std::uint16_t>(input.value),
                   .sp = static_cast<std::uint16_t>(input.initial_stack),
                   .a = nop,
                   .ccr_known = static_cast<std::uint8_t>(input.known | flag_i),
                   .ccr = static_cast<std::uint8_t>(input.flags & ~flag_i),
                   .hx_known = static_cast<std::uint16_t>(input.hx_known)});
    memory.interrupt = {.vector = vector};
    require(cpu.start_irq_bus(Hcs08IrqBusContract::reference_unstretched_sequence),
            "Actual IRQ source owns the native frame/vector/program queue");
    for (unsigned n = 0; n < 11; ++n) { cpu.advance_irq_bus(); }
    cpu.start_irq_program(selected);
  }

  void tick() {
    memory.period = static_cast<unsigned>(owned(cpu).attempted_periods + 1);
    cpu.advance_irq_program();
  }
};

inline void finish(Machine &machine, const Input input) {
  while (owned(machine.cpu).completed_periods < periods(input)) { machine.tick(); }
}
}
