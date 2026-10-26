// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace blofeld::test::indexed {
using namespace blofeld::test::hcs08;
constexpr unsigned handler = 0x9000, vector = 0xffe0, stack = 0x400;
constexpr unsigned nop = 0x9d, initial_hx = 0x1234;
constexpr unsigned sign_bit = 0x80;
constexpr unsigned vnz = flag_v | flag_n | flag_z;
constexpr auto contract = Hcs08PrefetchContract::reference_indexed_store_cycles;

enum class Form : std::uint8_t { ix, ix1, ix2 };
constexpr std::array forms{Form::ix, Form::ix1, Form::ix2};

struct Input {
  Form form = Form::ix;
  unsigned base = initial_hx, offset = 0, value = 0, pc = handler;
  unsigned flags = byte_mask, known = byte_mask;
  unsigned a_known = byte_mask, hx_known = word_mask;
};

constexpr unsigned size(const Form form) {
  return 1U + static_cast<unsigned>(form);
}

constexpr unsigned periods(const Form form) {
  return size(form) + 1;
}

constexpr unsigned write_phase(const Form form) {
  return form == Form::ix2 ? 2U : 1U;
}

constexpr unsigned target(const Input input) {
  return (input.base + input.offset) & word_mask;
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
  std::vector<unsigned> write_widths;

  std::uint8_t read(const std::uint16_t address) override {
    if (refused_read == address && !after) { fail(address, "before indexed program read"); }
    const auto value = hcs08::Memory::read(address);
    events.push_back(
        {.period = period, .access = {.address = address, .value = value, .write = false}});
    if (refused_read == address && after) { fail(address, "after indexed program read"); }
    return value;
  }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    if (refused_write == address && !after) { fail(address, "before indexed write"); }
    hcs08::Memory::write(address, value);
    events.push_back(
        {.period = period, .access = {.address = address, .value = value, .write = true}});
    if (refused_write == address && after) { fail(address, "after indexed write"); }
  }

  void write_memory(const MemoryWrite access) override {
    write_widths.push_back(access.size);
    require(access.size == 1, "Indexed STA issues a genuine one-byte write");
    Bus::write_memory(access);
  }

  void begin_execution(InstructionFetch /*fetch*/) override { ++native_starts; }
};

inline Hcs08PrefetchState owned(const Hcs08 &cpu) {
  const auto state = cpu.prefetch_state();
  if (!state) { throw Fault("Missing indexed program owner"); }
  return *state;
}

struct Machine {
  Memory memory;
  Hcs08 cpu{memory, BLOFELD_HCS08_SLA};

  void prepare(const Input input,
               const bool irq = true,
               const Hcs08PrefetchContract selected = contract) {
    memory = Memory{};
    memory.known.set();
    memory.writable.set();
    memory.bytes.fill(nop);
    constexpr unsigned sta_ix = 0xf7;
    constexpr unsigned sta_ix1 = 0xe7;
    constexpr unsigned sta_ix2 = 0xd7;
    constexpr unsigned byte_bits = 8;
    std::array<std::uint8_t, 3> code{sta_ix, nop, nop};
    if (input.form == Form::ix1) {
      code = {sta_ix1, static_cast<std::uint8_t>(input.offset), nop};
    } else if (input.form == Form::ix2) {
      code = {sta_ix2,
              static_cast<std::uint8_t>(input.offset >> byte_bits),
              static_cast<std::uint8_t>(input.offset)};
    }
    for (unsigned n = 0; n < code.size(); ++n) {
      memory.install((input.pc + n) & word_mask, std::span(&code[n], 1));
    }
    const std::array<std::uint8_t, 2> destination{static_cast<std::uint8_t>(input.pc >> byte_bits),
                                                  static_cast<std::uint8_t>(input.pc)};
    memory.install(vector, destination);
    cpu.set_state({.pc = static_cast<std::uint16_t>(irq ? entry : input.pc),
                   .hx = static_cast<std::uint16_t>(input.base),
                   .sp = stack,
                   .a = static_cast<std::uint8_t>(input.value),
                   .ccr_known = static_cast<std::uint8_t>(input.known | (irq ? flag_i : 0U)),
                   .ccr = static_cast<std::uint8_t>(irq ? input.flags & ~flag_i : input.flags),
                   .a_known = static_cast<std::uint8_t>(input.a_known),
                   .hx_known = static_cast<std::uint16_t>(input.hx_known)});
    if (irq) {
      memory.interrupt = {.vector = vector};
      require(cpu.start_irq_bus(Hcs08IrqBusContract::reference_unstretched_sequence),
              "Actual IRQ frame/vector fetch owns the native program queue");
      for (unsigned period = 0; period < 11; ++period) { cpu.advance_irq_bus(); }
      cpu.start_irq_program(selected);
    }
    memory.events.clear();
    memory.write_widths.clear();
    memory.native_starts = 0;
    memory.retired = 0;
  }

  void tick() {
    memory.period = static_cast<unsigned>(owned(cpu).attempted_periods + 1);
    cpu.advance_irq_program();
  }
};

inline Hcs08State expected(const Hcs08State before, const Input input, const unsigned phase) {
  auto result = before;
  if (phase >= write_phase(input.form)) {
    result.pc = static_cast<std::uint16_t>(input.pc + size(input.form));
    result.ccr = static_cast<std::uint8_t>((before.ccr & ~vnz) |
                                           ((input.value & sign_bit) != 0 ? flag_n : 0U) |
                                           (input.value == 0 ? flag_z : 0U));
    result.ccr_known |= vnz;
  }
  return result;
}

inline void finish(Machine &machine, const Input input) {
  while (owned(machine.cpu).completed_periods < periods(input.form)) { machine.tick(); }
}
}
