// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace blofeld::test::call {
using namespace blofeld::test::hcs08;
constexpr unsigned handler = 0x9000, vector = 0xffe0, stack = 0x400, nop = 0x9d;
constexpr unsigned first_indexed_call = 0xdd;
constexpr auto contract = Hcs08PrefetchContract::reference_call_program_cycles;

struct Spec {
  unsigned opcode, bytes, first, periods;
};

// Literal Family Rev2 p237/254: s writes PCL, then PCH; EXT/IX2 start with p.
constexpr std::array specs{Spec{.opcode = 0xad, .bytes = 2, .first = 1, .periods = 5},
                           Spec{.opcode = 0xbd, .bytes = 2, .first = 1, .periods = 5},
                           Spec{.opcode = 0xcd, .bytes = 3, .first = 2, .periods = 6},
                           Spec{.opcode = 0xdd, .bytes = 3, .first = 2, .periods = 6},
                           Spec{.opcode = 0xed, .bytes = 2, .first = 1, .periods = 5},
                           Spec{.opcode = 0xfd, .bytes = 1, .first = 1, .periods = 5}};

struct Input {
  Spec spec = specs[0];
  unsigned offset = 0x42, hx = 0x1234, sp = stack;
  unsigned flags = byte_mask, known = byte_mask, a_known = byte_mask, hx_known = word_mask;
};

constexpr unsigned target(const Input input) {
  constexpr unsigned bsr = 0xad;
  constexpr unsigned direct = 0xbd;
  constexpr unsigned extended = 0xcd;
  constexpr unsigned ix = 0xfd;
  constexpr unsigned sign_extension = 0xff00;
  if (input.spec.opcode == bsr) {
    const unsigned displacement = input.offset & byte_mask;
    return (handler + 2 + displacement + (displacement < 128 ? 0U : sign_extension)) & word_mask;
  }
  if (input.spec.opcode == direct) { return input.offset & byte_mask; }
  if (input.spec.opcode == extended) { return input.offset; }
  return (input.hx + (input.spec.opcode == ix ? 0U : input.offset)) & word_mask;
}

struct Event {
  unsigned period;
  Access access;
  bool operator==(const Event &) const = default;
};

class Memory final : public MaskedMemory {
public:
  unsigned period = 0, native_starts = 0;
  std::optional<unsigned> refused_write;
  bool after = false;
  std::vector<Event> events;

  std::uint8_t read(const std::uint16_t address) override {
    const auto value = hcs08::Memory::read(address);
    events.push_back(
        {.period = period, .access = {.address = address, .value = value, .write = false}});
    return value;
  }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    if (refused_write == address && !after) { fail(address, "before call write"); }
    hcs08::Memory::write(address, value);
    events.push_back(
        {.period = period, .access = {.address = address, .value = value, .write = true}});
    if (refused_write == address && after) { fail(address, "after call write"); }
  }

  void write_memory(const MemoryWrite access) override {
    require(access.size == 1 && access.value <= byte_mask, "Call transfers actual native bytes");
    Bus::write_memory(access);
  }

  void begin_execution(InstructionFetch /*fetch*/) override { ++native_starts; }
};

inline Hcs08PrefetchState owned(const Hcs08 &cpu) {
  const auto state = cpu.prefetch_state();
  if (!state) { throw Fault("Missing call program owner"); }
  return *state;
}

struct Machine {
  Memory memory;
  Hcs08 cpu{memory, BLOFELD_HCS08_SLA};

  void prepare(Input input, Hcs08PrefetchContract selected = contract);

  void tick() {
    memory.period = static_cast<unsigned>(owned(cpu).attempted_periods + 1);
    cpu.advance_irq_program();
  }
};

inline void finish(Machine &machine, const Input input) {
  while (owned(machine.cpu).completed_periods < input.spec.periods) { machine.tick(); }
}
}
