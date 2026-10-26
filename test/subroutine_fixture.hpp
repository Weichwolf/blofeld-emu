// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace blofeld::test::subroutine {
using namespace blofeld::test::hcs08;
constexpr unsigned handler = 0x9000, vector = 0xffe0, stack = 0x400, nop = 0x9d;
constexpr unsigned decrement_a = 0x4a, test_x = 0x5d, rts = 0x81, vnz = flag_v | flag_n | flag_z;
constexpr auto contract = Hcs08PrefetchContract::reference_subroutine_program_cycles;

struct Input {
  unsigned opcode = rts, value = 0x42, target = 0x91a5, sp = stack;
  unsigned flags = byte_mask, known = byte_mask, a_known = byte_mask, hx_known = word_mask;
};

struct Event {
  unsigned period;
  Access access;
  bool operator==(const Event &) const = default;
};

class Memory final : public MaskedMemory {
public:
  unsigned period = 0, native_starts = 0;
  std::optional<unsigned> refused_read;
  bool after = false;
  std::vector<Event> events;

  std::uint8_t read(const std::uint16_t address) override {
    if (refused_read == address && !after) { fail(address, "before return read"); }
    const auto value = hcs08::Memory::read(address);
    events.push_back(
        {.period = period, .access = {.address = address, .value = value, .write = false}});
    if (refused_read == address && after) { fail(address, "after return read"); }
    return value;
  }

  void begin_execution(InstructionFetch /*fetch*/) override { ++native_starts; }
};

inline Hcs08PrefetchState owned(const Hcs08 &cpu) {
  const auto state = cpu.prefetch_state();
  if (!state) { throw Fault("Missing subroutine program owner"); }
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

inline unsigned periods(const Input input) {
  return input.opcode == rts ? 6U : 1U;
}

inline void finish(Machine &machine, const Input input) {
  while (owned(machine.cpu).completed_periods < periods(input)) { machine.tick(); }
}
}
