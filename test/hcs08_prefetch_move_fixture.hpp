// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace blofeld::test::move {
using namespace blofeld::test::hcs08;
constexpr unsigned handler = 0x9000, vector = 0xffe0, stack = 0x240;
constexpr unsigned opcode = 0x4e, nop = 0x9d, periods = 5;
constexpr unsigned source_address = 0x20, destination_address = source_address + 1;
constexpr unsigned initial_hx = 0x1234;
constexpr unsigned changed = flag_v | flag_n | flag_z;
constexpr auto contract = Hcs08PrefetchContract::reference_scanner_move_cycles;

struct Input {
  unsigned source = source_address, destination = destination_address, value = 0x80, pc = handler;
  unsigned flags = byte_mask, known = byte_mask;
  unsigned a_known = byte_mask, hx_known = word_mask;
};

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
  std::vector<unsigned> read_widths, write_widths;

  std::uint8_t read(const std::uint16_t address) override {
    if (refused_read == address && !after) { fail(address, "own before-read failure"); }
    const auto value = hcs08::Memory::read(address);
    events.push_back(
        {.period = period, .access = {.address = address, .value = value, .write = false}});
    if (refused_read == address && after) { fail(address, "own after-read failure"); }
    return value;
  }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    if (refused_write == address && !after) { fail(address, "own before-write failure"); }
    hcs08::Memory::write(address, value);
    events.push_back(
        {.period = period, .access = {.address = address, .value = value, .write = true}});
    if (refused_write == address && after) { fail(address, "own after-write failure"); }
  }

  std::uint64_t read_memory(const MemoryRead access) override {
    read_widths.push_back(access.size);
    require(access.size == 1, "MOV data and program periods read actual single bytes");
    return Bus::read_memory(access);
  }

  void write_memory(const MemoryWrite access) override {
    write_widths.push_back(access.size);
    require(access.size == 1, "MOV write period submits one actual byte");
    Bus::write_memory(access);
  }

  void begin_execution(InstructionFetch /*fetch*/) override { ++native_starts; }
};

inline Hcs08PrefetchState owned(const Hcs08 &cpu) {
  const auto state = cpu.prefetch_state();
  if (!state) { throw Fault("Missing native MOV program owner"); }
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
    const std::array<std::uint8_t, 3> code{opcode,
                                           static_cast<std::uint8_t>(input.source),
                                           static_cast<std::uint8_t>(input.destination)};
    memory.install(input.pc, code);
    memory.install(input.source, std::array{static_cast<std::uint8_t>(input.value)});
    constexpr unsigned byte_bits = 8;
    const std::array<std::uint8_t, 2> destination{static_cast<std::uint8_t>(input.pc >> byte_bits),
                                                  static_cast<std::uint8_t>(input.pc)};
    memory.install(vector, destination);
    cpu.set_state({.pc = entry,
                   .hx = initial_hx,
                   .sp = stack,
                   .a = nop,
                   .ccr_known = static_cast<std::uint8_t>(input.known | flag_i),
                   .ccr = static_cast<std::uint8_t>(input.flags & ~flag_i),
                   .a_known = static_cast<std::uint8_t>(input.a_known),
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

inline void finish(Machine &machine) {
  while (owned(machine.cpu).completed_periods < periods) { machine.tick(); }
}
}
