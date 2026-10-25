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
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
constexpr unsigned handler = 0x9000, vector = 0xffe0, initial_stack = 0x240;
constexpr unsigned high_index = 0x1200, accumulator = 0x5a, partial = 0xef, vnzc = 0x87;
constexpr unsigned high_mask = 0xff00;
constexpr unsigned pattern = 0xa5;
constexpr unsigned nop = 0x9d, shift_x = 0x58, prefix = 0x9e, sta = 0xe7, stx = 0xef;
constexpr unsigned push_x = 0x89, clear_x = 0x5f, pull_x = 0x88;
constexpr auto contract = Hcs08PrefetchContract::reference_scanner_byte_cycles;
enum class Kind : std::uint8_t { shift, store_a, store_x, pull };
constexpr std::array<Kind, 4> kinds{Kind::shift, Kind::store_a, Kind::store_x, Kind::pull};

struct Input {
  Kind kind;
  unsigned data = 0, offset = 0x7f, flags = 0x75, known = byte_mask;
  unsigned stack = initial_stack, a_known = byte_mask, hx_known = word_mask;
};

unsigned periods(const Input input) {
  if (input.kind == Kind::shift) { return 1; }
  return input.kind == Kind::pull ? 6U : 4U;
}

bool store(const Input input) {
  return input.kind == Kind::store_a || input.kind == Kind::store_x;
}

unsigned target(const Input input) {
  const unsigned stack = static_cast<std::uint16_t>(input.stack + space_bytes - 5U);
  return input.kind == Kind::pull ? stack : static_cast<std::uint16_t>(stack + input.offset);
}

struct Event {
  unsigned period;
  Access access;
  bool operator==(const Event &) const = default;
};

class MemoryOwner final : public MaskedMemory {
public:
  unsigned period = 0;
  std::optional<unsigned> refused;
  bool after = false, refuse_write = false;
  std::vector<Event> events;

  std::uint8_t read(const std::uint16_t address) override {
    if (refused == address && !refuse_write && !after) { fail(address, "own before-read refusal"); }
    const auto value = Memory::read(address);
    events.push_back(
        {.period = period, .access = {.address = address, .value = value, .write = false}});
    if (refused == address && !refuse_write && after) { fail(address, "own after-read refusal"); }
    return value;
  }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    if (refused == address && refuse_write && !after) { fail(address, "own before-write refusal"); }
    Memory::write(address, value);
    events.push_back(
        {.period = period, .access = {.address = address, .value = value, .write = true}});
    if (refused == address && refuse_write && after) { fail(address, "own after-write refusal"); }
  }
};

Hcs08PrefetchState owned(const Hcs08 &cpu) {
  const auto q = cpu.prefetch_state();
  if (!q) { throw Fault("Missing native scanner byte queue"); }
  return *q;
}

struct Machine {
  MemoryOwner memory;
  Hcs08 cpu{memory, BLOFELD_HCS08_SLA};

  void prepare(const Input input, const Hcs08PrefetchContract selected = contract) {
    memory.refused.reset();
    memory.period = 0;
    memory.known.set();
    memory.writable.set();
    memory.bytes.fill(nop);
    memory.partial_known.fill(0);
    memory.events.clear();
    memory.accesses.clear();
    memory.retired = 0;
    constexpr std::array<std::uint8_t, 2> irq_target{0x90, 0};
    std::array<std::uint8_t, 3> code{shift_x, nop, nop};
    if (store(input)) {
      code = {prefix,
              static_cast<std::uint8_t>(input.kind == Kind::store_a ? sta : stx),
              static_cast<std::uint8_t>(input.offset)};
    }
    if (input.kind == Kind::pull) { code = {push_x, clear_x, pull_x}; }
    memory.install(handler, code);
    memory.install(vector, irq_target);
    const unsigned a = input.kind == Kind::store_a ? input.data : accumulator;
    cpu.set_state({.pc = entry,
                   .hx = static_cast<std::uint16_t>(high_index | input.data),
                   .sp = static_cast<std::uint16_t>(input.stack),
                   .a = static_cast<std::uint8_t>(a),
                   .ccr_known = static_cast<std::uint8_t>(input.known),
                   .ccr = static_cast<std::uint8_t>(input.flags & ~flag_i),
                   .a_known = static_cast<std::uint8_t>(input.a_known),
                   .hx_known = static_cast<std::uint16_t>(input.hx_known)});
    memory.interrupt = {.vector = vector};
    require(cpu.start_irq_bus(Hcs08IrqBusContract::reference_unstretched_sequence),
            "Actual source supplies the native frame, vector and owned program bytes");
    for (unsigned n = 0; n < 11; ++n) { cpu.advance_irq_bus(); }
    cpu.start_irq_program(selected);
  }

  void tick() {
    memory.period = static_cast<unsigned>(owned(cpu).attempted_periods + 1);
    cpu.advance_irq_program();
  }
};

unsigned stored_flags(const Hcs08State before, const unsigned value) {
  const unsigned retained = before.ccr & ~static_cast<unsigned>(flag_v | flag_n | flag_z);
  return retained | (value == 0 ? flag_z : 0U) | ((value & flag_v) != 0 ? flag_n : 0U);
}

Hcs08State expected_state(const Hcs08State before, const Input input, const unsigned period) {
  auto result = before;
  if (input.kind == Kind::shift) {
    const unsigned value = (input.data * 2U) & byte_mask;
    result.pc = handler + 1;
    result.hx = static_cast<std::uint16_t>((before.hx & high_mask) | value);
    const unsigned carry = input.data >> 7U;
    const unsigned negative = (value >> 7U) & 1U;
    result.ccr = static_cast<std::uint8_t>(
        (before.ccr & ~vnzc) | carry | (value == 0 ? flag_z : 0U) | (negative != 0 ? flag_n : 0U) |
        ((negative ^ carry) != 0 ? flag_v : 0U));
    result.ccr_known |= vnzc;
  } else if (store(input) && period >= 2) {
    result.pc = handler + 3;
    result.ccr = static_cast<std::uint8_t>(stored_flags(before, input.data));
    result.ccr_known |= flag_v | flag_n | flag_z;
  } else if (input.kind == Kind::pull) {
    if (period < 3) {
      result.pc = handler + 1;
      --result.sp;
    } else {
      result.pc = handler + 2;
      --result.sp;
      result.hx &= high_mask;
      result.ccr = static_cast<std::uint8_t>(stored_flags(before, 0));
      result.ccr_known |= flag_v | flag_n | flag_z;
      if (period >= 5) {
        result.pc = handler + 3;
        ++result.sp;
        result.hx |= static_cast<std::uint16_t>(input.data);
      }
    }
  }
  return result;
}

void check_case(Machine &m, const Input input) {
  m.prepare(input);
  const auto before = m.cpu.state();
  const auto start = m.memory.events.size();
  const auto prior_data = m.memory.bytes[target(input)];
  for (unsigned n = 1; n <= periods(input); ++n) {
    m.tick();
    const auto q = owned(m.cpu);
    require(m.cpu.state() == expected_state(before, input, n) && q.completed_periods == n &&
                q.attempted_periods == n,
            "Independent scanner byte register/flag/stack and period oracle");
    if (store(input)) {
      const auto writes = static_cast<unsigned>(n >= 2);
      require(m.memory.events.size() == start + n &&
                  m.memory.bytes[target(input)] == (writes != 0 ? input.data : prior_data),
              "SP1 store performs one actual byte transaction at w2");
      if (n >= 2) {
        require(m.memory.events[start + 1] ==
                    Event{.period = 2,
                          .access = {.address = target(input), .value = input.data, .write = true}},
                "Actual native SP+unsigned-offset write and value match the ISA oracle");
      }
    }
  }
  const auto q = owned(m.cpu);
  const unsigned count = input.kind == Kind::pull ? 3U : 1U;
  require(q.instruction_bytes == 0 && q.retired_instructions == count &&
              m.memory.retired == count && q.queued_bytes == 3 && q.ready_bytes == 2 &&
              q.program_byte_in_flight && q.head == before.pc + (input.kind == Kind::shift ? 1 : 3),
          "Every completed native instruction leaves its genuine three-byte successor queue");
  if (input.kind == Kind::pull) {
    require(
        q.operand.address == target(input) && q.operand.value == input.data && q.operand.consumed &&
            !q.operand.in_flight &&
            m.memory.events[start] ==
                Event{.period = 1,
                      .access = {.address = target(input), .value = input.data, .write = true}} &&
            m.memory.events[start + 3] ==
                Event{.period = 4,
                      .access = {.address = target(input), .value = input.data, .write = false}},
        "PULX consumes the actual native PSHX byte once at its u/f phases");
  }
}

struct Snapshot {
  MemoryOwner memory;
  Hcs08Checkpoint cpu;
};

Snapshot capture(const Input input, const unsigned cut) {
  Machine original;
  original.prepare(input);
  for (unsigned n = 0; n < cut; ++n) { original.tick(); }
  return {.memory = original.memory, .cpu = original.cpu.checkpoint()};
}

void restore(Machine &m, const Snapshot &s) {
  m.memory = s.memory;
  m.cpu.restore_checkpoint(s.cpu);
}

void fresh_suffixes() {
  for (const auto kind : kinds) {
    const Input input{.kind = kind, .data = pattern, .known = partial};
    Machine baseline;
    baseline.prepare(input);
    for (unsigned n = 0; n < periods(input); ++n) { baseline.tick(); }
    for (unsigned cut = 0; cut <= periods(input); ++cut) {
      const auto saved = capture(input, cut);
      for (unsigned replay = 0; replay < 2; ++replay) {
        Machine fresh;
        restore(fresh, saved);
        for (unsigned n = cut; n < periods(input); ++n) { fresh.tick(); }
        require(fresh.cpu.state() == baseline.cpu.state() &&
                    fresh.cpu.prefetch_state() == baseline.cpu.prefetch_state() &&
                    fresh.memory.bytes == baseline.memory.bytes &&
                    fresh.memory.events == baseline.memory.events,
                "Fresh joint owners preserve actual scanner byte effects, read arrival and program "
                "queue");
      }
    }
  }
}

void failed_prefixes() {
  for (const auto kind : kinds) {
    const Input input{.kind = kind, .data = pattern};
    const unsigned cut = kind == Kind::pull ? 3U : static_cast<unsigned>(store(input));
    const auto saved = capture(input, cut);
    Machine baseline;
    restore(baseline, saved);
    for (unsigned n = cut; n < periods(input); ++n) { baseline.tick(); }
    for (const bool after : {false, true}) {
      Machine fault;
      restore(fault, saved);
      fault.memory.refused = kind == Kind::shift ? handler + 3 : target(input);
      fault.memory.after = after;
      fault.memory.refuse_write = store(input);
      const auto events = fault.memory.events.size();
      require(rejected([&] { fault.tick(); }).contains("refusal") && fault.cpu.faulted() &&
                  owned(fault.cpu).completed_periods == cut &&
                  fault.memory.events.size() == events + static_cast<unsigned>(after),
              "Scanner read/write failures retain exactly their committed access prefix");
      require(rejected([&] { fault.tick(); }).contains("coordinated recovery"),
              "Failed native byte phases cannot silently retry");
      restore(fault, saved);
      for (unsigned n = cut; n < periods(input); ++n) { fault.tick(); }
      require(fault.cpu.state() == baseline.cpu.state() &&
                  fault.cpu.prefetch_state() == baseline.cpu.prefetch_state() &&
                  fault.memory.bytes == baseline.memory.bytes &&
                  fault.memory.events == baseline.memory.events,
              "Joint recovery executes every genuine byte transaction exactly once");
    }
  }
}

void strict_inputs() {
  Machine m;
  for (const auto kind : kinds) {
    const Input input{.kind = kind, .data = pattern};
    m.prepare(input, Hcs08PrefetchContract::reference_jump_program_cycles);
    if (kind == Kind::pull) {
      for (unsigned n = 0; n < 3; ++n) { m.tick(); }
    }
    const auto before = m.cpu.state();
    const auto accesses = m.memory.accesses;
    require(rejected([&] { m.tick(); }).contains("unqualified") && m.cpu.state() == before &&
                m.memory.accesses == accesses,
            "The retained JMP contract cannot silently acquire scanner byte cycles");
    m.prepare(input);
    constexpr std::array<std::uint8_t, 3> changed{nop, nop, nop};
    m.memory.install(handler, changed);
    const auto original = m.cpu.state();
    for (unsigned n = 0; n < periods(input); ++n) { m.tick(); }
    require(m.cpu.state() == expected_state(original, input, periods(input)),
            "Native scanner execution consumes captured bytes despite changed backing code");
  }
  for (const auto kind : {Kind::shift, Kind::store_a, Kind::store_x}) {
    const Input input{.kind = kind,
                      .data = pattern,
                      .a_known = kind == Kind::store_a ? 0U : byte_mask,
                      .hx_known = kind == Kind::store_a ? word_mask : 0xff00U};
    m.prepare(input);
    if (store(input)) { m.tick(); }
    require(rejected([&] { m.tick(); }).contains("partial-register") && m.cpu.faulted(),
            "Unknown consumed A/X bits refuse native scanner effects without invented data");
  }
  const Input pull{.kind = Kind::pull, .data = pattern};
  m.prepare(pull);
  for (unsigned n = 0; n < 3; ++n) { m.tick(); }
  const auto state = m.cpu.state();
  m.memory.known.reset(target(pull));
  require(rejected([&] { m.tick(); }).contains("read address=") && m.cpu.state() == state &&
              m.cpu.faulted(),
          "Unknown real stack data refuses PULX at u1 without inventing X or advancing SP");
}
}

int main() {
  try {
    Machine m;
    for (unsigned value = 0; value < byte_values; ++value) {
      for (const unsigned known : {byte_mask, partial}) {
        for (unsigned bits = 0; bits < 32; ++bits) {
          const unsigned flags =
              flags_fixed | (bits & 7U) | ((bits & 8U) << 1U) | ((bits & 16U) << 3U);
          check_case(m, {.kind = Kind::shift, .data = value, .flags = flags, .known = known});
        }
        check_case(m, {.kind = Kind::pull, .data = value, .known = known});
        for (const auto kind : {Kind::store_a, Kind::store_x}) {
          for (const unsigned offset : {0U, 1U, 0x7fU, byte_mask}) {
            check_case(m, {.kind = kind, .data = value, .offset = offset, .known = known});
          }
        }
      }
    }
    for (const auto kind : kinds) {
      if (kind == Kind::store_a) { continue; }
      check_case(
          m,
          {.kind = kind, .data = pattern, .known = partial, .a_known = 0, .hx_known = byte_mask});
    }
    for (const auto kind : {Kind::store_a, Kind::store_x}) {
      check_case(m, {.kind = kind, .data = pattern, .offset = byte_mask, .stack = 3});
    }
    fresh_suffixes();
    failed_prefixes();
    strict_inputs();
    std::cout << "HCS08 scanner bytes:20992 independent cases and38 fresh suffixes pass.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
