// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
constexpr auto contract = Hcs08PrefetchContract::reference_peripheral_program_cycles;
constexpr unsigned handler = 0x9000, vector = 0xffe0, stack = 0x240, saved_h = 0x23b;
constexpr unsigned sta_direct = 0xb7, sta_extended = 0xc7, mov = 0x6e;
constexpr unsigned indexed8 = 0xe6, indexed16 = 0xd6, pull = 0x8a;
constexpr unsigned accumulator = 0x5c, index = 0xfff0, ordinary_index = 0x1234;
constexpr unsigned extended_address = 0x1234, indexed8_address = 0x6f, indexed16_address = 0x1224;
constexpr unsigned low_index = 0x34, partial = 0xef, nop = 0x9d;
constexpr unsigned push_h = 0x8b, clear_h = 0x8c, index_offset = 0x7f, move_pattern = 0xa5;
constexpr std::array<std::uint8_t, 8> nops{nop, nop, nop, nop, nop, nop, nop, nop};

struct Input {
  unsigned opcode, data;
  std::uint8_t known = byte_mask;
  std::uint16_t hx_known = word_mask;
  std::uint8_t a_known = byte_mask;
  bool self_modify = false;
};

bool writes(const Input input) {
  return input.opcode == sta_direct || input.opcode == sta_extended || input.opcode == mov;
}

unsigned periods(const Input input) {
  return input.opcode == sta_extended || input.opcode == mov || input.opcode == indexed16 ? 4U : 3U;
}

unsigned bytes(const Input input) {
  if (input.opcode == pull) { return 1; }
  return periods(input) == 4 ? 3U : 2U;
}

unsigned data_phase(const Input input) {
  return periods(input) == 4 ? 2U : 1U;
}

unsigned address(const Input input) {
  if (input.self_modify) { return handler + 3; }
  switch (input.opcode) {
    case indexed8: return indexed8_address;
    case indexed16: return indexed16_address;
    case sta_extended: return extended_address;
    case pull: return saved_h;
    default: return operand;
  }
}

unsigned initial_hx(const Input input) {
  if (input.opcode == pull) { return (input.data << 8U) | low_index; }
  if (input.opcode == indexed8 || input.opcode == indexed16) { return index; }
  return ordinary_index;
}

Hcs08PrefetchState owned(const Hcs08 &cpu) {
  const auto q = cpu.prefetch_state();
  if (!q) { throw Fault("Expected owned peripheral instruction queue"); }
  return *q;
}

class MemoryOwner final : public MaskedMemory {
public:
  unsigned watch = operand, reads = 0, writes = 0;
  bool change_after_read = false, fail_before_write = false, fail_after_write = false;

  std::uint8_t read(const std::uint16_t location) override {
    const auto value = Memory::read(location);
    if (location == watch) {
      ++reads;
      if (change_after_read) {
        bytes[location] = static_cast<std::uint8_t>(value ^ byte_mask);
        ++generation; // Own read-to-change fixture, not a physical register claim.
      }
    }
    return value;
  }

  void write(const std::uint16_t location, const std::uint8_t value) override {
    if (location == watch && fail_before_write) { throw Fault("Own refusal before actual write"); }
    Memory::write(location, value);
    if (location == watch) {
      ++writes;
      if (fail_after_write) { throw Fault("Own refusal after actual committed write"); }
    }
  }
};

std::array<std::uint8_t, 3> instruction(const Input input) {
  if (input.opcode == pull) { return {push_h, clear_h, pull}; } // Actual PSHH; CLRH; PULH.
  if (input.opcode == mov) { return {mov, static_cast<std::uint8_t>(input.data), operand}; }
  if (input.opcode == indexed8) { return {indexed8, index_offset, nop}; }
  const unsigned target = input.opcode == indexed16 ? extended_address : address(input);
  return {static_cast<std::uint8_t>(input.opcode),
          static_cast<std::uint8_t>(bytes(input) == 3 ? target >> 8U : target),
          static_cast<std::uint8_t>(target)};
}

struct Machine {
  MemoryOwner memory;
  Hcs08 cpu{memory, BLOFELD_HCS08_SLA};

  void prepare(const Input input) {
    constexpr std::array<std::uint8_t, 2> target{0x90, 0};
    constexpr std::array<unsigned, 4> flags{0x61, 0xe6, 0x75, 0xf6};
    const auto code = instruction(input);
    memory.writable.set();
    memory.install(vector, target);
    memory.install(handler, nops);
    memory.install(handler, std::span(code).first(input.opcode == pull ? 3 : bytes(input)));
    const std::array<std::uint8_t, 1> data{
        static_cast<std::uint8_t>(writes(input) ? input.data ^ byte_mask : input.data)};
    if (!input.self_modify) { memory.install(address(input), data); }
    const auto hx = initial_hx(input);
    cpu.set_state({.pc = entry,
                   .hx = static_cast<std::uint16_t>(hx),
                   .sp = stack,
                   .a = static_cast<std::uint8_t>(
                       writes(input) && input.opcode != mov ? input.data : accumulator),
                   .ccr_known = input.known,
                   .ccr = static_cast<std::uint8_t>(flags[input.data % flags.size()]),
                   .a_known = input.a_known,
                   .hx_known = input.hx_known});
    memory.interrupt = {.vector = vector};
    require(cpu.start_irq_bus(Hcs08IrqBusContract::reference_unstretched_sequence),
            "Only a real source supplies the actual frame/vector/program queue");
    for (unsigned n = 0; n < 11; ++n) { cpu.advance_irq_bus(); }
    cpu.start_irq_program(contract);
    if (input.opcode == pull) {
      for (unsigned n = 0; n < 3; ++n) { cpu.advance_irq_program(); }
    }
    memory.watch = address(input);
    memory.change_after_read = !writes(input);
    memory.reads = 0;
    memory.writes = 0;
  }
};

Hcs08State effect(const Input input, Hcs08State state) {
  state.pc = static_cast<std::uint16_t>(state.pc + bytes(input));
  if (input.opcode == pull) {
    state.sp = static_cast<std::uint16_t>(state.sp + 1U);
    state.hx = static_cast<std::uint16_t>((input.data << 8U) | low_index);
    return state;
  }
  if (!writes(input)) { state.a = static_cast<std::uint8_t>(input.data); }
  unsigned flags = state.ccr & ~(flag_v | flag_n | flag_z);
  if (input.data == 0) { flags |= flag_z; }
  if ((input.data & flag_v) != 0) { flags |= flag_n; }
  state.ccr = static_cast<std::uint8_t>(flags);
  return state;
}

void check_case(Machine &machine, const Input input) {
  machine.prepare(input);
  const auto before = machine.cpu.state();
  const auto expected = effect(input, before);
  const auto initial = owned(machine.cpu);
  const auto start = machine.memory.accesses.size();
  const auto retired = machine.memory.retired;
  unsigned next_program = before.pc + 3;
  std::vector<Access> accesses;
  for (unsigned phase = 1; phase <= periods(input); ++phase) {
    machine.cpu.advance_irq_program();
    const bool data = phase == data_phase(input);
    const bool free = input.opcode == pull && phase == 2;
    if (!free) {
      const unsigned location = data ? address(input) : next_program++;
      const unsigned value = data ? input.data : machine.memory.bytes[location];
      accesses.push_back({.address = location, .value = value, .write = data && writes(input)});
    }
    const bool executed = writes(input) ? phase >= data_phase(input) : phase > data_phase(input);
    const auto q = owned(machine.cpu);
    require(
        machine.cpu.state() == (executed ? expected : before) && q.native_executed == executed &&
            q.completed_periods == initial.completed_periods + phase &&
            machine.memory.retired == retired + static_cast<unsigned>(phase == periods(input)) &&
            machine.memory.reads ==
                static_cast<unsigned>(!writes(input) && phase >= data_phase(input)) &&
            machine.memory.writes ==
                static_cast<unsigned>(writes(input) && phase >= data_phase(input)) &&
            machine.memory.accesses.size() == start + accesses.size(),
        "Independent wpp/pwpp/rpp/prpp/ufp native effects opcode=" + hex(input.opcode) +
            " data=" + hex(input.data) + " phase=" + std::to_string(phase));
    for (unsigned n = 0; n < accesses.size(); ++n) {
      require(machine.memory.accesses[start + n] == accesses[n],
              "Every actual peripheral/program access is ordered");
    }
    if (!writes(input)) {
      const bool sampled = phase >= data_phase(input);
      const auto value =
          sampled ? std::optional(static_cast<std::uint8_t>(input.data)) : std::nullopt;
      require(
          q.operand.value == value && q.operand.in_flight == (phase == data_phase(input)) &&
              q.operand.consumed == executed,
          "Native loads/pull consume exactly the real captured value after its one-period arrival");
    }
  }
  const auto q = owned(machine.cpu);
  require(q.head == expected.pc && q.queued_bytes == 3 && q.ready_bytes == 2 &&
              q.program_byte_in_flight,
          "The native successor retains its actual two ready bytes and one pending arrival");
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

void fresh_continuations() {
  constexpr std::array<unsigned, 6> opcodes{
      sta_direct, sta_extended, mov, indexed8, indexed16, pull};
  for (const auto opcode : opcodes) {
    const Input input{.opcode = opcode, .data = 0x80, .known = partial};
    Machine baseline;
    baseline.prepare(input);
    for (unsigned n = 0; n < periods(input); ++n) { baseline.cpu.advance_irq_program(); }
    for (unsigned cut = 0; cut <= periods(input); ++cut) {
      const auto saved = capture(input, cut);
      for (unsigned replay = 0; replay < 2; ++replay) {
        Machine fresh;
        fresh.memory = saved.memory;
        fresh.cpu.restore_checkpoint(saved.cpu);
        for (unsigned n = cut; n < periods(input); ++n) { fresh.cpu.advance_irq_program(); }
        require(fresh.cpu.state() == baseline.cpu.state() &&
                    fresh.cpu.prefetch_state() == baseline.cpu.prefetch_state() &&
                    fresh.memory.accesses == baseline.memory.accesses &&
                    fresh.memory.bytes == baseline.memory.bytes &&
                    fresh.memory.reads == baseline.memory.reads &&
                    fresh.memory.writes == baseline.memory.writes,
                "Fresh joint owners never repeat a committed store or captured indexed/stack read");
      }
    }
  }
}

void write_failures() {
  constexpr Input input{.opcode = sta_extended, .data = 0x81, .known = partial};
  for (const bool after : {false, true}) {
    Machine machine;
    machine.prepare(input);
    machine.cpu.advance_irq_program();
    const auto saved = machine.cpu.checkpoint();
    const auto memory = machine.memory;
    machine.memory.fail_before_write = !after;
    machine.memory.fail_after_write = after;
    require(rejected([&] { machine.cpu.advance_irq_program(); }).contains("Own refusal") &&
                machine.cpu.faulted() && machine.memory.writes == static_cast<unsigned>(after) &&
                owned(machine.cpu).completed_periods == 1 && !owned(machine.cpu).native_executed &&
                machine.memory.bytes[address(input)] ==
                    (after ? input.data : input.data ^ byte_mask),
            "A failed native w retains its actual committed prefix rather than rolling back or "
            "succeeding");
    require(rejected([&] { machine.cpu.advance_irq_program(); }).contains("coordinated recovery"),
            "A partial native store cannot continue without joint recovery");
    machine.memory = memory;
    machine.cpu.restore_checkpoint(saved);
    for (unsigned n = 1; n < periods(input); ++n) { machine.cpu.advance_irq_program(); }
    require(machine.memory.writes == 1 && machine.memory.retired == 1,
            "Coordinated CPU/RAM recovery completes one genuine store and retirement");
  }
}

void unknown_operands() {
  for (const auto opcode : {indexed8, indexed16, sta_direct, sta_extended}) {
    const Input input{.opcode = opcode,
                      .data = 1,
                      .known = partial,
                      .hx_known = static_cast<std::uint16_t>(
                          opcode == indexed8 || opcode == indexed16 ? byte_mask : word_mask),
                      .a_known = static_cast<std::uint8_t>(
                          opcode == sta_direct || opcode == sta_extended ? 0U : byte_mask)};
    Machine machine;
    machine.prepare(input);
    for (unsigned n = 1; n < data_phase(input); ++n) { machine.cpu.advance_irq_program(); }
    const auto accesses = machine.memory.accesses;
    require(rejected([&] {
              machine.cpu.advance_irq_program();
            }).contains(writes(input) ? "partial-register" : "address=unknown") &&
                machine.cpu.faulted() && machine.memory.accesses == accesses &&
                machine.memory.reads == 0 && machine.memory.writes == 0,
            "Unknown native A or effective H:X cannot be replaced by a representative data "
            "transaction");
  }
}

void self_modifying_store() {
  constexpr Input input{
      .opcode = sta_extended, .data = 0x98, .known = partial, .self_modify = true};
  Machine machine;
  machine.prepare(input);
  for (unsigned n = 0; n < periods(input); ++n) { machine.cpu.advance_irq_program(); }
  require(owned(machine.cpu).queued[0] == nop && machine.memory.bytes[handler + 3] == input.data &&
              machine.memory.writes == 1,
          "Actual p1 captured NOP survives the later real w2 store of CLC to its backing address");
  machine.cpu.advance_irq_program();
  require(machine.cpu.state().pc == handler + 4 && (machine.cpu.state().ccr & flag_c) != 0,
          "Native execution consumes the captured NOP, not the newly stored backing CLC");
}
}

int main() {
  try {
    Machine machine;
    for (const auto opcode : {sta_direct, sta_extended, mov, indexed8, indexed16, pull}) {
      for (unsigned data = 0; data < byte_values; ++data) {
        for (const auto known : {std::uint8_t{0xff}, std::uint8_t{0xef}}) {
          check_case(machine, {.opcode = opcode, .data = data, .known = known});
        }
      }
    }
    check_case(machine, {.opcode = mov, .data = move_pattern, .known = partial, .a_known = 0});
    fresh_continuations();
    write_failures();
    unknown_operands();
    self_modifying_store();
    std::cout << "HCS08 peripheral cycles:3072 independent byte/mask cases,54 fresh suffixes,"
                 "real store/indexed-load/pull effects and strict failed/unknown operands pass.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
