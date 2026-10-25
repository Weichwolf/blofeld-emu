// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
constexpr auto contract = Hcs08PrefetchContract::reference_data_branch_cycles;
constexpr unsigned handler = 0x9000, vector = 0xffe0, stack = 0x240;
constexpr unsigned accumulator = 0x5c, index = 0x1234, partial = 0xef;
constexpr unsigned index_high = 0x1200;
constexpr unsigned lda = 0xb6, ldx = 0xbe, cmp = 0xb1, bit7_clear = 0x0f;
constexpr unsigned lda_ext = 0xc6, ldx_ext = 0xce, cmp_ext = 0xc1;
constexpr unsigned extended_operand = 0x1234;
constexpr std::array<std::uint8_t, 16> nops{
    0x9d, 0x9d, 0x9d, 0x9d, 0x9d, 0x9d, 0x9d, 0x9d, 0x9d, 0x9d, 0x9d, 0x9d, 0x9d, 0x9d, 0x9d, 0x9d};

Hcs08PrefetchState owned(const Hcs08 &cpu) {
  const auto q = cpu.prefetch_state();
  if (!q) { throw Fault("Expected owned data/branch program"); }
  return *q;
}

class MemoryOwner final : public MaskedMemory {
public:
  unsigned data_reads = 0;
  bool change_after_read = true;
  unsigned data_address = operand;

  std::uint8_t read(const std::uint16_t address) override {
    const auto value = Memory::read(address);
    if (address == data_address) {
      ++data_reads;
      if (change_after_read) {
        bytes[address] = static_cast<std::uint8_t>(value ^ byte_mask);
        ++generation; // Own read-to-change fixture; not a claimed physical register behavior.
        if (decoded[address]) { ++code_generation; }
      }
    }
    return value;
  }
};

struct Input {
  unsigned opcode, data;
  int offset = 0;
  std::uint8_t known = byte_mask;
};

bool is_branch(const Input input) {
  return input.opcode < 16;
}

bool is_extended(const Input input) {
  return input.opcode >= cmp_ext;
}

unsigned data_address(const Input input) {
  return is_extended(input) ? extended_operand : operand;
}

unsigned read_phase(const Input input) {
  return is_extended(input) ? 2U : 1U;
}

unsigned instruction_periods(const Input input) {
  if (is_branch(input)) { return 5; }
  return is_extended(input) ? 4U : 3U;
}

unsigned selected_pc(const Input input) {
  if (!is_branch(input)) { return handler + (is_extended(input) ? 3U : 2U); }
  const bool tested = ((input.data >> (input.opcode / 2)) & 1U) != 0;
  const bool taken = tested == ((input.opcode % 2) == 0);
  const int target = static_cast<int>(handler) + 3 + (taken ? input.offset : 0);
  return static_cast<std::uint16_t>(target);
}

struct Machine {
  MemoryOwner memory;
  Hcs08 cpu{memory, BLOFELD_HCS08_SLA};

  void prepare(const Input input) {
    constexpr std::array<std::uint8_t, 2> target{0x90, 0};
    constexpr std::array<unsigned, 4> unrelated{0x60, 0xe6, 0x74, 0xf6};
    const unsigned bit =
        is_branch(input) ? (input.data >> (input.opcode / 2)) & 1U : input.data & 1U;
    const unsigned flags = unrelated[input.data % unrelated.size()] | (bit ^ 1U);
    const auto address = data_address(input);
    const std::array<std::uint8_t, 3> code{
        static_cast<std::uint8_t>(input.opcode),
        static_cast<std::uint8_t>(is_extended(input) ? address >> 8U : address),
        static_cast<std::uint8_t>(is_branch(input) ? input.offset : static_cast<int>(address))};
    memory.writable.set();
    memory.install(vector, target);
    memory.install(selected_pc(input), nops);
    memory.install(handler, nops);
    memory.install(handler, std::span(code).first(is_branch(input) || is_extended(input) ? 3 : 2));
    const std::array<std::uint8_t, 1> value{static_cast<std::uint8_t>(input.data)};
    memory.install(address, value);
    memory.data_address = address;
    memory.data_reads = 0;
    memory.retired = 0;
    memory.accesses.clear();
    cpu.set_state({.pc = entry,
                   .hx = index,
                   .sp = stack,
                   .a = accumulator,
                   .ccr_known = input.known,
                   .ccr = static_cast<std::uint8_t>(flags)});
    memory.interrupt = {.vector = vector};
    require(cpu.start_irq_bus(Hcs08IrqBusContract::reference_unstretched_sequence),
            "A real source supplies the actual frame/vector/queue");
    for (unsigned n = 0; n < 11; ++n) { cpu.advance_irq_bus(); }
    cpu.start_irq_program(contract);
  }
};

Hcs08State result(const Input input, Hcs08State before) {
  before.pc = static_cast<std::uint16_t>(selected_pc(input));
  if (is_branch(input)) {
    const unsigned tested = (input.data >> (input.opcode / 2)) & 1U;
    before.ccr = static_cast<std::uint8_t>((before.ccr & ~flag_c) | tested);
    return before;
  }
  unsigned arithmetic = input.data;
  unsigned defined = flag_v | flag_n | flag_z;
  unsigned computed = 0;
  if (input.opcode == lda || input.opcode == lda_ext) {
    before.a = static_cast<std::uint8_t>(input.data);
  }
  if (input.opcode == ldx || input.opcode == ldx_ext) {
    before.hx = static_cast<std::uint16_t>(index_high | input.data);
  }
  if (input.opcode == cmp || input.opcode == cmp_ext) {
    arithmetic = (accumulator + byte_values - input.data) & byte_mask;
    defined |= flag_c;
    if (accumulator < input.data) { computed |= flag_c; }
    if (((accumulator ^ input.data) & (accumulator ^ arithmetic) & flag_v) != 0) {
      computed |= flag_v;
    }
  }
  if (arithmetic == 0) { computed |= flag_z; }
  if ((arithmetic & flag_v) != 0) { computed |= flag_n; }
  before.ccr = static_cast<std::uint8_t>((before.ccr & ~defined) | computed);
  return before;
}

void check_accesses(const Machine &machine,
                    const std::size_t from,
                    const Input input,
                    const unsigned phase) {
  const auto &effects = machine.memory.accesses;
  require(effects.size() == from + phase &&
              machine.memory.data_reads == static_cast<unsigned>(phase >= read_phase(input)),
          "Exactly one genuine data read occupies its documented cycle");
  unsigned next_program = handler + 3;
  for (unsigned n = 1; n <= phase; ++n) {
    unsigned address = data_address(input);
    unsigned value = input.data;
    if (n != read_phase(input)) {
      address = is_branch(input) && n >= 3 ? selected_pc(input) + n - 3 : next_program++;
      value = machine.memory.bytes[address];
    }
    require(
        effects[from + n - 1] == Access{.address = address, .value = value, .write = false},
        "Independent rpp/prpp/rpppp addresses preserve data, speculation and selected-PC refill");
  }
}

void check_case(Machine &machine, const Input input) {
  machine.prepare(input);
  const auto before = machine.cpu.state();
  const auto expected = result(input, before);
  const auto accesses = machine.memory.accesses.size();
  const unsigned periods = instruction_periods(input);
  for (unsigned phase = 1; phase <= periods; ++phase) {
    machine.cpu.advance_irq_program();
    const auto q = owned(machine.cpu);
    const auto actual = machine.cpu.state();
    const bool sampled = phase >= read_phase(input);
    const bool executed = phase > read_phase(input);
    const auto captured =
        sampled ? std::optional(static_cast<std::uint8_t>(input.data)) : std::nullopt;
    require(actual == (executed ? expected : before) && q.operand.value == captured &&
                q.operand.address == (sampled ? data_address(input) : 0) &&
                q.operand.in_flight == (phase == read_phase(input)) &&
                q.operand.consumed == executed && q.native_executed == executed &&
                q.completed_periods == phase && q.attempted_periods == phase &&
                machine.memory.retired == static_cast<unsigned>(phase == periods),
            "Data opcode=" + hex(input.opcode) + " phase=" + std::to_string(phase) +
                " PC=" + hex(actual.pc) + " A=" + hex(actual.a) + " HX=" + hex(actual.hx) +
                " CCR=" + hex(actual.ccr) + " expected-PC=" + hex(expected.pc) +
                " expected-CCR=" + hex(expected.ccr));
    check_accesses(machine, accesses, input, phase);
    if (is_branch(input) && phase >= 2) {
      if (!q.discarded) { throw Fault("Missing actual speculative program observation"); }
      require(q.discarded->address == handler + 3 &&
                  q.discarded->value == machine.memory.bytes[handler + 3] &&
                  q.discarded->in_flight == (phase == 2),
              "A discarded speculative read retains its own real one-period propagation");
    }
    if (phase == periods) {
      require(q.head == expected.pc && q.ready_bytes == 2 && q.queued_bytes == 3 &&
                  q.program_byte_in_flight && q.retired_instructions == 1,
              "The actual selected successor retains two ready bytes and a third in flight");
    }
  }
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
  constexpr std::array<Input, 6> inputs{
      {{.opcode = bit7_clear, .data = 0, .offset = -1, .known = partial},
       {.opcode = bit7_clear, .data = 0x80, .offset = 127},
       {.opcode = lda, .data = 0x80, .known = partial},
       {.opcode = cmp, .data = 0xff, .known = partial},
       {.opcode = lda_ext, .data = 0x80, .known = partial},
       {.opcode = cmp_ext, .data = 0xff, .known = partial}}};
  for (const auto input : inputs) {
    const unsigned periods = instruction_periods(input);
    Machine baseline;
    baseline.prepare(input);
    for (unsigned n = 0; n < periods; ++n) { baseline.cpu.advance_irq_program(); }
    for (unsigned cut = 0; cut <= periods; ++cut) {
      const auto saved = capture(input, cut);
      for (unsigned replay = 0; replay < 2; ++replay) {
        Machine fresh;
        fresh.memory = saved.memory;
        fresh.cpu.restore_checkpoint(saved.cpu);
        for (unsigned n = cut; n < periods; ++n) { fresh.cpu.advance_irq_program(); }
        require(
            fresh.cpu.state() == baseline.cpu.state() &&
                fresh.cpu.prefetch_state() == baseline.cpu.prefetch_state() &&
                fresh.memory.accesses == baseline.memory.accesses &&
                fresh.memory.bytes == baseline.memory.bytes && fresh.memory.data_reads == 1,
            "Fresh owners resume every data/decision/speculation/refill phase without a reread");
      }
    }
  }
}

void failures_and_discard() {
  constexpr Input input{.opcode = bit7_clear, .data = 0x80, .offset = 1};
  Machine machine;
  machine.prepare(input);
  machine.cpu.advance_irq_program();
  const auto memory = machine.memory;
  const auto saved = machine.cpu.checkpoint();
  machine.memory.known.reset(handler + 3);
  require(rejected([&] { machine.cpu.advance_irq_program(); }).contains("address=0x9003") &&
              machine.cpu.faulted() && machine.memory.data_reads == 1 &&
              owned(machine.cpu).operand.value == input.data &&
              !owned(machine.cpu).operand.in_flight,
          "Failed p2 preserves the prior actual data arrival and read-side effect");
  machine.memory = memory;
  machine.cpu.restore_checkpoint(saved);
  machine.cpu.advance_irq_program();
  constexpr std::array<std::uint8_t, 3> changed{0x98, 0x9d, 0x9d};
  machine.memory.install(handler + 3, changed);
  for (unsigned n = 0; n < 3; ++n) { machine.cpu.advance_irq_program(); }
  const auto q = owned(machine.cpu);
  if (!q.discarded) { throw Fault("Missing speculative observation after recovery"); }
  require(
      q.discarded->value != q.queued[0] && q.queued[0] == changed[0] &&
          machine.memory.data_reads == 1,
      "Not-taken refill observes newer memory instead of reusing the discarded old program byte");
  machine.cpu.advance_irq_program();
  require((machine.cpu.state().ccr & flag_c) == 0 && machine.cpu.state().pc == handler + 4,
          "Native successor executes the actual new CLC, never the stale speculative NOP");
}
}

int main() {
  try {
    Machine machine;
    constexpr std::array<int, 5> offsets{-128, -1, 0, 1, 127};
    for (unsigned opcode = 0; opcode < 16; ++opcode) {
      for (unsigned data = 0; data < byte_values; ++data) {
        const Input input{
            .opcode = opcode, .data = data, .offset = offsets[(opcode + data) % offsets.size()]};
        check_case(machine, input);
        if (opcode >= 14) {
          auto unknown = input;
          unknown.known = partial;
          check_case(machine, unknown);
        }
      }
    }
    for (const auto opcode : {lda, ldx, cmp, lda_ext, ldx_ext, cmp_ext}) {
      for (unsigned data = 0; data < byte_values; ++data) {
        for (const auto known : {std::uint8_t{0xff}, std::uint8_t{0xef}}) {
          check_case(machine, {.opcode = opcode, .data = data, .known = known});
        }
      }
    }
    fresh_continuations();
    failures_and_discard();
    std::cout << "HCS08 data/bit-branch reference:7680 independent cases,60 fresh phase "
                 "continuations, one real data read and native selected-PC refill pass.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
