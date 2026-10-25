// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_known_memory.hpp"

#include <algorithm>
#include <array>
#include <bitset>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
constexpr unsigned handler = 0x9000, vector = 0xffe0, stack = 0x240;
constexpr unsigned rti = 0x80, nop = 0x9d, clc = 0x98, sec = 0x99;
constexpr unsigned frame_bytes = 5, execute_phase = 6, periods = 9;
constexpr unsigned partial = 0xef, pattern = 0xa5, high_index = 0x5a00;
constexpr std::uint8_t alternating = 0x55, inverse_alternating = 0xaa;
constexpr auto contract = Hcs08PrefetchContract::reference_peripheral_program_cycles;

struct Input {
  unsigned flags = 0, data = pattern, sp = stack;
  std::uint8_t ccr_known = partial, a_known = byte_mask, x_known = byte_mask;
  bool change = true;
};

Hcs08State expected(const Input input) {
  return {.pc = entry,
          .hx = static_cast<std::uint16_t>(high_index | (input.data & input.x_known)),
          .sp = static_cast<std::uint16_t>(input.sp),
          .a = static_cast<std::uint8_t>((input.data ^ byte_mask) & input.a_known),
          .ccr_known = input.ccr_known,
          .ccr = static_cast<std::uint8_t>((input.flags | flags_fixed) & input.ccr_known),
          .a_known = input.a_known,
          .hx_known = static_cast<std::uint16_t>((word_mask ^ byte_mask) | input.x_known)};
}

Hcs08PrefetchState owned(const Hcs08 &cpu) {
  const auto q = cpu.prefetch_state();
  if (!q) { throw Fault("Expected owned RTI program queue"); }
  return *q;
}

class MemoryOwner final : public MaskedMemory {
public:
  std::bitset<space_bytes> watch;
  std::optional<unsigned> failure;
  bool after = false, change = false;
  unsigned samples = 0;

  void before_read(const unsigned address) const {
    if (failure == address && !after) { fail(address, "own RTI before-read refusal"); }
  }

  void after_read(const unsigned address) {
    if (watch[address]) {
      ++samples;
      if (change) {
        bytes[address] ^= byte_mask;
        ++generation;
      }
    }
    if (failure == address && after) { fail(address, "own RTI after-read refusal"); }
  }

  std::uint8_t read(const std::uint16_t address) override {
    before_read(address);
    const auto value = Memory::read(address);
    after_read(address);
    return value;
  }

  Hcs08KnownByte read_known_byte(const std::uint16_t address) override {
    const bool whole = known[address] || !writable[address];
    before_read(address);
    const auto value = MaskedMemory::read_known_byte(address);
    if (!whole) { after_read(address); }
    return value;
  }
};

struct Machine {
  MemoryOwner memory;
  Hcs08 cpu;

  explicit Machine(const Hcs08RtiIrqContract policy = Hcs08RtiIrqContract::unqualified)
      : cpu(memory, BLOFELD_HCS08_SLA, policy) {}

  void prepare(const Input input) {
    memory = MemoryOwner{};
    memory.writable.set();
    constexpr std::array<std::uint8_t, 2> target{0x90, 0};
    constexpr std::array<std::uint8_t, 3> code{rti, clc, sec};
    constexpr std::array<std::uint8_t, 4> foreground{nop, sec, clc, nop};
    memory.install(vector, target);
    memory.install(handler, code);
    memory.install(entry, foreground);
    cpu.set_state(expected(input));
    memory.interrupt = {.vector = vector};
    require(cpu.start_irq_bus(Hcs08IrqBusContract::reference_unstretched_sequence),
            "The actual IRQ entry supplies all five frame bytes");
    for (unsigned n = 0; n < 11; ++n) { cpu.advance_irq_bus(); }
    cpu.start_irq_program(contract);
    for (unsigned n = 0; n < frame_bytes; ++n) {
      memory.watch.set(static_cast<std::uint16_t>(input.sp + space_bytes - n));
    }
    memory.change = input.change;
    // Already fetched RTI/successors must survive changes to their backing code.
    constexpr std::array<std::uint8_t, 3> changed{nop, nop, nop};
    memory.install(handler, changed);
  }
};

std::array<Hcs08KnownByte, frame_bytes> frame(const Input input) {
  const auto state = expected(input);
  return {{{.value = state.ccr, .known = state.ccr_known},
           {.value = state.a, .known = state.a_known},
           {.value = static_cast<std::uint8_t>(state.hx), .known = input.x_known},
           {.value = static_cast<std::uint8_t>(entry >> 8U)},
           {.value = static_cast<std::uint8_t>(entry & byte_mask)}}};
}

void check_case(Machine &machine, const Input input) {
  machine.prepare(input);
  const auto before = machine.cpu.state();
  const auto start = machine.memory.accesses.size();
  const auto saved_frame = frame(input);
  std::vector<Access> accesses;
  for (unsigned phase = 1; phase <= periods; ++phase) {
    machine.cpu.advance_irq_program();
    if (phase <= frame_bytes) {
      const auto byte = saved_frame.at(phase - 1);
      accesses.push_back({.address = static_cast<std::uint16_t>(before.sp + phase),
                          .value = byte.value,
                          .write = false,
                          .known = byte.known});
    } else if (phase > execute_phase) {
      const unsigned offset = phase - execute_phase - 1;
      constexpr std::array<unsigned, 3> foreground{nop, sec, clc};
      accesses.push_back(
          {.address = entry + offset, .value = foreground.at(offset), .write = false});
    }
    const auto q = owned(machine.cpu);
    const bool executed = phase >= execute_phase;
    require(machine.cpu.state() == (executed ? expected(input) : before) &&
                q.native_executed == executed && q.completed_periods == phase &&
                q.attempted_periods == phase &&
                q.retired_instructions == static_cast<unsigned>(phase == periods) &&
                machine.memory.retired == static_cast<unsigned>(phase == periods) &&
                machine.memory.samples == std::min(phase, frame_bytes) &&
                machine.memory.accesses.size() == start + accesses.size(),
            "Independent uuuuufppp state, arrival, consumption and retirement phase=" +
                std::to_string(phase));
    for (unsigned n = 0; n < accesses.size(); ++n) {
      require(machine.memory.accesses[start + n] == accesses[n],
              "RTI reads each real frame byte once, then the native restored PC");
    }
    for (unsigned n = 0; n < frame_bytes; ++n) {
      const auto &read = q.return_reads[n];
      const bool sampled = phase > n;
      require(read.value == (sampled ? std::optional(saved_frame[n].value) : std::nullopt) &&
                  (!sampled || (read.address == static_cast<std::uint16_t>(before.sp + n + 1) &&
                                read.known == saved_frame[n].known)) &&
                  read.in_flight == (phase == n + 1) && read.consumed == executed,
              "Each RTI raw-copy latch preserves its actual value, mask and one-period arrival");
    }
    if (executed) {
      require(q.discarded == Hcs08DiscardedProgramRead{.address = handler + 1,
                                                       .value = clc,
                                                       .in_flight = false} &&
                  q.return_discarded == Hcs08DiscardedProgramRead{.address = handler + 2,
                                                                  .value = sec,
                                                                  .in_flight = false} &&
                  q.head == entry && q.next_read == entry + phase - execute_phase &&
                  q.queued_bytes == phase - execute_phase &&
                  q.ready_bytes == (phase > execute_phase ? phase - execute_phase - 1 : 0) &&
                  q.program_byte_in_flight == (phase > execute_phase),
              "Both genuine sequential successors are discarded before actual native-PC refill");
    }
  }
  require(machine.cpu.irq_delay() == Hcs08IrqDelay::unqualified_return,
          "Completing RTI does not silently qualify the conflicting IRQ-delay references");
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
  for (const unsigned sp : {stack, 2U}) {
    const Input input{.sp = sp, .a_known = alternating, .x_known = inverse_alternating};
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
        require(fresh.cpu.state() == baseline.cpu.state() &&
                    fresh.cpu.prefetch_state() == baseline.cpu.prefetch_state() &&
                    fresh.cpu.irq_delay() == baseline.cpu.irq_delay() &&
                    fresh.memory.accesses == baseline.memory.accesses &&
                    fresh.memory.bytes == baseline.memory.bytes &&
                    fresh.memory.samples == frame_bytes && fresh.memory.retired == 1,
                "Fresh RTI owners retain every real read, raw mask, discard and refill prefix");
      }
    }
  }
}

void read_failures() {
  for (const unsigned phase : {1U, 2U, 3U, 4U, 5U, 7U, 8U, 9U}) {
    for (const bool after : {false, true}) {
      Machine machine;
      machine.prepare({});
      for (unsigned n = 1; n < phase; ++n) { machine.cpu.advance_irq_program(); }
      const auto saved = machine.cpu.checkpoint();
      const auto memory = machine.memory;
      const auto state = machine.cpu.state();
      const auto size = memory.accesses.size();
      machine.memory.failure =
          phase <= frame_bytes ? stack - frame_bytes + phase : entry + phase - execute_phase - 1;
      machine.memory.after = after;
      const auto error = rejected([&] { machine.cpu.advance_irq_program(); });
      require(error.contains("own RTI") && error.contains("pc=0x9000") && machine.cpu.faulted() &&
                  machine.cpu.state() == state &&
                  machine.memory.accesses.size() == size + static_cast<unsigned>(after) &&
                  owned(machine.cpu).completed_periods == phase - 1 &&
                  owned(machine.cpu).attempted_periods == phase && machine.memory.retired == 0,
              "RTI read failures preserve the actual bus/native prefix and contextual fault");
      require(rejected([&] { machine.cpu.advance_irq_program(); }).contains("coordinated recovery"),
              "A failed RTI period cannot repeat a read without joint recovery");
      machine.memory = memory;
      machine.cpu.restore_checkpoint(saved);
      for (unsigned n = phase; n <= periods; ++n) { machine.cpu.advance_irq_program(); }
      require(machine.cpu.state() == expected({}) && machine.memory.samples == frame_bytes &&
                  machine.memory.retired == 1,
              "Coordinated RTI recovery resumes each actual read exactly once");
    }
  }
}

void unknown_pc() {
  for (const unsigned phase : {4U, 5U}) {
    Machine machine;
    machine.prepare({});
    for (unsigned n = 1; n < phase; ++n) { machine.cpu.advance_irq_program(); }
    const auto before = machine.cpu.state();
    const auto size = machine.memory.accesses.size();
    machine.memory.known.reset(stack - frame_bytes + phase);
    require(rejected([&] { machine.cpu.advance_irq_program(); }).contains("Unknown HCS08") &&
                machine.cpu.faulted() && machine.cpu.state() == before &&
                machine.memory.accesses.size() == size && !owned(machine.cpu).native_executed,
            "Unknown return PC bytes fail before native register/PC restoration");
  }
}

void pending_irq_boundaries() {
  for (const auto policy : {Hcs08RtiIrqContract::unqualified,
                            Hcs08RtiIrqContract::reference_immediate,
                            Hcs08RtiIrqContract::reference_next_instruction}) {
    Machine machine(policy);
    machine.prepare({});
    for (unsigned n = 0; n < periods; ++n) { machine.cpu.advance_irq_program(); }
    machine.memory.interrupt = {.vector = vector};
    const bool protected_instruction = policy == Hcs08RtiIrqContract::reference_next_instruction;
    if (protected_instruction) {
      machine.cpu.advance_irq_program();
      auto state = expected({});
      ++state.pc;
      require(machine.cpu.state() == state && machine.memory.retired == 2 &&
                  machine.cpu.irq_delay() == Hcs08IrqDelay::none,
              "The explicit next-instruction RTI contract protects exactly one actual queued NOP");
    }
    const auto state = machine.cpu.state();
    const auto accesses = machine.memory.accesses;
    const auto q = owned(machine.cpu);
    const auto error = rejected([&] { machine.cpu.advance_irq_program(); });
    require(error.contains(policy == Hcs08RtiIrqContract::unqualified
                               ? "RTI IRQ timing contract"
                               : "qualified queue handoff") &&
                error.contains(protected_instruction ? "pc=0x8001" : "pc=0x8000") &&
                machine.cpu.faulted() && machine.cpu.state() == state &&
                machine.memory.accesses == accesses && machine.memory.interrupt &&
                machine.memory.interrupt->vector == vector &&
                machine.memory.retired == (protected_instruction ? 2U : 1U) &&
                owned(machine.cpu).return_reads == q.return_reads &&
                owned(machine.cpu).return_discarded == q.return_discarded &&
                owned(machine.cpu).completed_periods == q.completed_periods &&
                owned(machine.cpu).attempted_periods == q.attempted_periods + 1,
            "Owned program boundaries never drop a pending IRQ, discard its frame history or "
            "silently select an RTI-delay policy");
  }
}
}

int main() {
  try {
    Machine machine;
    for (unsigned flags = 0; flags < byte_values; ++flags) {
      if ((flags & (flags_fixed | flag_i)) != 0) { continue; }
      for (const auto known : {std::uint8_t{byte_mask}, std::uint8_t{partial}}) {
        for (const unsigned data : {0U, 1U, 0x80U, byte_mask}) {
          check_case(machine, {.flags = flags, .data = data, .ccr_known = known});
        }
      }
    }
    for (unsigned mask = 0; mask < byte_values; ++mask) {
      check_case(machine,
                 {.a_known = static_cast<std::uint8_t>(mask),
                  .x_known = static_cast<std::uint8_t>(mask ^ byte_mask)});
    }
    for (const unsigned sp : {0U, 1U, 2U, 3U, 4U, word_mask}) { check_case(machine, {.sp = sp}); }
    fresh_continuations();
    read_failures();
    unknown_pc();
    pending_irq_boundaries();
    std::cout
        << "HCS08 RTI cycles:256 flag/data cases,256 raw-mask cases,6 stack wraps,"
           "40 fresh suffixes,16 prefix failures,unknown PC refusals and3 IRQ policies pass.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
