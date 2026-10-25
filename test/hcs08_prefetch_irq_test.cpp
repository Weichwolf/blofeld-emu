// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"

#include <algorithm>
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
constexpr unsigned handler = 0x9000, alternate = 0x9100, vector = 0xffe0, high_vector = 0xfffa;
constexpr unsigned stack = 0x240, rti = 0x80, nop = 0x9d, clc = 0x98, sec = 0x99;
constexpr unsigned irq_periods = 11, return_periods = 9, total = irq_periods + return_periods;
constexpr unsigned pattern = 0xa5, index = 0xb73c, partial = 0xef;
constexpr std::uint8_t alternating = 0x55, inverse_alternating = 0xaa;
constexpr auto program_contract = Hcs08PrefetchContract::reference_peripheral_program_cycles;
constexpr auto irq_contract = Hcs08IrqBusContract::reference_owned_program_handoff;

struct Input {
  unsigned flags = 1, pc = entry, sp = stack;
  std::uint8_t a_known = byte_mask, x_known = byte_mask, ccr_known = partial;
  bool late = false;
};

Hcs08State expected(const Input input) {
  return {
      .pc = static_cast<std::uint16_t>(input.pc),
      .hx = static_cast<std::uint16_t>((index & (word_mask ^ byte_mask)) | (index & input.x_known)),
      .sp = static_cast<std::uint16_t>(input.sp),
      .a = static_cast<std::uint8_t>(pattern & input.a_known),
      .ccr_known = input.ccr_known,
      .ccr = static_cast<std::uint8_t>((flags_fixed | input.flags) & input.ccr_known),
      .a_known = input.a_known,
      .hx_known = static_cast<std::uint16_t>((word_mask ^ byte_mask) | input.x_known)};
}

Hcs08PrefetchState program(const Hcs08 &cpu) {
  const auto q = cpu.prefetch_state();
  if (!q) { throw Fault("Expected an owned program queue"); }
  return *q;
}

Hcs08IrqBusState sequence(const Hcs08 &cpu) {
  const auto q = cpu.irq_bus_state();
  if (!q) { throw Fault("Expected an owned queue-to-IRQ sequence"); }
  return *q;
}

class MemoryOwner final : public MaskedMemory {
public:
  std::optional<unsigned> failure;
  bool after = false;
  unsigned entries = 0, acknowledgements = 0;

  void check(const unsigned address, const bool completed) const {
    if (failure == address && completed == after) { fail(address, "own handoff access refusal"); }
  }

  std::uint8_t read(const std::uint16_t address) override {
    check(address, false);
    const auto value = Memory::read(address);
    check(address, true);
    return value;
  }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    check(address, false);
    Memory::write(address, value);
    check(address, true);
  }

  void write_known_byte(const ByteWrite access, const std::uint8_t known) override {
    check(access.address, false);
    Memory::write_known_byte(access, known);
    check(access.address, true);
  }

  void begin_interrupt(const InterruptRequest request, const bool reuse) override {
    require(request.vector == vector && !reuse, "Both real entries initially admit the own source");
    ++entries;
  }

  void acknowledge_interrupt(const std::uint16_t selected) override {
    require(selected == vector || selected == high_vector,
            "Only the actual selected source is notified");
    ++acknowledgements;
  }
};

struct Machine {
  MemoryOwner memory;
  Hcs08 cpu;

  explicit Machine(const Hcs08RtiIrqContract policy = Hcs08RtiIrqContract::reference_immediate)
      : cpu(memory, BLOFELD_HCS08_SLA, policy) {}

  void prepare(const Input input, const unsigned elapsed = return_periods) {
    memory = MemoryOwner{};
    memory.writable.set();
    constexpr std::array<std::uint8_t, 3> code{rti, clc, sec};
    constexpr std::array<std::uint8_t, 2> target{0x90, 0};
    constexpr std::array<std::uint8_t, 2> other{0x91, 0};
    constexpr std::array<std::uint8_t, 4> foreground{nop, sec, clc, nop};
    memory.install(vector, target);
    memory.install(high_vector, other);
    memory.install(handler, code);
    memory.install(alternate, code);
    for (unsigned n = 0; n < foreground.size(); ++n) {
      const std::array<std::uint8_t, 1> byte{foreground[n]};
      memory.install(static_cast<std::uint16_t>(input.pc + n), byte);
    }
    cpu.set_state(expected(input));
    memory.interrupt = {.vector = vector};
    require(cpu.start_irq_bus(Hcs08IrqBusContract::reference_unstretched_sequence),
            "Actual first IRQ generates the frame and instruction queue");
    for (unsigned n = 0; n < irq_periods; ++n) { cpu.advance_irq_bus(); }
    cpu.start_irq_program(program_contract);
    for (unsigned n = 0; n < elapsed; ++n) { cpu.advance_irq_program(); }
    if (elapsed == return_periods) {
      constexpr std::array<std::uint8_t, 1> changed{clc};
      memory.install(input.pc, changed);
    }
  }

  void advance(const Input input, const unsigned period) {
    if (period == 5 && input.late) { memory.interrupt = {.vector = high_vector}; }
    if (period < irq_periods) {
      cpu.advance_irq_bus();
    } else {
      if (period == irq_periods) { cpu.start_irq_program(program_contract); }
      cpu.advance_irq_program();
    }
  }
};

std::vector<Access> expected_accesses(const Input input) {
  const auto s = expected(input);
  const unsigned selected = input.late ? high_vector : vector;
  const unsigned target = input.late ? alternate : handler;
  const auto location = [s](const unsigned n) {
    return static_cast<std::uint16_t>(s.sp + space_bytes - n);
  };
  return {
      {.address = location(0), .value = s.pc & byte_mask, .write = true},
      {.address = location(1), .value = static_cast<unsigned>(s.pc) >> 8U, .write = true},
      {.address = location(2), .value = s.hx & byte_mask, .write = true, .known = input.x_known},
      {.address = location(3), .value = s.a, .write = true, .known = s.a_known},
      {.address = location(4), .value = s.ccr, .write = true, .known = s.ccr_known},
      {.address = selected, .value = target >> 8U, .write = false},
      {.address = selected + 1, .value = target & byte_mask, .write = false},
      {.address = target, .value = rti, .write = false},
      {.address = target + 1, .value = clc, .write = false},
      {.address = target + 2, .value = sec, .write = false}};
}

void check_case(Machine &machine, const Input input) {
  machine.prepare(input);
  const auto before = expected(input);
  auto archived = program(machine.cpu);
  require(machine.cpu.state() == before, "The first genuine RTI restores the declared caller");
  const auto start = machine.memory.accesses.size();
  require(
      machine.cpu.start_irq_bus(irq_contract) && !machine.cpu.prefetch_state() &&
          sequence(machine.cpu).discarded_program == archived &&
          machine.memory.accesses.size() == start && machine.memory.entries == 1 &&
          machine.memory.acknowledgements == 1 && machine.cpu.state() == before,
      "Queue handoff captures all actual state without a bus read, frame write or notification");
  archived.ready_bytes = 3;
  archived.program_byte_in_flight = false;
  const auto accesses = expected_accesses(input);
  unsigned count = 0;
  for (unsigned phase = 1; phase <= irq_periods; ++phase) {
    machine.advance(input, phase - 1);
    auto state = before;
    state.sp = static_cast<std::uint16_t>(before.sp + space_bytes - std::min(phase, 5U));
    if (phase >= 5) { state.ccr |= flag_i; }
    if (phase >= 8) { state.pc = static_cast<std::uint16_t>(input.late ? alternate : handler); }
    if (phase != 8) { ++count; }
    const auto q = sequence(machine.cpu);
    require(machine.cpu.state() == state && q.entry == before && q.completed_cycles == phase &&
                q.attempted_cycles == phase && q.discarded_program == archived &&
                machine.memory.accesses.size() == start + count && machine.memory.retired == 1 &&
                machine.memory.entries == 2 &&
                machine.memory.acknowledgements == (phase >= 7 ? 2U : 1U),
            "Independent new IRQ frame/vector phases retain the discarded queue and its arrival");
    for (unsigned n = 0; n < count; ++n) {
      require(machine.memory.accesses[start + n] == accesses[n],
              "Every second IRQ stack/vector/program access follows independent expectations");
    }
  }
  machine.cpu.start_irq_program(program_contract);
  for (unsigned n = 0; n < return_periods; ++n) { machine.cpu.advance_irq_program(); }
  require(machine.cpu.state() == before && machine.memory.retired == 2 &&
              program(machine.cpu).retired_instructions == 1,
          "The second native RTI returns through its newly written real frame");
}

struct Snapshot {
  MemoryOwner memory;
  Hcs08Checkpoint cpu;
};

Snapshot capture(const Input input, const unsigned cut) {
  Machine original;
  original.prepare(input);
  require(original.cpu.start_irq_bus(irq_contract), "Actual pending source admits the handoff");
  for (unsigned n = 0; n < cut; ++n) { original.advance(input, n); }
  return {.memory = original.memory, .cpu = original.cpu.checkpoint()};
}

void fresh_continuations() {
  for (const bool late : {false, true}) {
    const Input input{.a_known = alternating, .x_known = inverse_alternating, .late = late};
    Machine baseline;
    baseline.prepare(input);
    require(baseline.cpu.start_irq_bus(irq_contract), "Real source admits baseline handoff");
    for (unsigned n = 0; n < total; ++n) { baseline.advance(input, n); }
    for (unsigned cut = 0; cut <= total; ++cut) {
      const auto saved = capture(input, cut);
      for (unsigned replay = 0; replay < 2; ++replay) {
        Machine fresh;
        fresh.memory = saved.memory;
        fresh.cpu.restore_checkpoint(saved.cpu);
        for (unsigned n = cut; n < total; ++n) { fresh.advance(input, n); }
        require(
            fresh.cpu.state() == baseline.cpu.state() &&
                fresh.cpu.prefetch_state() == baseline.cpu.prefetch_state() &&
                fresh.memory.bytes == baseline.memory.bytes &&
                fresh.memory.known == baseline.memory.known &&
                fresh.memory.partial_known == baseline.memory.partial_known &&
                fresh.memory.accesses == baseline.memory.accesses && fresh.memory.entries == 2 &&
                fresh.memory.acknowledgements == 2 && fresh.memory.retired == 2,
            "Fresh owners retain discarded arrivals, late selection and the genuine second frame");
      }
    }
  }
}

void access_failures() {
  constexpr Input input{};
  const auto accesses = expected_accesses(input);
  for (unsigned phase = 1; phase <= irq_periods; ++phase) {
    if (phase == 8) { continue; }
    for (const bool after : {false, true}) {
      Machine machine;
      machine.prepare(input);
      require(machine.cpu.start_irq_bus(irq_contract), "Real source admits failed-prefix fixture");
      for (unsigned n = 1; n < phase; ++n) { machine.cpu.advance_irq_bus(); }
      const auto saved = machine.cpu.checkpoint();
      const auto memory = machine.memory;
      const auto state = machine.cpu.state();
      const auto archive = sequence(machine.cpu).discarded_program;
      machine.memory.failure = accesses.at(phase < 8 ? phase - 1 : phase - 2).address;
      machine.memory.after = after;
      require(rejected([&] { machine.cpu.advance_irq_bus(); }).contains("own handoff") &&
                  machine.cpu.faulted() && machine.cpu.state() == state &&
                  machine.memory.accesses.size() ==
                      memory.accesses.size() + static_cast<unsigned>(after) &&
                  sequence(machine.cpu).completed_cycles == phase - 1 &&
                  sequence(machine.cpu).discarded_program.has_value(),
              "Failed new IRQ access retains its real prefix and captured prior program owner");
      auto expected_archive = archive;
      if (!expected_archive) { throw Fault("Expected prior queue archive"); }
      expected_archive->ready_bytes = 3;
      expected_archive->program_byte_in_flight = false;
      require(sequence(machine.cpu).discarded_program == expected_archive &&
                  rejected([&] { machine.cpu.advance_irq_bus(); }).contains("coordinated recovery"),
              "Arrivals survive failed s1 and no partial new IRQ can retry silently");
      machine.memory = memory;
      machine.cpu.restore_checkpoint(saved);
      for (unsigned n = phase - 1; n < total; ++n) { machine.advance(input, n); }
      require(machine.cpu.state() == expected(input) && machine.memory.retired == 2,
              "Joint recovery resumes the real new frame and native RTI once");
    }
  }
}

void admission_boundaries() {
  for (unsigned elapsed = 0; elapsed < return_periods; ++elapsed) {
    Machine machine;
    machine.prepare({}, elapsed);
    const auto q = program(machine.cpu);
    const auto state = machine.cpu.state();
    const auto accesses = machine.memory.accesses;
    require(!machine.cpu.start_irq_bus(irq_contract) && !machine.cpu.faulted() &&
                program(machine.cpu) == q && machine.cpu.state() == state &&
                machine.memory.accesses == accesses,
            "A masked source or unretired RTI never transfers its actual queue");
  }
  Machine idle;
  idle.prepare({});
  idle.memory.interrupt.reset();
  const auto q = program(idle.cpu);
  require(!idle.cpu.start_irq_bus(irq_contract) && program(idle.cpu) == q && !idle.cpu.faulted(),
          "No real pending source means no queue mutation or synthetic IRQ");
  Machine ambiguous(Hcs08RtiIrqContract::unqualified);
  ambiguous.prepare({});
  const auto original = program(ambiguous.cpu);
  require(rejected([&] {
            static_cast<void>(ambiguous.cpu.start_irq_bus(irq_contract));
          }).contains("RTI IRQ timing") &&
              ambiguous.cpu.faulted() && program(ambiguous.cpu) == original &&
              !ambiguous.cpu.irq_bus_state(),
          "Owned handoff cannot choose a default policy for the contradictory RTI references");
  Machine delayed(Hcs08RtiIrqContract::reference_next_instruction);
  delayed.prepare({});
  const auto pending = program(delayed.cpu);
  require(!delayed.cpu.start_irq_bus(irq_contract) && program(delayed.cpu) == pending,
          "The next-instruction reference preserves queue ownership until one actual instruction");
  delayed.cpu.advance_irq_program();
  auto after_nop = expected({});
  ++after_nop.pc;
  require(delayed.cpu.state() == after_nop,
          "The actual queued NOP survives the changed backing CLC and preserves C");
  require(delayed.cpu.start_irq_bus(irq_contract) && sequence(delayed.cpu).entry.pc == entry + 1,
          "The real protected NOP completes before the newly stacked PC advances");
}
}

int main() {
  try {
    Machine machine;
    for (unsigned flags = 0; flags < byte_values; ++flags) {
      if ((flags & (flags_fixed | flag_i)) != 0) { continue; }
      for (const bool late : {false, true}) {
        for (const auto known : {std::uint8_t{byte_mask}, std::uint8_t{partial}}) {
          check_case(machine, {.flags = flags, .ccr_known = known, .late = late});
        }
      }
    }
    for (unsigned mask = 0; mask < byte_values; ++mask) {
      check_case(machine,
                 {.a_known = static_cast<std::uint8_t>(mask),
                  .x_known = static_cast<std::uint8_t>(mask ^ byte_mask)});
    }
    check_case(machine, {.sp = 2});
    check_case(machine, {.pc = word_mask - 2});
    fresh_continuations();
    access_failures();
    admission_boundaries();
    std::cout << "HCS08 queue IRQ:128 flags/priority cases,256 raw-mask cases,2 wraps,"
                 "84 fresh suffixes,20 failed prefixes and protected admission boundaries pass.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
