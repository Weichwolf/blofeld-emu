// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_flash.hpp"

#include <algorithm>
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
constexpr unsigned handler = 0x9000, vector = 0xffe0, stack = 0x240, irq_stack = 0x23b;
constexpr unsigned accumulator = 0x5c, index = 0x1234, flags = 0x69;
constexpr unsigned mul_opcode = 0x42, mul_hx = 0x7734, product_low = 0x43, product_hx = 0x1201;
constexpr unsigned carry_clear_flags = 0x68, partial_ccr = 0xef;
constexpr auto irq_contract = Hcs08IrqBusContract::reference_unstretched_sequence;
constexpr auto program_contract = Hcs08PrefetchContract::reference_linear_program_cycles;
constexpr std::array<std::uint8_t, 8> nops{0x9d, 0x9d, 0x9d, 0x9d, 0x9d, 0x9d, 0x9d, 0x9d};

Hcs08PrefetchState owned_program(const Hcs08 &cpu) {
  const auto q = cpu.prefetch_state();
  if (!q) { throw Fault("Expected owned prefetch queue"); }
  return *q;
}

class MemoryOwner final : public MaskedMemory {
public:
  Hcs08 *observer = nullptr;
  bool refuse_retirement = false;
  bool checkpoint_refused = false;

  void retire_instruction() override {
    if (observer != nullptr) {
      checkpoint_refused =
          rejected([&] { (void)observer->checkpoint(); }).contains("completed native boundary");
    }
    if (refuse_retirement) { throw Fault("Own prefetch observer refusal"); }
    Memory::retire_instruction();
  }
};

struct Machine {
  MemoryOwner memory;
  Hcs08 cpu{memory, BLOFELD_HCS08_SLA};

  explicit Machine(const std::span<const std::uint8_t> code = nops) {
    constexpr std::array<std::uint8_t, 2> target{0x90, 0};
    memory.writable.set(); // Own flat RAM bus-cycle fixture, not a board map.
    memory.install(vector, target);
    memory.install(handler, nops);
    memory.install(handler, code);
    cpu.set_state(
        {.pc = entry, .hx = index, .sp = stack, .a = accumulator, .ccr = flags_fixed | 1});
    memory.interrupt = {.vector = vector};
  }

  void interrupt() {
    require(cpu.start_irq_bus(irq_contract), "Only a real pending source admits this IRQ");
    for (unsigned n = 0; n < 11; ++n) { cpu.advance_irq_bus(); }
  }

  void program() {
    interrupt();
    cpu.start_irq_program(program_contract);
  }
};

class NoMemory final : public Hcs08BdcMemory {
public:
  [[nodiscard]] bool secured(ExecutionContext /*context*/) const override { return false; }

  std::uint8_t read(std::uint16_t /*address*/, ExecutionContext /*context*/) override {
    throw Fault("Unexpected BDC read");
  }

  void write(ByteWrite /*access*/, ExecutionContext /*context*/) override {
    throw Fault("Unexpected BDC write");
  }
};

struct Case {
  std::uint8_t opcode;
  unsigned bytes, periods, a, hx, ccr, pushed = 0;
};

constexpr std::array<Case, 21> cases{
    {{.opcode = 0x9d, .bytes = 1, .periods = 1, .a = accumulator, .hx = index, .ccr = flags},
     {.opcode = 0x98, .bytes = 1, .periods = 1, .a = accumulator, .hx = index, .ccr = 0x68},
     {.opcode = 0x99, .bytes = 1, .periods = 1, .a = accumulator, .hx = index, .ccr = flags},
     {.opcode = 0x4f, .bytes = 1, .periods = 1, .a = 0, .hx = index, .ccr = 0x6b},
     {.opcode = 0x5f, .bytes = 1, .periods = 1, .a = accumulator, .hx = 0x1200, .ccr = 0x6b},
     {.opcode = 0x8c, .bytes = 1, .periods = 1, .a = accumulator, .hx = 0x34, .ccr = 0x6b},
     {.opcode = 0x97, .bytes = 1, .periods = 1, .a = accumulator, .hx = 0x125c, .ccr = flags},
     {.opcode = 0x9f, .bytes = 1, .periods = 1, .a = 0x34, .hx = index, .ccr = flags},
     {.opcode = 0x4c, .bytes = 1, .periods = 1, .a = 0x5d, .hx = index, .ccr = flags},
     {.opcode = 0x5c, .bytes = 1, .periods = 1, .a = accumulator, .hx = 0x1235, .ccr = flags},
     {.opcode = 0xa6, .bytes = 2, .periods = 2, .a = 0x42, .hx = index, .ccr = flags},
     {.opcode = 0xae, .bytes = 2, .periods = 2, .a = accumulator, .hx = 0x1242, .ccr = flags},
     {.opcode = 0xa4, .bytes = 2, .periods = 2, .a = 0x40, .hx = index, .ccr = flags},
     {.opcode = 0xa5, .bytes = 2, .periods = 2, .a = accumulator, .hx = index, .ccr = flags},
     {.opcode = 0xa1, .bytes = 2, .periods = 2, .a = accumulator, .hx = index, .ccr = 0x68},
     {.opcode = 0x45, .bytes = 3, .periods = 3, .a = accumulator, .hx = 0x4256, .ccr = flags},
     {.opcode = 0x42, .bytes = 1, .periods = 5, .a = 0xb0, .hx = 0x7712, .ccr = 0x68},
     {.opcode = 0x52, .bytes = 1, .periods = 6, .a = 0x5a, .hx = 0x1434, .ccr = 0x68},
     {.opcode = 0x87,
      .bytes = 1,
      .periods = 2,
      .a = accumulator,
      .hx = index,
      .ccr = flags,
      .pushed = accumulator},
     {.opcode = 0x89,
      .bytes = 1,
      .periods = 2,
      .a = accumulator,
      .hx = index,
      .ccr = flags,
      .pushed = 0x34},
     {.opcode = 0x8b,
      .bytes = 1,
      .periods = 2,
      .a = accumulator,
      .hx = index,
      .ccr = flags,
      .pushed = 0x12}}};

void instruction_case(const Case input, const std::uint8_t known) {
  const std::array<std::uint8_t, 3> code{input.opcode, 0x42, 0x56};
  Machine machine(code);
  auto state = machine.cpu.state();
  state.ccr_known = known;
  if (input.opcode == mul_opcode) { state.hx = mul_hx; }
  machine.cpu.set_state(state);
  machine.program();
  const auto before = machine.cpu.state();
  const auto access_count = machine.memory.accesses.size();
  for (unsigned period = 1; period <= input.periods; ++period) {
    machine.cpu.advance_irq_program();
    const auto q = owned_program(machine.cpu);
    const bool finished = period == input.periods;
    const bool executed = input.pushed != 0 || finished;
    auto expected = before;
    if (executed) {
      expected.pc = static_cast<std::uint16_t>(handler + input.bytes);
      expected.a = static_cast<std::uint8_t>(input.a);
      expected.hx = static_cast<std::uint16_t>(input.hx);
      expected.ccr = static_cast<std::uint8_t>(input.ccr);
      if (input.opcode == mul_opcode) { expected.ccr_known = byte_mask; }
      if (input.pushed != 0) { --expected.sp; }
    }
    const auto retired = static_cast<unsigned>(finished);
    const unsigned fetches = input.bytes > 1 ? period : retired;
    const auto writes = static_cast<unsigned>(input.pushed != 0);
    const auto observed = machine.cpu.state();
    require(observed == expected && q.completed_periods == period &&
                q.attempted_periods == period && q.retired_instructions == retired &&
                machine.memory.retired == retired &&
                machine.memory.accesses.size() == access_count + fetches + writes,
            "Prefetch opcode=" + hex(input.opcode) + " period=" + std::to_string(period) +
                " A=" + hex(observed.a) + " HX=" + hex(observed.hx) + " CCR=" + hex(observed.ccr) +
                " expected-A=" + hex(expected.a) + " expected-HX=" + hex(expected.hx) +
                " expected-CCR=" + hex(expected.ccr));
    if (input.pushed != 0) {
      require(machine.memory.accesses[access_count] ==
                  Access{.address = irq_stack, .value = input.pushed, .write = true},
              "Native push performs its actual s-cycle write before the following program fetch");
    }
    for (unsigned n = 0; n < fetches; ++n) {
      const unsigned address = handler + 3 + n;
      require(
          machine.memory.accesses[access_count + writes + n] ==
              Access{.address = address, .value = machine.memory.bytes[address], .write = false},
          "Only actual sequential p-cycle reads replenish the queue");
    }
  }
}

struct Snapshot {
  MemoryOwner memory;
  Hcs08Checkpoint cpu;
};

constexpr std::array<std::uint8_t, 9> sequence{
    0x8b, 0xa6, 0x11, 0xae, 0x13, 0x42, 0x9d, 0x9d, 0x9d};
constexpr unsigned sequence_periods = 11;

Snapshot cut_sequence(const unsigned cut) {
  Machine original(sequence);
  original.program();
  for (unsigned n = 0; n < cut; ++n) { original.cpu.advance_irq_program(); }
  return {.memory = original.memory, .cpu = original.cpu.checkpoint()};
}

void fresh_continuations() {
  Machine baseline(sequence);
  baseline.program();
  for (unsigned n = 0; n < sequence_periods; ++n) { baseline.cpu.advance_irq_program(); }
  require(baseline.cpu.state().a == product_low && baseline.cpu.state().hx == product_hx &&
              baseline.cpu.state().sp == irq_stack - 1 && baseline.memory.retired == 4,
          "Own PSHH/LDA17/LDX19/MUL yields literal0143 from actual queued instructions");
  for (unsigned cut = 0; cut <= sequence_periods; ++cut) {
    const auto saved = cut_sequence(cut);
    for (unsigned replay = 0; replay < 2; ++replay) {
      Machine fresh;
      fresh.memory = saved.memory;
      fresh.cpu.restore_checkpoint(saved.cpu);
      for (unsigned n = cut; n < sequence_periods; ++n) { fresh.cpu.advance_irq_program(); }
      require(
          fresh.cpu.state() == baseline.cpu.state() &&
              fresh.cpu.prefetch_state() == baseline.cpu.prefetch_state() &&
              fresh.memory.accesses == baseline.memory.accesses &&
              fresh.memory.bytes == baseline.memory.bytes && fresh.memory.retired == 4,
          "Fresh joint memory/CPU restoration resumes every instruction and read-propagation cut");
    }
  }
}

void classification_and_wrap() {
  for (unsigned raw = 0; raw < byte_values; ++raw) {
    if (std::ranges::any_of(cases, [raw](const Case input) { return input.opcode == raw; })) {
      continue;
    }
    const std::array<std::uint8_t, 3> code{static_cast<std::uint8_t>(raw), 0, 0};
    Machine machine(code);
    machine.program();
    const auto before = machine.cpu.state();
    const auto effects = machine.memory.accesses;
    require(
        rejected([&] { machine.cpu.advance_irq_program(); }).contains("unqualified program") &&
            machine.cpu.state() == before && machine.memory.accesses == effects &&
            owned_program(machine.cpu).ready_bytes == 3,
        "All235 other opcode forms refuse before effects while retaining the real byte arrival");
  }
  Machine wrap;
  constexpr std::array<std::uint8_t, 2> target{0xff, 0xff};
  constexpr std::array<std::uint8_t, 1> nop{0x9d};
  wrap.memory.install(vector, target);
  wrap.memory.install(word_mask, nop);
  wrap.memory.install(0, nops);
  wrap.program();
  wrap.cpu.advance_irq_program();
  require(wrap.cpu.state().pc == 0 && owned_program(wrap.cpu).head == 0 &&
              owned_program(wrap.cpu).next_read == 3 && wrap.memory.accesses.back().address == 2,
          "A real one-byte instruction and sequential queue refill wrap the16-bit program bus");
}

void captured_code_and_invalidation() {
  constexpr std::array<std::uint8_t, 4> original{0x8b, 0x9d, 0x9d, 0x9d};
  constexpr std::array<std::uint8_t, 4> changed{0x8d, 0x8d, 0x8d, 0x98};
  Machine machine(original);
  machine.interrupt();
  machine.memory.install(handler, changed);
  machine.cpu.start_irq_program(program_contract);
  constexpr unsigned periods = 5;
  for (unsigned n = 0; n < periods; ++n) { machine.cpu.advance_irq_program(); }
  require(
      machine.cpu.state().pc == handler + 4 && machine.cpu.state().sp == irq_stack - 1 &&
          machine.cpu.state().ccr == carry_clear_flags && machine.memory.retired == 4,
      "Three genuinely prefetched old opcodes execute; the first new physical byte then clears C");
  Machine cached(original);
  cached.interrupt();
  cached.memory.install(handler, changed);
  cached.cpu.start_irq_program(program_contract);
  cached.cpu.advance_irq_program();
  cached.cpu.advance_irq_program();
  auto debug = cached.cpu.state();
  debug.pc = handler;
  cached.cpu.set_state(debug);
  require(rejected([&] { cached.cpu.step(); }).contains("0x9000") && cached.cpu.faulted(),
          "Debug replacement invalidates queue-backed translations before ordinary unknown code");
}

void refusal_and_prefixes() {
  Machine missing;
  require(rejected([&] {
            missing.cpu.start_irq_program(Hcs08PrefetchContract::unqualified);
          }).contains("explicit") &&
              !missing.cpu.faulted(),
          "No implicit queue contract can select successful execution");
  constexpr std::array<std::uint8_t, 3> branch{0x27, 0, 0x9d};
  Machine unsupported(branch);
  unsupported.program();
  const auto before = unsupported.memory.accesses;
  const auto registers = unsupported.cpu.state();
  require(rejected([&] { unsupported.cpu.advance_irq_program(); }).contains("opcode=0x0027") &&
              unsupported.cpu.faulted() && unsupported.cpu.state() == registers &&
              unsupported.memory.accesses == before &&
              owned_program(unsupported.cpu).ready_bytes == 3,
          "Unsupported flow refuses without losing the previously in-flight program byte");
  constexpr std::array<std::uint8_t, 3> push{0x8b, 0x9d, 0x9d};
  Machine failed(push);
  failed.program();
  failed.cpu.advance_irq_program();
  const auto saved_memory = failed.memory;
  const auto saved = failed.cpu.checkpoint();
  failed.memory.known.reset(handler + 3);
  require(rejected([&] { failed.cpu.advance_irq_program(); }).contains("address=0x9003") &&
              failed.cpu.faulted() && failed.cpu.state().sp == irq_stack - 1 &&
              owned_program(failed.cpu).native_executed && failed.memory.retired == 0,
          "A failed replenishment retains the real prior push and incomplete retirement");
  failed.memory = saved_memory;
  failed.cpu.restore_checkpoint(saved);
  failed.cpu.advance_irq_program();
  require(!failed.cpu.faulted() && failed.memory.retired == 1,
          "Coordinated recovery completes the p cycle without repeating the native stack write");
  constexpr std::array<std::uint8_t, 3> mul{0x42, 0x9d, 0x9d};
  Machine partial(mul);
  auto state = partial.cpu.state();
  state.ccr_known = partial_ccr;
  partial.cpu.set_state(state);
  partial.program();
  for (unsigned n = 0; n < 4; ++n) { partial.cpu.advance_irq_program(); }
  require(rejected([&] { partial.cpu.advance_irq_program(); }).contains("partial-CCR") &&
              partial.cpu.faulted() && partial.memory.retired == 0,
          "A qualified cycle shape cannot bypass independently unqualified native flag semantics");
}

void foreign_owners_and_observers() {
  for (unsigned owner = 0; owner < 4; ++owner) {
    Machine machine;
    machine.program();
    const auto before = machine.memory.accesses;
    if (owner == 0) {
      require(rejected([&] { machine.cpu.step(); }).contains("prefetch"),
              "Step cannot steal queue");
    } else if (owner == 1) {
      require(rejected([&] { (void)machine.cpu.service_interrupt(); }).contains("prefetch"),
              "Untimed IRQ cannot steal queue");
    } else if (owner == 2) {
      require(!machine.cpu.reserve_instruction(
                  Hcs08InstructionContract::reference_instruction_retirement),
              "Instruction-retirement clock cannot steal queue");
    } else {
      NoMemory memory;
      Hcs08Bdc bdc(
          machine.cpu, memory, Hcs08FlashContract::candidate_gt8a, Hcs08BdcReset::bkgd_held_low);
      bdc.receive(hcs08_bdc::go);
      require(rejected([&] { bdc.service(); }).contains("unowned instruction/IRQ queue") &&
                  bdc.faulted() && !machine.cpu.faulted(),
              "BDC GO cannot erase program queue ownership or its partial history");
    }
    require(machine.memory.accesses == before && machine.cpu.prefetch_state().has_value(),
            "Foreign owners retain the actual queue without hidden accesses");
  }
  Machine observed;
  observed.program();
  observed.memory.observer = &observed.cpu;
  observed.memory.refuse_retirement = true;
  require(
      rejected([&] { observed.cpu.advance_irq_program(); }).contains("observer refusal") &&
          observed.cpu.faulted() && observed.memory.checkpoint_refused &&
          observed.cpu.state().pc == handler + 1 && owned_program(observed.cpu).native_executed,
      "Refusing retirement retains genuine native effects and blocks reentrant snapshot capture");
}
}

int main() {
  try {
    for (const auto &input : cases) {
      for (const std::uint8_t known : {std::uint8_t{0xff}, std::uint8_t{0xef}}) {
        if (input.opcode == mul_opcode && known != byte_mask) { continue; }
        instruction_case(input, known);
      }
    }
    fresh_continuations();
    classification_and_wrap();
    captured_code_and_invalidation();
    refusal_and_prefixes();
    foreign_owners_and_observers();
    std::cout
        << "HCS08 captured IRQ program:41 independent instruction/mask cases,24 fresh partial "
           "continuations, actual p/f/s cycles and retained native failure prefixes pass.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
