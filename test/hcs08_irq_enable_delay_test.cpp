// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_run_clock.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
constexpr unsigned vector = 0xffe0, handler = 0x9000, stack = 0x240, value = 0x42;
constexpr unsigned flags = 0x61, masked_flags = 0x69, unknown_i = 0xf7, hx = 0x1234;
constexpr unsigned cli = 0x9a, tap = 0x84, rti = 0x80, initial_a = 0xa5;
constexpr auto required = Hcs08IrqDelay::next_instruction;
constexpr auto none = Hcs08IrqDelay::none;
constexpr auto uncertain = Hcs08IrqDelay::unknown_transition;
constexpr auto irq_contract = Hcs08IrqBusContract::reference_unstretched_sequence;
constexpr auto instruction_contract = Hcs08InstructionContract::reference_instruction_retirement;
constexpr auto run_contract = Hcs08RunContract::reference_instruction_retirement;
constexpr std::array<std::uint8_t, 4> cli_load{cli, 0xa6, value, 0x9d};
constexpr std::array<std::uint8_t, 3> cli_nop{cli, 0x9d, 0x9d};

class MemoryOwner final : public MaskedMemory {
public:
  mutable unsigned queries = 0;
  Hcs08 *observer = nullptr;
  std::optional<Hcs08IrqDelay> observed;
  bool refuse_retirement = false, checkpoint_blocked = false;

  [[nodiscard]] std::optional<InterruptRequest>
  interrupt_request(std::uint8_t /*ccr*/) const override {
    ++queries;
    return interrupt;
  }

  void retire_instruction() override {
    if (observer != nullptr) {
      observed = observer->irq_delay();
      checkpoint_blocked =
          rejected([&] { (void)observer->checkpoint(); }).contains("completed native boundary");
    }
    if (refuse_retirement) { throw Fault("Own delay retirement refusal"); }
    MaskedMemory::retire_instruction();
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

struct Machine {
  MemoryOwner memory;
  Hcs08 cpu;
  NoMemory mapping;
  std::unique_ptr<Hcs08Bdc> bdc;
  std::unique_ptr<Hcs08RunClock> clock;

  explicit Machine(
      const std::span<const std::uint8_t> code = cli_load,
      const Hcs08RtiIrqContract contract = Hcs08RtiIrqContract::reference_next_instruction)
      : cpu(memory, BLOFELD_HCS08_SLA, contract) {
    constexpr std::array<std::uint8_t, 2> target{0x90, 0};
    constexpr std::array<std::uint8_t, 3> body{rti, 0x9d, 0x9d};
    memory.install(entry, code);
    memory.install(vector, target);
    memory.install(handler, body);
    memory.writable.set(); // Own flat RAM ISA fixture, not a physical board map.
    cpu.set_state({.pc = entry, .hx = hx, .sp = stack, .a = initial_a, .ccr = masked_flags});
    bdc = std::make_unique<Hcs08Bdc>(
        cpu, mapping, Hcs08FlashContract::candidate_gt8a, Hcs08BdcReset::normal);
    clock = std::make_unique<Hcs08RunClock>(memory, *bdc, run_contract);
  }
};

void unchanged_admission(Machine &machine) {
  const auto effects = machine.memory.accesses;
  const auto queries = machine.memory.queries;
  for (unsigned repeat = 0; repeat < 3; ++repeat) {
    require(!machine.cpu.pending_interrupt() && !machine.cpu.service_interrupt() &&
                !machine.bdc->start_irq_bus(irq_contract) && machine.memory.accesses == effects &&
                machine.memory.queries == queries && machine.cpu.irq_delay() == required &&
                !machine.cpu.faulted(),
            "Queries and both IRQ owners cannot consume a required next instruction or its source");
  }
}

void rti_contracts() {
  constexpr auto strict = Hcs08RtiIrqContract::unqualified;
  constexpr auto immediate = Hcs08RtiIrqContract::reference_immediate;
  for (const auto contract : {strict, immediate}) {
    Machine machine(cli_nop, contract);
    auto foreground = machine.cpu.state();
    foreground.ccr = flags;
    foreground.pc += 1;
    machine.cpu.set_state(foreground);
    machine.memory.interrupt = {.vector = vector};
    require(machine.cpu.service_interrupt(), "Actual source supplies the RTI policy test frame");
    machine.cpu.step();
    const auto effects = machine.memory.accesses;
    if (contract == strict) {
      require(machine.cpu.irq_delay() == Hcs08IrqDelay::unqualified_return &&
                  rejected([&] {
                    (void)machine.cpu.pending_interrupt();
                  }).contains("RTI IRQ timing contract") &&
                  machine.cpu.state() == foreground && !machine.cpu.faulted() &&
                  machine.memory.accesses == effects,
              "Default RTI policy refuses a genuine conflicting arbitration without effects or a "
              "guessed answer");
      const auto saved = machine.cpu.checkpoint();
      Machine fresh(cli_nop, strict);
      fresh.memory = machine.memory;
      fresh.cpu.restore_checkpoint(saved);
      require(rejected([&] {
                (void)fresh.cpu.service_interrupt();
              }).contains("RTI IRQ timing contract") &&
                  fresh.cpu.faulted() && fresh.memory.accesses == effects,
              "Trusted restore retains unresolved RTI admission; actual service fails before any "
              "frame byte");
      machine.memory.interrupt.reset();
      machine.cpu.step();
      machine.memory.interrupt = {.vector = vector};
      require(
          machine.cpu.irq_delay() == none && machine.cpu.pending_interrupt() == vector,
          "With no pending source, one real instruction resolves both RTI reference alternatives");
    } else {
      require(machine.cpu.irq_delay() == none && machine.cpu.pending_interrupt() == vector &&
                  machine.cpu.state() == foreground && machine.memory.accesses == effects,
              "Explicit immediate RTI reference retains genuine boundary arbitration without a "
              "source acknowledgement");
    }
  }
  const Machine left(cli_nop, strict);
  Machine right(cli_nop, immediate);
  const auto before = right.cpu.state();
  const auto saved = left.cpu.checkpoint();
  require(
      rejected([&] {
        right.cpu.restore_checkpoint(saved);
      }).contains("reference contract mismatch") &&
          right.cpu.state() == before && !right.cpu.faulted(),
      "A checkpoint cannot silently change the receiving CPU's immutable RTI reference contract");
}

void pending_across_rti() {
  Machine machine(cli_nop);
  auto foreground = machine.cpu.state();
  foreground.ccr = flags;
  foreground.pc += 1;
  machine.cpu.set_state(foreground);
  machine.memory.interrupt = {.vector = vector};
  require(machine.cpu.service_interrupt(), "Actual pending source creates its genuine IRQ frame");
  machine.cpu.step();
  require(machine.cpu.state() == foreground && machine.memory.retired == 1 &&
              machine.cpu.irq_delay() == required,
          "Native RTI restores its real frame and protects its returned foreground instruction");
  unchanged_admission(machine);
  machine.cpu.step();
  require(machine.cpu.state().pc == entry + 2 && machine.cpu.irq_delay() == none &&
              machine.cpu.pending_interrupt() == vector && machine.cpu.start_irq_bus(irq_contract),
          "Completing the real next NOP releases actual eleven-period IRQ admission");
  constexpr unsigned cycles = 11;
  for (unsigned n = 0; n < cycles; ++n) { machine.cpu.advance_irq_bus(); }
  require(machine.cpu.state().pc == handler && machine.memory.bytes[stack] == 2 &&
              machine.memory.bytes[stack - 1] == (entry >> 8U) && machine.memory.retired == 2 &&
              machine.memory.interrupt.has_value(),
          "The next real hardware frame saves the completed foreground PC without clearing source");
}

struct Transition {
  unsigned before, before_known, after, after_known;
  Hcs08IrqDelay expected;
};

constexpr std::array<Transition, 9> transitions{{{.before = masked_flags,
                                                  .before_known = byte_mask,
                                                  .after = flags,
                                                  .after_known = byte_mask,
                                                  .expected = required},
                                                 {.before = flags,
                                                  .before_known = byte_mask,
                                                  .after = flags,
                                                  .after_known = byte_mask,
                                                  .expected = none},
                                                 {.before = masked_flags,
                                                  .before_known = byte_mask,
                                                  .after = masked_flags,
                                                  .after_known = byte_mask,
                                                  .expected = none},
                                                 {.before = flags,
                                                  .before_known = byte_mask,
                                                  .after = masked_flags,
                                                  .after_known = byte_mask,
                                                  .expected = none},
                                                 {.before = flags,
                                                  .before_known = unknown_i,
                                                  .after = flags,
                                                  .after_known = byte_mask,
                                                  .expected = uncertain},
                                                 {.before = flags,
                                                  .before_known = unknown_i,
                                                  .after = masked_flags,
                                                  .after_known = byte_mask,
                                                  .expected = none},
                                                 {.before = masked_flags,
                                                  .before_known = byte_mask,
                                                  .after = flags,
                                                  .after_known = unknown_i,
                                                  .expected = required},
                                                 {.before = flags,
                                                  .before_known = byte_mask,
                                                  .after = flags,
                                                  .after_known = unknown_i,
                                                  .expected = none},
                                                 {.before = flags,
                                                  .before_known = unknown_i,
                                                  .after = flags,
                                                  .after_known = unknown_i,
                                                  .expected = uncertain}}};

Hcs08State prepare(Machine &machine, const unsigned opcode, const Transition transition) {
  const std::array<std::uint8_t, 3> program{static_cast<std::uint8_t>(opcode), 0x9d, 0x9d};
  machine.memory.install(entry, program);
  const Hcs08State before{.pc = entry,
                          .hx = hx,
                          .sp = stack,
                          .a = static_cast<std::uint8_t>(opcode == tap ? transition.after : value),
                          .ccr_known = static_cast<std::uint8_t>(transition.before_known),
                          .ccr = static_cast<std::uint8_t>(transition.before)};
  auto expected = before;
  expected.pc += 1;
  expected.ccr = static_cast<std::uint8_t>(transition.after & transition.after_known);
  expected.ccr_known = static_cast<std::uint8_t>(transition.after_known);
  if (opcode == rti) {
    const std::array<std::uint8_t, 5> frame{
        static_cast<std::uint8_t>(transition.after), value, 0x34, 0x80, 1};
    machine.memory.install(stack + 1, frame);
    machine.memory.write_known_byte({.address = stack + 1, .value = frame[0]},
                                    static_cast<std::uint8_t>(transition.after_known));
    expected.sp += 5;
    expected.a = value;
  }
  machine.cpu.set_state(before);
  return expected;
}

void check_transition(Machine &machine, const Hcs08State &expected, const Hcs08IrqDelay delay) {
  machine.cpu.step();
  require(machine.cpu.state() == expected && machine.cpu.irq_delay() == delay,
          "Independent transition table controls actual native registers, masks and IRQ delay");
  machine.memory.interrupt = {.vector = vector};
  if (delay == required) {
    unchanged_admission(machine);
    machine.cpu.step();
    require(machine.cpu.state().pc == entry + 2 && machine.cpu.irq_delay() == none,
            "A single real NOP releases the proven next-instruction guarantee");
    if ((expected.ccr_known & flag_i) == 0) {
      require(rejected([&] { (void)machine.cpu.pending_interrupt(); }).contains("unknown CPU I"),
              "The guarantee does not define the independently unknown restored I bit");
    } else {
      require(machine.cpu.pending_interrupt() == vector,
              "The known unmasked source is now eligible");
    }
    return;
  }
  if ((expected.ccr_known & flag_i) == 0 || delay == uncertain) {
    require(rejected([&] { (void)machine.cpu.pending_interrupt(); }).contains("unknown") &&
                !machine.cpu.faulted(),
            "An unresolved I bit or enable transition cannot grant a guessed IRQ eligibility");
  } else {
    require(machine.cpu.pending_interrupt().has_value() == ((expected.ccr & flag_i) == 0),
            "No1->0 transition adds no spurious delay; the actual current I bit still masks IRQs");
  }
  machine.memory.interrupt.reset();
  machine.cpu.step();
  require(
      machine.cpu.irq_delay() == none && machine.cpu.state().ccr_known == expected.ccr_known,
      "With no local request, a completed instruction resolves delay without defining unknown I");
}

void transition_table() {
  for (const auto opcode : {cli, tap, rti}) {
    for (const auto transition : transitions) {
      if (opcode != rti && transition.after_known != byte_mask) { continue; }
      if (opcode == cli && transition.after != flags) { continue; }
      Machine machine;
      const auto expected = prepare(machine, opcode, transition);
      check_transition(machine, expected, transition.expected);
    }
  }
}

void unrelated_flags() {
  for (const auto opcode : {cli, tap, rti}) {
    Machine machine;
    for (unsigned raw = 0; raw < byte_values; ++raw) {
      if ((raw & (flags_fixed | flag_i)) != (flags_fixed | flag_i)) { continue; }
      const Transition transition{.before = raw,
                                  .before_known = byte_mask,
                                  .after = raw & ~flag_i,
                                  .after_known = byte_mask,
                                  .expected = required};
      const auto expected = prepare(machine, opcode, transition);
      machine.memory.interrupt = {.vector = vector};
      check_transition(machine, expected, required);
      machine.memory.interrupt.reset();
    }
  }
}

struct Snapshot {
  MemoryOwner memory;
  Hcs08Checkpoint cpu;
  Hcs08BdcCheckpoint bdc;
  Hcs08RunCheckpoint clock;
};

Snapshot protected_budget(const unsigned cut) {
  Machine original;
  original.memory.interrupt = {.vector = vector};
  original.cpu.step(); // Actual CLI, with a previously masked pending source.
  for (unsigned n = 0; n < cut; ++n) { original.clock->advance(); }
  require(original.cpu.irq_delay() == (cut < 2 ? required : none),
          "Only completing the protected two-period LDA consumes the delay");
  return {.memory = original.memory,
          .cpu = original.cpu.checkpoint(),
          .bdc = original.bdc->checkpoint(),
          .clock = original.clock->checkpoint()};
}

void snapshots_and_halted_time() {
  for (unsigned cut = 0; cut <= 2; ++cut) {
    const auto saved = protected_budget(cut);
    for (unsigned replay = 0; replay < 2; ++replay) {
      Machine restored;
      restored.memory = saved.memory;
      restored.cpu.restore_checkpoint(saved.cpu);
      restored.bdc->restore_checkpoint(saved.bdc);
      restored.clock->restore_checkpoint(saved.clock);
      for (unsigned n = cut; n < 2; ++n) { restored.clock->advance(); }
      require(restored.cpu.state().pc == entry + 3 && restored.cpu.state().a == value &&
                  restored.cpu.irq_delay() == none && restored.cpu.pending_interrupt() == vector &&
                  restored.memory.retired == 2,
              "Fresh graph resumes exact pre/mid/post protected instruction after original "
              "destruction");
    }
  }
  Machine halted;
  halted.memory.interrupt = {.vector = vector};
  halted.cpu.step();
  Hcs08Bdc held(
      halted.cpu, halted.mapping, Hcs08FlashContract::candidate_gt8a, Hcs08BdcReset::bkgd_held_low);
  Hcs08RunClock clock(halted.memory, held, run_contract);
  constexpr unsigned held_periods = 7;
  for (unsigned n = 0; n < held_periods; ++n) { clock.advance(); }
  require(
      halted.cpu.irq_delay() == required && clock.state().executing_periods == 0 &&
          halted.memory.retired == 1 && !held.service_interrupt(),
      "Elapsed BDC-held periods and repeated IRQ attempts cannot substitute for an instruction");
  held.receive(hcs08_bdc::go);
  held.service();
  clock.advance();
  require(halted.cpu.irq_delay() == required, "A half-budget does not release the protected LDA");
  clock.advance();
  require(halted.cpu.irq_delay() == none && halted.cpu.pending_interrupt() == vector,
          "BDC GO resumes the actual protected instruction before a still-pending source");
}

void incomplete_and_unknown() {
  constexpr std::array<std::uint8_t, 5> missing{cli, 0xc6, 0, operand, 0x9d};
  Machine machine(missing);
  machine.memory.interrupt = {.vector = vector};
  machine.cpu.step();
  const auto before = machine.cpu.checkpoint();
  const auto reservation = machine.cpu.reserve_instruction(instruction_contract);
  if (!reservation) {
    throw Fault("A protected next instruction must be admitted despite pending source");
  }
  const auto &token = *reservation;
  require(rejected([&] {
            machine.cpu.retire_reserved_instruction(token);
          }).contains("address=0x0040") &&
              machine.cpu.faulted() && machine.cpu.irq_delay() == required &&
              machine.cpu.instruction_reserved() && machine.memory.retired == 1,
          "Native data failure retains both incomplete ownership and the unconsumed delay");
  machine.cpu.restore_checkpoint(before);
  constexpr std::array<std::uint8_t, 1> data{value};
  machine.memory.install(operand, data);
  machine.cpu.step();
  require(machine.cpu.irq_delay() == none && machine.cpu.pending_interrupt() == vector,
          "Explicit recovery plus real data permits genuine instruction completion");
  Machine unknown;
  auto state = unknown.cpu.state();
  state.ccr_known = unknown_i;
  unknown.cpu.set_state(state);
  unknown.cpu.step();
  require(unknown.cpu.irq_delay() == uncertain,
          "CLI cannot guess whether an unknown old I was set");
  const auto saved = unknown.cpu.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    Machine restored;
    restored.memory = unknown.memory;
    restored.cpu.restore_checkpoint(saved);
    restored.memory.interrupt = {.vector = vector};
    require(restored.cpu.irq_delay() == uncertain && rejected([&] {
                                                       (void)restored.cpu.pending_interrupt();
                                                     }).contains("unknown I-mask"),
            "A fresh trusted restore retains unresolved eligibility without a guessed IRQ");
  }
  unknown.memory.interrupt = {.vector = vector};
  const auto effects = unknown.memory.accesses;
  require(rejected([&] { (void)unknown.cpu.service_interrupt(); }).contains("unknown I-mask") &&
              unknown.cpu.faulted() && unknown.memory.accesses == effects &&
              unknown.cpu.irq_delay() == uncertain,
          "Unknown-transition IRQ service fails before any frame byte with actual context");
}

void go_cannot_discard_owned_work() {
  for (const bool instruction : {false, true}) {
    Machine machine;
    if (instruction) {
      machine.cpu.step();
      const auto token = machine.cpu.reserve_instruction(instruction_contract);
      if (!token) { throw Fault("Expected genuine protected instruction reservation"); }
    } else {
      auto state = machine.cpu.state();
      state.ccr = flags;
      machine.cpu.set_state(state);
      machine.memory.interrupt = {.vector = vector};
      require(machine.cpu.start_irq_bus(irq_contract), "Real local source owns IRQ admission");
    }
    Hcs08Bdc held(machine.cpu,
                  machine.mapping,
                  Hcs08FlashContract::candidate_gt8a,
                  Hcs08BdcReset::bkgd_held_low);
    const auto effects = machine.memory.accesses;
    const auto before = machine.cpu.state();
    held.receive(hcs08_bdc::go);
    require(rejected([&] { held.service(); }).contains("unowned instruction/IRQ queue") &&
                held.faulted() && !machine.cpu.faulted() && machine.cpu.state() == before &&
                machine.memory.accesses == effects &&
                machine.cpu.instruction_reserved() == instruction &&
                machine.cpu.irq_bus_state().has_value() != instruction,
            "GO cannot silently drop another owner's accepted instruction or captured IRQ queue");
  }
}

void observers_and_debug() {
  for (const unsigned observed_instruction : {0U, 1U}) {
    Machine machine(cli_nop);
    machine.memory.interrupt = {.vector = vector};
    if (observed_instruction != 0) { machine.cpu.step(); }
    machine.memory.observer = &machine.cpu;
    machine.memory.refuse_retirement = true;
    require(rejected([&] { machine.cpu.step(); }).contains("Own delay retirement refusal") &&
                machine.cpu.faulted() && machine.memory.checkpoint_blocked &&
                machine.cpu.irq_delay() == (observed_instruction == 0 ? required : none) &&
                machine.memory.observed == machine.cpu.irq_delay(),
            "Observers see complete enable-delay effects even when retirement notification fails");
  }
  constexpr std::array<std::uint8_t, 3> repeated{cli, cli, 0x9d};
  Machine machine(repeated);
  machine.memory.interrupt = {.vector = vector};
  machine.cpu.step();
  machine.cpu.step();
  require(machine.cpu.irq_delay() == none && machine.cpu.pending_interrupt() == vector,
          "A second CLI with I already zero does not extend an earlier one-instruction guarantee");
  Machine debugger;
  debugger.cpu.step();
  debugger.cpu.set_state(debugger.cpu.state());
  require(debugger.cpu.irq_delay() == none,
          "Explicit debugger boundary replacement discards previous execution history");
}
}

int main() {
  try {
    rti_contracts();
    pending_across_rti();
    transition_table();
    unrelated_flags();
    snapshots_and_halted_time();
    incomplete_and_unknown();
    go_cannot_discard_owned_work();
    observers_and_debug();
    std::cout << "HCS08 CLI/TAP/RTI next-instruction delay and trusted recovery pass.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
