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
constexpr unsigned value = 0x42, initial_a = 0xa5, stack = 0x240, irq_vector = 0xffe0;
constexpr unsigned initial_ccr = 0x61, initial_hx = 0x1234, handler = 0x9000;
constexpr unsigned stored_a = 0x5a, canary = 0xaa, loaded_mask = 0xee;
constexpr unsigned high_byte_known = 0xff00, known_masked_ccr = 0x68;
constexpr auto admission = Hcs08InstructionContract::reference_instruction_retirement;
constexpr auto run_contract = Hcs08RunContract::reference_instruction_retirement;
constexpr auto irq_contract = Hcs08IrqBusContract::reference_unstretched_sequence;
constexpr Hcs08State initial{
    .pc = entry, .hx = initial_hx, .sp = stack, .a = initial_a, .ccr = initial_ccr};
constexpr std::array<std::uint8_t, 3> load_code{0xa6, value, 0x9d};
constexpr std::array<std::uint8_t, 3> nop_code{0x9d, 0x9d, 0x9d};

class TrackingMemory final : public Memory {
public:
  mutable unsigned queries = 0, fetches = 0;
  mutable bool signal_pending = false;
  bool signal_on_fetch = false, refuse_retirement = false;
  Hcs08 *observer = nullptr;
  std::optional<Hcs08State> observed;
  bool observed_reserved = true, checkpoint_blocked = false;
  unsigned entries = 0, acknowledgements = 0;

  [[nodiscard]] std::optional<InterruptRequest>
  interrupt_request(std::uint8_t /*ccr*/) const override {
    ++queries;
    if (signal_pending) { return InterruptRequest{.vector = irq_vector}; }
    return interrupt;
  }

  void check_execute(const InstructionFetch fetch) const override {
    Memory::check_execute(fetch);
    ++fetches;
    if (signal_on_fetch) { signal_pending = true; }
  }

  void begin_interrupt(const InterruptRequest request, const bool reuse) override {
    require(request.vector == irq_vector && !reuse && request.kind == InterruptKind::maskable,
            "Only actual maskable source admission creates this frame");
    ++entries;
  }

  void acknowledge_interrupt(const std::uint16_t vector) override {
    require(vector == irq_vector, "Acknowledge the actual source without clearing its request");
    ++acknowledgements;
  }

  void retire_instruction() override {
    if (observer != nullptr) {
      observed = observer->state();
      observed_reserved = observer->instruction_reserved();
      checkpoint_blocked =
          rejected([&] { (void)observer->checkpoint(); }).contains("completed native boundary");
    }
    if (refuse_retirement) { throw Fault("Own reserved-retirement observer refusal"); }
    Memory::retire_instruction();
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
  TrackingMemory memory;
  Hcs08 cpu{memory, BLOFELD_HCS08_SLA};
  NoMemory mapping;
  std::unique_ptr<Hcs08Bdc> bdc;
  std::unique_ptr<Hcs08RunClock> clock;

  explicit Machine(const std::span<const std::uint8_t> code = load_code,
                   const Hcs08State state = initial) {
    memory.install(entry, code);
    constexpr std::array<std::uint8_t, 2> vector{0x90, 0};
    memory.install(irq_vector, vector);
    memory.install(handler, nop_code);
    memory.writable.set(); // Own flat-RAM ISA fixture, not a physical peripheral map.
    cpu.set_state(state);
    bdc = std::make_unique<Hcs08Bdc>(
        cpu, mapping, Hcs08FlashContract::candidate_gt8a, Hcs08BdcReset::normal);
    clock = std::make_unique<Hcs08RunClock>(memory, *bdc, run_contract);
  }
};

Hcs08InstructionReservation reserve(Hcs08 &cpu) {
  const auto token = cpu.reserve_instruction(admission);
  if (!token) { throw Fault("Expected an accepted native instruction"); }
  return *token;
}

void admission_ownership() {
  Machine machine;
  require(rejected([&] {
            (void)machine.cpu.reserve_instruction(Hcs08InstructionContract::unqualified);
          }).contains("explicit retirement") &&
              rejected([&] {
                (void)machine.bdc->reserve_instruction(Hcs08InstructionContract::unqualified);
              }).contains("explicit retirement") &&
              !machine.cpu.faulted() && !machine.bdc->faulted() && machine.memory.queries == 0 &&
              machine.memory.fetches == 0,
          "No implicit instruction contract grants an admission or damages a healthy owner");
  const auto token = reserve(machine.cpu);
  const auto saved = machine.cpu.checkpoint();
  require(!machine.cpu.reserve_instruction(admission) && machine.memory.queries == 1 &&
              rejected([&] { machine.cpu.step(); }).contains("reservation still has an owner") &&
              machine.cpu.faulted() && machine.memory.fetches == 0 && machine.memory.retired == 0 &&
              machine.cpu.instruction_reserved(),
          "A direct step cannot steal a claim; repeated admission has no extra source query");
  machine.cpu.restore_checkpoint(saved);
  machine.cpu.retire_reserved_instruction(token);
  require(machine.cpu.state().pc == entry + 2 && machine.cpu.state().a == value &&
              machine.memory.retired == 1 && !machine.cpu.instruction_reserved(),
          "Trusted CPU recovery retains the real accepted instruction and only its own ticket");
  Machine halted;
  Hcs08Bdc held(
      halted.cpu, halted.mapping, Hcs08FlashContract::candidate_gt8a, Hcs08BdcReset::bkgd_held_low);
  require(!held.reserve_instruction(admission) && halted.memory.queries == 0 &&
              halted.memory.fetches == 0 && !halted.cpu.instruction_reserved(),
          "An actually held BDC rejects admission before consulting the CPU or local source");
}

void late_irq() {
  Machine machine;
  machine.clock->advance();
  require(machine.cpu.state() == initial && machine.cpu.instruction_reserved() &&
              machine.clock->state().remaining_periods == 1 && machine.memory.queries == 1,
          "A two-period LDA is accepted once and has no first-period architectural effects");
  machine.memory.interrupt = {.vector = irq_vector};
  require(!machine.cpu.pending_interrupt() && !machine.cpu.service_interrupt() &&
              !machine.cpu.start_irq_bus(irq_contract) &&
              !machine.cpu.reserve_instruction(admission) && machine.memory.queries == 1 &&
              machine.memory.accesses.empty() && !machine.cpu.faulted(),
          "An arriving local source cannot steal the already accepted instruction or its state");
  machine.clock->advance();
  auto after = initial;
  after.pc += 2;
  after.a = value;
  require(machine.cpu.state() == after && !machine.cpu.instruction_reserved() &&
              machine.memory.retired == 1 && machine.memory.queries == 1 &&
              machine.clock->state().retired_instructions == 1 &&
              machine.clock->state().executing_periods == 2,
          "The accepted LDA completes before any newly pending IRQ query");
  const auto fetches = machine.memory.fetches;
  require(rejected([&] { machine.clock->advance(); }).contains("before fetch") &&
              machine.memory.fetches == fetches && machine.clock->state().offered_periods == 2 &&
              !machine.cpu.faulted() && !machine.bdc->faulted() &&
              !machine.cpu.instruction_reserved(),
          "Next-boundary IRQ admission refuses another opcode without charging its period");
  require(machine.bdc->start_irq_bus(irq_contract),
          "The actual pending source can now own IRQ entry");
  constexpr unsigned cycles = 11;
  for (unsigned n = 0; n < cycles; ++n) { machine.bdc->advance_irq_bus(); }
  const auto &ram = machine.memory.bytes;
  require(ram[stack] == 2 && ram[stack - 1] == (entry >> 8U) &&
              ram[stack - 2] == (initial_hx & byte_mask) && ram[stack - 3] == value &&
              ram[stack - 4] == initial_ccr && machine.cpu.state().pc == handler &&
              machine.cpu.state().sp == stack - 5 && machine.memory.entries == 1 &&
              machine.memory.acknowledgements == 1 && machine.memory.retired == 1 &&
              machine.memory.interrupt.has_value(),
          "Real11-period IRQ frame saves the completed instruction's genuine PC and result");
}

void irq_positions_and_masks() {
  constexpr std::array<std::uint8_t, 4> store_code{0xc7, 0, operand, 0x9d};
  auto state = initial;
  state.a = stored_a;
  for (unsigned arrival = 1; arrival < 4; ++arrival) {
    Machine machine(store_code, state);
    constexpr std::array<std::uint8_t, 1> data{canary};
    machine.memory.install(operand, data);
    for (unsigned period = 1; period <= 4; ++period) {
      if (period > arrival) { machine.memory.interrupt = {.vector = irq_vector}; }
      machine.clock->advance();
      require(
          machine.memory.bytes[operand] == (period < 4 ? canary : stored_a) &&
              machine.cpu.state().pc == (period < 4 ? entry : entry + 3) &&
              machine.memory.retired == (period < 4 ? 0U : 1U),
          "Each possible mid-STA IRQ position retains the independent four-period store boundary");
    }
    require(machine.memory.queries == 1, "An accepted store is never resampled as an IRQ");
  }
  Machine immediate(nop_code);
  immediate.memory.signal_on_fetch = true;
  immediate.clock->advance();
  require(immediate.cpu.state().pc == entry + 1 && immediate.memory.retired == 1 &&
              immediate.memory.queries == 1 && immediate.memory.signal_pending,
          "A source raised after NOP admission cannot replace its one-period execution");
  Machine present;
  present.memory.interrupt = {.vector = irq_vector};
  require(rejected([&] { present.clock->advance(); }).contains("before fetch") &&
              present.memory.fetches == 0 && present.memory.accesses.empty() &&
              present.clock->state().offered_periods == 0 && !present.cpu.faulted(),
          "An IRQ already eligible at the boundary precedes all opcode timing/fetch work");
  auto masked = initial;
  masked.ccr |= flag_i;
  Machine blocked(load_code, masked);
  blocked.memory.interrupt = {.vector = irq_vector};
  blocked.clock->advance();
  blocked.clock->advance();
  require(blocked.cpu.state().pc == entry + 2 && blocked.cpu.state().a == value &&
              (blocked.cpu.state().ccr & flag_i) != 0 && !blocked.cpu.pending_interrupt(),
          "A genuinely masked source does not block an otherwise accepted instruction");
}

void withdrawal_and_knowledge() {
  Machine machine;
  machine.clock->advance();
  machine.memory.interrupt = {.vector = irq_vector};
  require(!machine.cpu.pending_interrupt(), "An in-flight instruction still owns the boundary");
  machine.memory.interrupt.reset();
  machine.clock->advance();
  machine.clock->advance();
  require(machine.cpu.state().pc == entry + 3 && machine.memory.retired == 2 &&
              machine.memory.queries == 2 && !machine.cpu.irq_bus_state(),
          "The CPU does not manufacture a remembered source that is no longer locally pending");
  auto partial = initial;
  partial.a_known = 0;
  partial.hx_known = high_byte_known;
  partial.ccr_known = known_masked_ccr;
  partial.ccr = known_masked_ccr;
  Machine unknown(load_code, partial);
  unknown.clock->advance();
  unknown.clock->advance();
  constexpr Hcs08State expected{.pc = entry + 2,
                                .hx = 0x1200,
                                .sp = stack,
                                .a = value,
                                .ccr_known = loaded_mask,
                                .ccr = 0x68,
                                .hx_known = 0xff00};
  require(unknown.cpu.state() == expected && !unknown.cpu.instruction_reserved(),
          "Reserved native LDA establishes A/VNZ while retaining independently unknown X/H/C bits");
}

void stale_and_cross_cpu() {
  Machine first(nop_code);
  Machine second(nop_code);
  const auto a = reserve(first.cpu);
  const auto b = reserve(second.cpu);
  const auto same = first.cpu.state();
  require(same == second.cpu.state() && a.matches_context(same, second.memory.code_revision()) &&
              !second.cpu.reservation_matches(a) && second.cpu.reservation_matches(b),
          "Equal registers, code revision and admission order do not grant another CPU's token");
  const auto saved = second.cpu.checkpoint();
  const auto fetches = second.memory.fetches;
  require(
      rejected([&] { second.cpu.retire_reserved_instruction(a); }).contains("owner/state/code") &&
          second.cpu.faulted() && second.memory.retired == 0 && second.memory.fetches == fetches,
      "Wrong-owner execution fails before code or data effects and cannot consume the actual "
      "claim");
  second.cpu.restore_checkpoint(saved);
  second.cpu.retire_reserved_instruction(b);
  require(second.cpu.state().pc == entry + 1 && second.memory.retired == 1,
          "The genuine captured owner survives trusted recovery");
  second.cpu.set_state(initial);
  require(!second.cpu.reservation_matches(b) && rejected([&] {
                                                  second.cpu.retire_reserved_instruction(b);
                                                }).contains("owner/state/code") &&
              second.memory.retired == 1,
          "Restoring identical debug registers never revives a consumed instruction claim");
}

void future_epochs() {
  Machine parent(nop_code);
  const auto first = reserve(parent.cpu);
  const auto active = parent.cpu.checkpoint();
  const auto memory = parent.memory;
  parent.cpu.retire_reserved_instruction(first);
  parent.cpu.set_state(initial);
  const auto old_future = reserve(parent.cpu);
  parent.cpu.restore_checkpoint(active);
  parent.cpu.retire_reserved_instruction(first);
  parent.cpu.set_state(initial);
  const auto new_future = reserve(parent.cpu);
  require(old_future.matches_context(parent.cpu.state(), parent.memory.code_revision()) &&
              old_future != new_future && !parent.cpu.reservation_matches(old_future) &&
              parent.cpu.reservation_matches(new_future),
          "Same-CPU rewind resumes the captured claim but never reuses a later admission identity");
  Machine fork(nop_code);
  fork.memory = memory;
  fork.cpu.restore_checkpoint(active);
  fork.cpu.retire_reserved_instruction(first);
  fork.cpu.set_state(initial);
  const auto fork_future = reserve(fork.cpu);
  require(first.matches_context(fork.cpu.state(), fork.memory.code_revision()) &&
              fork_future != first && !fork.cpu.reservation_matches(first) &&
              fork.cpu.reservation_matches(fork_future),
          "A fresh restored CPU resumes the old claim, then uses its own lifetime identity for "
          "future work");
}

struct Snapshot {
  TrackingMemory memory;
  Hcs08Checkpoint cpu;
  Hcs08BdcCheckpoint bdc;
  Hcs08RunCheckpoint clock;
};

Snapshot accepted_snapshot() {
  Machine original;
  original.clock->advance();
  original.memory.interrupt = {.vector = irq_vector};
  return {.memory = original.memory,
          .cpu = original.cpu.checkpoint(),
          .bdc = original.bdc->checkpoint(),
          .clock = original.clock->checkpoint()};
}

void copies_and_pairing() {
  const auto saved = accepted_snapshot(); // Original CPU is destroyed; no peer pointer may remain.
  for (unsigned replay = 0; replay < 2; ++replay) {
    Machine restored;
    restored.memory = saved.memory;
    restored.cpu.restore_checkpoint(saved.cpu);
    restored.bdc->restore_checkpoint(saved.bdc);
    restored.clock->restore_checkpoint(saved.clock);
    restored.clock->advance();
    auto expected = initial;
    expected.pc += 2;
    expected.a = value;
    require(restored.cpu.state() == expected && restored.memory.retired == 1 &&
                restored.memory.queries == 1 && !restored.cpu.instruction_reserved(),
            "Fresh complete graph resumes an accepted instruction after the original CPU lifetime "
            "ends");
  }
  Machine mismatched;
  mismatched.memory = saved.memory;
  mismatched.memory.interrupt.reset();
  const auto correct = reserve(mismatched.cpu); // Same scalar context, unrelated opaque claim.
  require(rejected([&] {
            mismatched.clock->restore_checkpoint(saved.clock);
          }).contains("changed during") &&
              mismatched.clock->faulted() && mismatched.cpu.reservation_matches(correct),
          "An unrelated CPU/clock snapshot pair cannot consume a valid other owner's claim");
}

void interference_and_faults() {
  for (unsigned change = 0; change < 3; ++change) {
    Machine machine;
    machine.clock->advance();
    if (change == 0) {
      machine.cpu.set_state(machine.cpu.state());
    } else if (change == 1) {
      machine.memory.install(entry, load_code);
    } else {
      auto altered = machine.cpu.state();
      altered.a = 0;
      machine.cpu.set_state(altered);
    }
    require(rejected([&] { machine.clock->advance(); }).contains("changed during") &&
                machine.clock->faulted() && machine.memory.retired == 0 &&
                machine.cpu.state().pc == entry,
            "Identical debug replacement, lost code continuity and real register mutation all "
            "invalidate a budget");
  }
  Machine owned;
  owned.clock->advance();
  Hcs08RunClock other(owned.memory, *owned.bdc, run_contract);
  require(rejected([&] { other.advance(); }).contains("ownership changed") &&
              other.state().offered_periods == 0 && !owned.cpu.faulted(),
          "A second clock cannot steal or damage an already accepted instruction");
  owned.clock->advance();
  require(owned.cpu.state().a == value && owned.memory.retired == 1,
          "The original clock still completes its genuine instruction");
  constexpr std::array<std::uint8_t, 3> read_unknown{0xc6, 0, operand};
  Machine missing(read_unknown);
  const auto ticket = reserve(missing.cpu);
  require(rejected([&] {
            missing.cpu.retire_reserved_instruction(ticket);
          }).contains("address=0x0040") &&
              missing.cpu.faulted() && missing.cpu.instruction_reserved() &&
              missing.memory.retired == 0,
          "A genuine native data fault retains the incomplete claim and prevents a silent retry");
}

void retirement_observers() {
  for (const bool refuse : {false, true}) {
    Machine machine;
    machine.memory.observer = &machine.cpu;
    machine.memory.refuse_retirement = refuse;
    machine.clock->advance();
    if (refuse) {
      require(
          rejected([&] { machine.clock->advance(); }).contains("reserved-retirement observer") &&
              machine.cpu.faulted() && machine.clock->faulted() && machine.memory.retired == 0,
          "Observer failure retains native results and quarantines without another admission");
    } else {
      machine.clock->advance();
    }
    auto expected = initial;
    expected.pc += 2;
    expected.a = value;
    require(machine.cpu.state() == expected && machine.memory.observed == expected &&
                !machine.memory.observed_reserved && machine.memory.checkpoint_blocked &&
                !machine.cpu.instruction_reserved(),
            "Observers see complete native values/masks and consumed ownership while snapshots "
            "remain blocked");
  }
}
}

int main() {
  try {
    admission_ownership();
    late_irq();
    irq_positions_and_masks();
    withdrawal_and_knowledge();
    stale_and_cross_cpu();
    future_epochs();
    copies_and_pairing();
    interference_and_faults();
    retirement_observers();
    std::cout
        << "HCS08 accepted instruction identities, late IRQs and joint budget restores pass.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
