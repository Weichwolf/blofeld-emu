// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_icg_events.hpp"
#include "keyboard/hcs08_icg_history.hpp"
#include "keyboard/hcs08_known_interrupts.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
constexpr unsigned irq_mask = 8, reference_vector = 0xffe0, stack = 0x240;
constexpr unsigned known_except_i = 0xf7, masked_flags = 0x68, enabled_flags = 0x60;
constexpr unsigned reset_on_lock = 0x80, status1 = 0x4a;
constexpr std::array<std::uint8_t, 3> program{0x9a, 0x9d, 0x9d}; // Own CLI;NOP;NOP.

class KnownMemory final : public MaskedMemory, public Hcs08KnownInterrupts {
public:
  Hcs08IcgHistory history{Hcs08IcgHistoryContract::candidate_gt8a_unknown_events};
  mutable std::optional<Hcs08State> queried;
  mutable unsigned queries = 0;

  KnownMemory() {
    writable.set();
    install(entry, program);
  }

  void begin_instruction(const ExecutionContext value) override {
    MaskedMemory::begin_instruction(value);
    history.context(value);
    require(history.action(true) != Hcs08IcgFaultAction::reset,
            "Reset action requires coordinated handling before any native instruction");
  }

  [[nodiscard]] std::optional<InterruptRequest>
  interrupt_request(const std::uint8_t ccr) const override {
    throw Fault("Legacy IRQ query cannot represent uncertain source knowledge ccr=" + hex(ccr));
  }

  [[nodiscard]] std::optional<InterruptRequest>
  interrupt_request(const Hcs08State &state) const override {
    queried = state;
    ++queries;
    auto sample = history;
    sample.context({.pc = state.pc, .ccr = state.ccr});
    const bool masked = (state.ccr_known & irq_mask) != 0 && (state.ccr & irq_mask) != 0;
    switch (sample.action(masked)) {
      case Hcs08IcgFaultAction::none: return {};
      case Hcs08IcgFaultAction::interrupt:
        return InterruptRequest{.vector = reference_vector, .kind = InterruptKind::maskable};
      case Hcs08IcgFaultAction::reset: throw Fault("Known reset requires coordinated owner");
      default: throw Fault("Invalid effective ICG action");
    }
  }
};

Hcs08State initial(const unsigned flags, const unsigned known = byte_mask) {
  return {.pc = entry + 1,
          .sp = stack,
          .ccr_known = static_cast<std::uint8_t>(known),
          .ccr = static_cast<std::uint8_t>(flags)};
}

void uncertain(const unsigned flags, const unsigned known, const bool accepted) {
  KnownMemory memory;
  memory.history.unobserved_interval();
  const auto supplied = initial(flags, known);
  const auto before = initial(known == known_except_i ? enabled_flags : flags, known);
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.set_state(supplied);
  require(cpu.state() == before, "Unknown supplied I has no invented numerical value");
  if (accepted) {
    cpu.step();
    auto expected = before;
    ++expected.pc;
    require(cpu.state() == expected && memory.retired == 1 && !cpu.faulted(),
            "A known masked CPU executes native NOP despite uncertain maskable history");
  } else {
    const auto failure = rejected([&] { cpu.step(); });
    require(failure.contains("unobserved event history affects IRQ/reset selection") &&
                failure.contains("pc=0x8001") && cpu.state() == before && cpu.faulted() &&
                memory.retired == 0 && memory.accesses.empty(),
            "Unmasked or unknown-I ambiguity fails before opcode, stack or vector effects");
  }
  require(memory.queried == before && memory.queries == 1,
          "Native arbitration supplies the actual full CPU state and its original I knowledge");
  memory.history.write({.address = status1, .value = 1});
  require(memory.history.read_known(status1).known == 0,
          "An IRQ query never acknowledges or reads the real fault-latch status");
}

void known_source() {
  KnownMemory memory;
  memory.history.signal(Hcs08IcgFault::loss_of_lock);
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.set_state(initial(enabled_flags, known_except_i));
  const auto failure = rejected([&] { cpu.step(); });
  require(failure.contains("unknown CPU I mask") && memory.accesses.empty() &&
              memory.retired == 0 && cpu.faulted(),
          "A known source retains native CPU rejection of an unknown global I mask");
  KnownMemory quiet;
  Hcs08 other(quiet, BLOFELD_HCS08_SLA);
  const auto before = initial(enabled_flags, known_except_i);
  other.set_state(initial(masked_flags, known_except_i));
  require(other.state() == before, "Absent-source input also retains an unknown canonical I bit");
  other.step();
  auto expected = before;
  ++expected.pc;
  require(other.state() == expected && quiet.retired == 1,
          "With a definitely absent source, native NOP need not invent the unknown I bit");
}

void next_instruction() {
  KnownMemory memory;
  memory.history.unobserved_interval();
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  auto start = initial(masked_flags);
  start.pc = entry;
  cpu.set_state(start);
  cpu.step();
  require(cpu.state().pc == entry + 1 && cpu.state().ccr == enabled_flags && memory.queries == 1 &&
              memory.retired == 1,
          "Native CLI executes with known old I and records its next-instruction guarantee");
  auto restored = memory;
  const auto checkpoint = cpu.checkpoint();
  Hcs08 fresh(restored, BLOFELD_HCS08_SLA);
  fresh.restore_checkpoint(checkpoint);
  cpu.step();
  fresh.step();
  require(cpu.state() == fresh.state() && cpu.state().pc == entry + 2 && memory.queries == 1 &&
              restored.queries == 1 && memory.retired == 2 && restored.retired == 2,
          "Original and fresh copied CPU preserve one protected native NOP without a query");
  const auto first = rejected([&] { cpu.step(); });
  const auto second = rejected([&] { fresh.step(); });
  require(first == second && first.contains("pc=0x8002") &&
              first.contains("unobserved event history") && cpu.state() == fresh.state() &&
              memory.retired == 2 && restored.retired == 2,
          "Both copies fail at the same first unprotected ambiguous source query");
}

void reset_after_cli() {
  KnownMemory memory;
  memory.history.control2(reset_on_lock);
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  auto start = initial(masked_flags);
  start.pc = entry;
  cpu.set_state(start);
  cpu.step();
  memory.history.unobserved_interval();
  const auto before = cpu.state();
  const auto failure = rejected([&] { cpu.step(); });
  require(failure.contains("IRQ/reset selection") && cpu.state() == before && cpu.faulted() &&
              memory.retired == 1 && memory.queries == 1,
          "The CLI maskable-IRQ guarantee cannot skip a board's independent possible-reset guard");
}

void reset_priority() {
  KnownMemory memory;
  memory.history.control2(reset_on_lock);
  memory.history.unobserved_interval();
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.set_state(initial(masked_flags));
  const auto failure = rejected([&] { cpu.step(); });
  require(failure.contains("IRQ/reset selection") && cpu.faulted() && memory.retired == 0,
          "A known set I bit cannot exclude a possible reset");
}
} // namespace

int main() {
  try {
    uncertain(masked_flags, byte_mask, true);
    uncertain(enabled_flags, byte_mask, false);
    uncertain(masked_flags, known_except_i, false);
    uncertain(enabled_flags, known_except_i, false);
    known_source();
    next_instruction();
    reset_priority();
    reset_after_cli();
    std::cout
        << "Verified native CPU IRQ-mask knowledge, strict ambiguity and fresh CLI continuation.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
