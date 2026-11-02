// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_interrupts.hpp"
#include "dsp/own_dsp_interrupts.hpp"
#include "dsp/own_dsp_io.hpp"
#include "dsp56300/core.hpp"

#include <exception>
#include <iostream>
#include <stdexcept>

namespace {
using dsp56300::Word;
constexpr Word irqa_vector = 0x10;
constexpr Word priority_register = 0xffffff;
constexpr Word foreground_address = 0x300;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

dsp56300::InstructionProgress vector_word(dsp56300::Core &cpu,
                                          blofeld::OwnDspInterrupts &owner,
                                          blofeld::DspBus &bus,
                                          const Word address,
                                          const bool first) {
  using namespace dsp56300;
  require(cpu.instruction_fetch().address == address, "Wrong native interrupt vector address");
  const auto before = cpu.registers();
  const auto instructions = cpu.progress().instructions;
  if (!first) {
    require(cpu.instruction_fetch().exception_vector && cpu.instruction_fetch().latched_opcode == 0,
            "Second vector slot must retain the accepted zero opcode");
  }
  auto result = cpu.begin_fetch(InstructionPublication::pcu_updates);
  unsigned fetched = 0;
  for (unsigned effects = 0; effects < 16 && result.pending && !result.retired; ++effects) {
    const auto ticket = result.pending.value().ticket;
    const auto effect = result.pending.value().effect;
    result = owner.advance(ticket);
    if (effect.kind == EffectKind::fetch_opcode || effect.kind == EffectKind::fetch_extension) {
      require(first && effect.address == address + fetched && effect.pc == address &&
                  effect.kind ==
                      (fetched == 0 ? EffectKind::fetch_opcode : EffectKind::fetch_extension),
              "The first native instruction reads both actual vector words in order");
      if (result.fault.kind != FaultKind::none) { return result; }
      ++fetched;
      const bool held = fetched < 2;
      require(owner.pending() == held && bus.interrupt_request(0).has_value() == held &&
                  cpu.registers() == before && cpu.progress().instructions == instructions,
              "Only the second successful vector read acknowledges; neither read retires a slot");
      bool rejected = false;
      try {
        static_cast<void>(owner.advance(ticket));
      } catch (const std::invalid_argument &) { rejected = true; }
      require(rejected && owner.pending() == held && bus.interrupt_request(0).has_value() == held,
              "A stale read ticket cannot consume or repeat peripheral acknowledgement");
    }
  }
  require(result.retired && result.fault.kind == FaultKind::none && fetched == (first ? 2U : 0U),
          "First vector slot captures the pair; second slot retires without another program read");
  return result;
}

void check_vector(const Word base, const bool missing_second) {
  using namespace blofeld;
  using namespace dsp56300;
  DspBus bus;
  OwnDspIo io(bus);
  Core cpu(io, {.backend = Backend::reference});
  OwnDspInterrupts owner(bus, cpu);
  // Independently authored NOPs, not manufacturer program bytes.
  bus.write({.address = base + irqa_vector, .value = 0});
  if (!missing_second) { bus.write({.address = base + irqa_vector + 1, .value = 0}); }
  bus.write({.address = foreground_address, .value = 0});
  auto regs = cpu.registers();
  regs.pc = foreground_address;
  regs.vba = base;
  constexpr Word masked_status = 0x300;
  regs.sr = masked_status;
  cpu.set_registers(regs);
  bus.observe_irqa(true);
  bus.write({.space = DspSpace::x, .address = priority_register, .value = 6});
  bus.observe_irqa(false);
  require(bus.interrupt_request(0) == DspInterruptRequest{.vector = irqa_vector, .level = 1},
          "Expected actual latched IRQA edge");
  const InstructionWords foreground{.address = foreground_address, .opcode = 0};
  require(!owner.admit(foreground), "Masked interrupt was admitted");
  regs.sr = 0;
  cpu.set_registers(regs);
  require(owner.admit(foreground) && owner.pending(), "Unmasked interrupt was not admitted");
  const auto before = cpu.registers();
  const auto result = vector_word(cpu, owner, bus, base + irqa_vector, true);
  if (missing_second) {
    require(result.fault.kind == FaultKind::memory_read && result.fault.space == Space::program &&
                result.fault.address == base + irqa_vector + 1 && io.failure(),
            "Unknown second vector word did not report the actual failed program fetch");
    require(owner.pending() && bus.interrupt_request(0) && cpu.registers() == before &&
                cpu.progress().instructions == 0,
            "A refused second vector read neither acknowledges nor executes the first slot");
  } else {
    static_cast<void>(vector_word(cpu, owner, bus, base + irqa_vector + 1, false));
    require(cpu.registers() == before && !owner.pending() &&
                cpu.progress() == Progress{.instructions = 2, .estimated_cycles = 2},
            "Two actual fast NOP slots preserve full foreground registers and literal base costs");
  }
}

void no_irq_guards() {
  using namespace blofeld;
  using namespace dsp56300;
  DspBus bus;
  DspBus alien_bus;
  OwnDspIo io(bus);
  OwnDspIo alien_io(alien_bus);
  Core cpu(io, {.backend = Backend::reference});
  Core alien(alien_io, {.backend = Backend::reference});
  OwnDspInterrupts owner(bus, cpu);
  bus.write({.address = foreground_address, .value = 0});
  alien_bus.write({.address = foreground_address, .value = 0});
  const Registers initial{.pc = foreground_address};
  cpu.set_registers(initial);
  alien.set_registers(initial);
  auto result = cpu.begin_fetch(InstructionPublication::pcu_updates);
  const auto alien_result = alien.begin_fetch(InstructionPublication::pcu_updates);
  if (!result.pending || !alien_result.pending) {
    throw std::runtime_error("Both independent owners must stage their own fetch");
  }
  const auto first = result.pending.value().ticket;
  const auto effect = result.pending.value().effect;
  bool rejected = false;
  try {
    static_cast<void>(owner.advance(alien_result.pending.value().ticket));
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected && cpu.registers() == initial && cpu.progress() == Progress{} &&
              cpu.pending_instruction().value().effect == effect && !owner.pending(),
          "A foreign ticket cannot advance or acquire an ordinary no-IRQ owner");
  result = owner.advance(first);
  const auto after_fetch = cpu.pending_instruction();
  rejected = false;
  try {
    static_cast<void>(owner.advance(first));
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected && after_fetch &&
              cpu.pending_instruction().value().effect == after_fetch->effect &&
              cpu.registers() == initial && cpu.progress() == Progress{} && !owner.pending(),
          "A consumed ticket is rejected without publication or interrupt acknowledgement");
  auto last = first;
  constexpr unsigned effect_limit = 16;
  for (unsigned effects = 0; effects < effect_limit && result.pending; ++effects) {
    last = result.pending.value().ticket;
    result = owner.advance(last);
  }
  auto expected = initial;
  expected.pc = foreground_address + 1;
  const Progress retired{.instructions = 1, .estimated_cycles = 1};
  require(result.retired && result.fault.kind == FaultKind::none && cpu.registers() == expected &&
              cpu.progress() == retired && !owner.pending(),
          "An authored ordinary NOP publishes only literal PC+1 and one instruction/base cycle");
  rejected = false;
  try {
    static_cast<void>(owner.advance(last));
  } catch (const std::logic_error &) { rejected = true; }
  require(rejected && cpu.registers() == expected && cpu.progress() == retired,
          "An unavailable retired ticket remains a visible refusal with unchanged CPU state");
  result = cpu.begin_fetch(InstructionPublication::pcu_updates);
  if (!result.pending) {
    throw std::runtime_error("Next actual program fetch must remain staged before sampling");
  }
  const auto failed = result.pending.value().ticket;
  result = owner.advance(failed);
  require(result.fault.kind == FaultKind::memory_read && result.fault.space == Space::program &&
              result.fault.address == foreground_address + 1 && io.failure() && !owner.pending() &&
              cpu.registers() == expected && cpu.progress() == retired,
          "An unknown program word retains its actual native fault without fabricated retirement");
  rejected = false;
  try {
    static_cast<void>(owner.advance(failed));
  } catch (const std::logic_error &) { rejected = true; }
  require(rejected && cpu.registers() == expected && cpu.progress() == retired && io.failure(),
          "A faulted ticket remains refused after a no-IRQ native read failure");
  std::cout << "PASS no-IRQ native NOP, foreign/stale/retired/faulted tickets and unknown fetch\n";
}

}

int main() {
  try {
    constexpr Word alternate_base = 0x100;
    no_irq_guards();
    check_vector(0, false);
    check_vector(alternate_base, false);
    check_vector(0, true);
    std::cout << "Own DSP interrupt ownership checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
