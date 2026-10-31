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
  auto result = cpu.begin_fetch(InstructionPublication::pcu_updates);
  unsigned fetched = 0;
  for (unsigned effects = 0; effects < 16 && result.pending && !result.retired; ++effects) {
    const auto ticket = result.pending->ticket;
    const auto effect = result.pending->effect;
    result = owner.advance(ticket);
    if (effect.kind == EffectKind::fetch_opcode) {
      require(effect.address == address, "Native vector fetch address changed");
      if (result.fault.kind != FaultKind::none) { return result; }
      ++fetched;
      require(owner.pending() == first && bus.interrupt_request(0).has_value() == first,
              "Peripheral acknowledgement did not follow the second real vector read");
      if (first) {
        bool rejected = false;
        try {
          static_cast<void>(owner.advance(ticket));
        } catch (const std::invalid_argument &) { rejected = true; }
        require(rejected && owner.pending() && bus.interrupt_request(0),
                "Stale native ticket consumed interrupt ownership");
      }
    }
  }
  require(result.retired && result.fault.kind == FaultKind::none && fetched == 1,
          "Vector word did not genuinely fetch and retire");
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
  static_cast<void>(vector_word(cpu, owner, bus, base + irqa_vector, true));
  const auto result = vector_word(cpu, owner, bus, base + irqa_vector + 1, false);
  if (missing_second) {
    require(result.fault.kind == FaultKind::memory_read && result.fault.space == Space::program &&
                result.fault.address == base + irqa_vector + 1 && io.failure(),
            "Unknown second vector word did not report the actual failed program fetch");
    require(owner.pending() && bus.interrupt_request(0),
            "Refused second vector read acknowledged an interrupt");
  } else {
    require(cpu.registers().pc == foreground_address && !owner.pending(),
            "Fast interrupt did not return to foreground");
  }
}
}

int main() {
  try {
    constexpr Word alternate_base = 0x100;
    check_vector(0, false);
    check_vector(alternate_base, false);
    check_vector(0, true);
    std::cout << "Own DSP interrupt ownership checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
