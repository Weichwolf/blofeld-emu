// SPDX-License-Identifier: GPL-3.0-only
#include "keyboard/hcs08_interrupts.hpp"

#include "common/bus.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_known_interrupts.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace blofeld::detail::hcs08 {
namespace {
constexpr unsigned mask = 8, first_vector = 0xffc0, last_vector = 0xfffa;

[[noreturn]] void fail(const Hcs08State &state, const unsigned address, const std::string &reason) {
  throw Fault("HCS08 IRQ query address=" + hex(address) +
              " pc=" + (state.pc_known ? hex(state.pc) : "unknown pc-known=0") +
              " ccr=" + hex(state.ccr) + " ccr-known=" + hex(state.ccr_known) + ": " + reason);
}
}

std::optional<std::uint16_t>
pending_interrupt(const Bus &bus, const Hcs08State &state, const Hcs08IrqDelay delay) {
  if (delay == Hcs08IrqDelay::next_instruction) { return {}; }
  // Legacy owners expose local sources without a substituted global I bit.
  // An owner of uncertain history needs the actual mask and its knowledge.
  const auto *const known = dynamic_cast<const Hcs08KnownInterrupts *>(&bus);
  const auto request = known != nullptr
                           ? known->interrupt_request(state)
                           : bus.interrupt_request(static_cast<std::uint8_t>(state.ccr & ~mask));
  if (!request) { return {}; }
  if (request->kind != InterruptKind::maskable) {
    fail(state, request->vector, "unsupported HCS08 interrupt kind");
  }
  if ((request->vector & 1U) != 0 || request->vector < first_vector ||
      request->vector > last_vector) {
    fail(state, request->vector, "invalid maskable vector (reset/SWI are not hardware IRQs)");
  }
  if ((state.ccr_known & mask) == 0) { fail(state, request->vector, "unknown CPU I mask"); }
  if ((state.ccr & mask) != 0) { return {}; }
  if (!state.pc_known) { fail(state, request->vector, "unknown stacked CPU PC"); }
  if (delay == Hcs08IrqDelay::unqualified_return) {
    fail(state,
         request->vector,
         "RTI IRQ timing contract is unqualified: conflicting primary descriptions");
  }
  if (delay == Hcs08IrqDelay::unknown_transition) {
    fail(state,
         request->vector,
         "unknown I-mask transition requires next-instruction qualification");
  }
  return request->vector;
}

Hcs08IrqDelay irq_delay_after(const Hcs08State &before,
                              const Hcs08State &after,
                              const unsigned opcode,
                              const Hcs08RtiIrqContract contract) {
  constexpr unsigned cli = 0x9a;
  constexpr unsigned tap = 0x84;
  constexpr unsigned rti = 0x80;
  if (opcode != cli && opcode != tap && opcode != rti) { return Hcs08IrqDelay::none; }
  if (((before.ccr_known & mask) != 0 && (before.ccr & mask) == 0) ||
      ((after.ccr_known & mask) != 0 && (after.ccr & mask) != 0)) {
    return Hcs08IrqDelay::none;
  }
  if (opcode == rti) {
    if (contract == Hcs08RtiIrqContract::unqualified) { return Hcs08IrqDelay::unqualified_return; }
    if (contract == Hcs08RtiIrqContract::reference_immediate) { return Hcs08IrqDelay::none; }
  }
  // With known old I=1, an unknown restored I either masks IRQs or enables
  // this documented guarantee. Neither alternative may preempt the next instruction.
  if ((before.ccr_known & mask) != 0) { return Hcs08IrqDelay::next_instruction; }
  return Hcs08IrqDelay::unknown_transition;
}
}
