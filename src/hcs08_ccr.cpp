// SPDX-License-Identifier: GPL-3.0-only
#include "hcs08_ccr.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/hcs08.hpp"

#include <cstdint>

namespace blofeld::detail::hcs08 {
namespace {
constexpr unsigned ldx_immediate = 0xae, lda_immediate = 0xa6, sta_extended = 0xc7;
constexpr unsigned txs = 0x94, nop = 0x9d, bgnd = 0x82, tap = 0x84;
constexpr unsigned cli = 0x9a, sei = 0x9b, clc = 0x98, sec = 0x99;
constexpr unsigned ldhx_immediate = 0x45, clrh = 0x8c, clrx = 0x5f, clra = 0x4f, clr_indexed = 0x7f;
constexpr unsigned bra = 0x20, bcs = 0x25, aix = 0xaf, cphx_immediate = 0x65;
constexpr unsigned sthx_direct = 0x35, sta_direct = 0xb7, stack_prefix = 0x9e;
constexpr unsigned ldhx_stack = 0xfe, sthx_stack = 0xff;
constexpr unsigned lda_direct = 0xb6, lda_extended = 0xc6, clr_direct = 0x3f;
constexpr unsigned mov_immediate_direct = 0x6e, bit_immediate = 0xa5, beq = 0x27;
constexpr unsigned cmp_immediate = 0xa1, jsr_extended = 0xcd, ais = 0xa7;

[[noreturn]] void unqualified(const Hcs08State &before, const unsigned opcode) {
  throw Fault("HCS08 partial-CCR instruction operation address=" + hex(before.pc) + " pc=" +
              hex(before.pc) + " opcode=" + hex(opcode) + " ccr-known=" + hex(before.ccr_known) +
              ": unknown flags or unqualified partial-register semantics");
}

void qualify_stack(const Bus &bus, const Hcs08State &before) {
  bus.check_execute({.address = before.pc, .length = 2});
  const auto second = bus.peek(static_cast<std::uint16_t>(before.pc + 1U));
  if (second != ldhx_stack && second != sthx_stack) {
    unqualified(before, (stack_prefix << 8U) | second);
  }
}
}

std::uint8_t ccr_after(const Bus &bus, const Hcs08State &before) {
  constexpr unsigned all = 0xff;
  constexpr unsigned vnz = 0x86;
  constexpr unsigned interrupt = 8;
  constexpr unsigned carry = 1;
  constexpr unsigned zero = 2;
  if (before.ccr_known == all) { return all; }
  bus.check_execute({.address = before.pc, .length = 1});
  const auto opcode = bus.peek(before.pc);
  unsigned defined = 0;
  switch (opcode) {
    case ldx_immediate: // LDX immediate
    case lda_immediate: // LDA immediate
    case sta_extended:  // STA extended
    case ldhx_immediate:
    case lda_direct:
    case lda_extended:
    case clr_direct:
    case mov_immediate_direct:
    case bit_immediate:
    case sta_direct:
    case sthx_direct:
    case clrh:
    case clrx:
    case clra:
    case clr_indexed: defined = vnz; break; // CLR preserves C, including unknown C.
    case cphx_immediate:
    case cmp_immediate: defined = vnz | carry; break;
    case stack_prefix:
      qualify_stack(bus, before);
      defined = vnz;
      break;
    case bcs:
      if ((before.ccr_known & carry) == 0) { unqualified(before, opcode); }
      break;
    case beq:
      if ((before.ccr_known & zero) == 0) { unqualified(before, opcode); }
      break;
    case jsr_extended:
    case ais:
    case bra:
    case aix:
    case txs:                             // TXS
    case nop:                             // NOP
    case bgnd: break;                     // BGND
    case tap: defined = all; break;       // TAP
    case cli:                             // CLI
    case sei: defined = interrupt; break; // SEI
    case clc:                             // CLC
    case sec: defined = carry; break;     // SEC
    default: unqualified(before, opcode);
  }
  return static_cast<std::uint8_t>(before.ccr_known | defined);
}
}
