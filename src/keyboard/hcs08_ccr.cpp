// SPDX-License-Identifier: GPL-3.0-only
#include "keyboard/hcs08_ccr.hpp"

#include "common/bus.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_division.hpp"

#include <cstdint>

namespace blofeld::detail::hcs08 {
namespace {
constexpr unsigned ldx_immediate = 0xae, lda_immediate = 0xa6, sta_extended = 0xc7;
constexpr unsigned txs = 0x94, nop = 0x9d, bgnd = 0x82, tap = 0x84;
constexpr unsigned cli = 0x9a, sei = 0x9b, clc = 0x98, sec = 0x99;
constexpr unsigned ldhx_immediate = 0x45, clrh = 0x8c, clrx = 0x5f, clr_indexed = 0x7f;
constexpr unsigned bra = 0x20, bcs = 0x25, aix = 0xaf, cphx_immediate = 0x65;
constexpr unsigned sthx_direct = 0x35, sta_direct = 0xb7, stack_prefix = 0x9e;
constexpr unsigned ldhx_stack = 0xfe, sthx_stack = 0xff, ldhx_direct = 0x55;
constexpr unsigned clra = 0x4f;
constexpr unsigned shift_x = 0x58, pull_x = 0x88;
constexpr unsigned sta_stack = 0xe7, stx_stack = 0xef;
constexpr unsigned lda_direct = 0xb6, lda_extended = 0xc6, clr_direct = 0x3f;
constexpr unsigned mov_immediate_direct = 0x6e, bit_immediate = 0xa5, beq = 0x27;
constexpr unsigned cmp_immediate = 0xa1, jsr_extended = 0xcd, ais = 0xa7;
constexpr unsigned ldx_direct = 0xbe, and_immediate = 0xa4, inc_a = 0x4c, inc_x = 0x5c;
constexpr unsigned ldx_extended = 0xce;
constexpr unsigned mov_direct = 0x4e, sta_indexed16 = 0xd7, cmp_direct = 0xb1;
constexpr unsigned lda_indexed16 = 0xd6, cmp_extended = 0xc1;
constexpr unsigned push_h = 0x8b, pull_h = 0x8a, tax = 0x97, txa = 0x9f, brclr7 = 0x0f;
constexpr unsigned brset7 = 0x0e, bhi = 0x22, bls = 0x23, bne = 0x26, jmp_extended = 0xcc;
constexpr unsigned jmp_direct = 0xbc, jmp_indexed16 = 0xdc, jmp_indexed8 = 0xec, jmp_indexed = 0xfc;
constexpr unsigned push_a = 0x87, push_x = 0x89, pull_a = 0x86;
constexpr unsigned lda_indexed8 = 0xe6, cmp_indexed = 0xf1;
constexpr unsigned ldx_stack = 0xee, cmp_stack = 0xe1, asl_stack = 0x68, rol_stack = 0x69,
                   add_stack = 0xeb;
constexpr unsigned vnz = 0x86, carry = 1, zero = 2, half = 0x10;

[[noreturn]] void unqualified(const Hcs08State &before, const unsigned opcode) {
  throw Fault("HCS08 partial-CCR instruction operation address=" + hex(before.pc) + " pc=" +
              hex(before.pc) + " opcode=" + hex(opcode) + " ccr-known=" + hex(before.ccr_known) +
              ": unknown flags or unqualified partial-register semantics");
}

unsigned stack_defined(const Bus &bus, const Hcs08State &before) {
  bus.check_execute({.address = before.pc, .length = 2});
  const auto second = bus.peek(static_cast<std::uint16_t>(before.pc + 1U));
  switch (second) {
    case ldhx_stack:
    case sthx_stack:
    case ldx_stack:
    case sta_stack:
    case stx_stack: return vnz;
    case cmp_stack:
    case asl_stack: return vnz | carry;
    case rol_stack:
      if ((before.ccr_known & carry) == 0) { unqualified(before, (stack_prefix << 8U) | second); }
      return vnz | carry;
    case add_stack: return vnz | carry | half;
    default: unqualified(before, (stack_prefix << 8U) | second);
  }
}

bool carry_or_zero_known(const Hcs08State &before) {
  constexpr unsigned bits = carry | zero;
  // A known set bit decides C|Z even if the other bit remains unknown.
  return (before.ccr_known & bits) == bits || (before.ccr & before.ccr_known & bits) != 0;
}
}

std::uint8_t ccr_after(const Bus &bus, const Hcs08State &before) {
  constexpr unsigned all = 0xff;
  constexpr unsigned interrupt = 8;
  if (bus.peek(before.pc) == divide_opcode) {
    bus.check_execute({.address = before.pc, .length = 1});
    // Z depends on the quotient. C is defined even when A/H are indeterminate.
    const unsigned known = before.ccr_known | carry;
    return static_cast<std::uint8_t>(division_defined(before) ? known | zero : known & ~zero);
  }
  if (before.ccr_known == all) { return all; }
  bus.check_execute({.address = before.pc, .length = 1});
  const auto opcode = bus.peek(before.pc);
  unsigned defined = 0;
  switch (opcode) {
    case ldx_immediate: // LDX immediate
    case lda_immediate: // LDA immediate
    case sta_extended:  // STA extended
    case ldhx_immediate:
    case ldhx_direct:
    case lda_direct:
    case lda_extended:
    case clr_direct:
    case mov_immediate_direct:
    case bit_immediate:
    case ldx_direct:
    case ldx_extended:
    case and_immediate:
    case inc_a:
    case inc_x:
    case mov_direct:
    case lda_indexed16:
    case lda_indexed8:
    case sta_indexed16:
    case sta_direct:
    case sthx_direct:
    case clrh:
    case clrx:
    case clra:
    case clr_indexed: defined = vnz; break; // CLR preserves C, including unknown C.
    case shift_x:
    case cphx_immediate:
    case cmp_extended:
    case cmp_direct:
    case cmp_indexed:
    case cmp_immediate: defined = vnz | carry; break;
    case brset7:
    case brclr7: defined = carry; break; // C copies the tested memory bit, not an incoming flag.
    case stack_prefix: defined = stack_defined(bus, before); break;
    case bhi:
    case bls:
      if (!carry_or_zero_known(before)) { unqualified(before, opcode); }
      break;
    case bcs:
      if ((before.ccr_known & carry) == 0) { unqualified(before, opcode); }
      break;
    case beq:
    case bne:
      if ((before.ccr_known & zero) == 0) { unqualified(before, opcode); }
      break;
    case jsr_extended:
    case jmp_extended:
    case jmp_direct:
    case jmp_indexed16:
    case jmp_indexed8:
    case jmp_indexed:
    case push_a:
    case push_x:
    case pull_x:
    case pull_a:
    case push_h:
    case pull_h:
    case tax:
    case txa:
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
