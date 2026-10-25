// SPDX-License-Identifier: GPL-3.0-only
#include "keyboard/hcs08_registers.hpp"

#include "common/bus.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_division.hpp"

#include <cstdint>

namespace blofeld::detail::hcs08 {
namespace {
constexpr unsigned lda_immediate = 0xa6;
constexpr unsigned lda_direct = 0xb6;
constexpr unsigned lda_extended = 0xc6;
constexpr unsigned clra = 0x4f;
constexpr unsigned shift_x = 0x58, pull_x = 0x88, push_x = 0x89;
constexpr unsigned sta_stack = 0xe7, stx_stack = 0xef;
constexpr unsigned tpa = 0x85;
constexpr unsigned ldx_immediate = 0xae;
constexpr unsigned clrx = 0x5f;
constexpr unsigned ldhx_immediate = 0x45;
constexpr unsigned ldhx_direct = 0x55;
constexpr unsigned clrh = 0x8c;
constexpr unsigned txs = 0x94;
constexpr unsigned aix = 0xaf;
constexpr unsigned cphx_immediate = 0x65;
constexpr unsigned sthx_direct = 0x35;
constexpr unsigned clr_indexed = 0x7f;
constexpr unsigned sta_direct = 0xb7;
constexpr unsigned sta_extended = 0xc7;
constexpr unsigned bit_immediate = 0xa5;
constexpr unsigned cmp_immediate = 0xa1;
constexpr unsigned tap = 0x84;
constexpr unsigned stack_prefix = 0x9e;
constexpr unsigned mov_immediate_direct = 0x6e;
constexpr unsigned clr_direct = 0x3f;
constexpr unsigned bra = 0x20;
constexpr unsigned bcs = 0x25;
constexpr unsigned beq = 0x27;
constexpr unsigned jsr_extended = 0xcd;
constexpr unsigned ais = 0xa7;
constexpr unsigned nop = 0x9d;
constexpr unsigned bgnd = 0x82;
constexpr unsigned cli = 0x9a;
constexpr unsigned sei = 0x9b;
constexpr unsigned clc = 0x98;
constexpr unsigned sec = 0x99;
constexpr unsigned ldhx_stack = 0xfe;
constexpr unsigned sthx_stack = 0xff;
constexpr unsigned ldx_direct = 0xbe, and_immediate = 0xa4, inc_a = 0x4c, inc_x = 0x5c;
constexpr unsigned mov_direct = 0x4e, cmp_direct = 0xb1;
constexpr unsigned lda_indexed16 = 0xd6, cmp_extended = 0xc1;
constexpr unsigned push_h = 0x8b, pull_h = 0x8a, tax = 0x97, txa = 0x9f, brclr7 = 0x0f;
constexpr unsigned jmp_direct = 0xbc, jmp_extended = 0xcc;
constexpr unsigned jmp_indexed16 = 0xdc, jmp_indexed8 = 0xec, jmp_indexed = 0xfc;
constexpr unsigned byte_mask = 0xff, high_mask = 0xff00, word_mask = 0xffff;

[[noreturn]] void unqualified(const Hcs08State &s, const unsigned opcode) {
  throw Fault("HCS08 partial-register instruction operation address=" + hex(s.pc) +
              " pc=" + hex(s.pc) + " opcode=" + hex(opcode) + " a-known=" + hex(s.a_known) +
              " hx-known=" + hex(s.hx_known) + " ccr=" + hex(s.ccr) +
              " ccr-known=" + hex(s.ccr_known) + ": unknown operand or unqualified semantics");
}

RegisterKnowledge stack(const Bus &bus, const Hcs08State &s) {
  bus.check_execute({.address = s.pc, .length = 2});
  const auto second = bus.peek(static_cast<std::uint16_t>(s.pc + 1U));
  if (second == ldhx_stack) { return {.a = s.a_known, .hx = word_mask}; } // LDHX SP1
  if (second == sthx_stack && s.hx_known == word_mask) {                  // STHX SP1
    return {.a = s.a_known, .hx = s.hx_known};
  }
  if ((second == sta_stack && s.a_known == byte_mask) ||
      (second == stx_stack && (s.hx_known & byte_mask) == byte_mask)) {
    return {.a = s.a_known, .hx = s.hx_known};
  }
  unqualified(s, (stack_prefix << 8U) | second);
}

RegisterKnowledge division(const Bus &bus, const Hcs08State &state) {
  bus.check_execute({.address = state.pc, .length = 1});
  if (state.a_known != byte_mask || state.hx_known != word_mask) {
    unqualified(state, divide_opcode);
  }
  // Overflow/zero leaves only the unchanged X known; A and H are indeterminate.
  const bool defined = division_defined(state);
  return {.a = static_cast<std::uint8_t>(defined ? byte_mask : 0),
          .hx = static_cast<std::uint16_t>(defined ? word_mask : byte_mask)};
}
}

RegisterKnowledge registers_after(const Bus &bus, const Hcs08State &before) {
  const auto &s = before;
  RegisterKnowledge result{.a = s.a_known, .hx = s.hx_known};
  if (bus.peek(s.pc) == divide_opcode) { return division(bus, s); }
  if (s.a_known == byte_mask && s.hx_known == word_mask) { return result; }
  bus.check_execute({.address = s.pc, .length = 1});
  const auto opcode = bus.peek(s.pc);
  switch (opcode) {
    case lda_immediate:                     // LDA immediate
    case lda_direct:                        // LDA direct
    case lda_extended:                      // LDA extended
    case clra: result.a = byte_mask; break; // CLRA
    case lda_indexed16:
      if (s.hx_known != word_mask) { unqualified(s, opcode); }
      result.a = byte_mask;
      break;
    case tpa: // TPA; partial CCR cannot be converted into known A.
      if (s.ccr_known != Hcs08State::all_ccr) { unqualified(s, opcode); }
      result.a = byte_mask;
      break;
    case ldx_immediate: // LDX immediate
    case ldx_direct:
    case clrx:
    case pull_x: result.hx |= byte_mask; break;
    case ldhx_immediate:
    case ldhx_direct: result.hx = word_mask; break; // LDHX immediate
    case clrh: // CLRH and PULH define H without consuming its prior value.
    case pull_h: result.hx |= high_mask; break;
    case push_h:
      if ((s.hx_known & high_mask) != high_mask) { unqualified(s, opcode); }
      break;
    case inc_x:
    case shift_x:
    case push_x:
    case txa:
      if ((s.hx_known & byte_mask) != byte_mask) { unqualified(s, opcode); }
      if (opcode == txa) { result.a = byte_mask; }
      break;
    case tax:
      if (s.a_known != byte_mask) { unqualified(s, opcode); }
      result.hx |= byte_mask;
      break;
    case txs:            // TXS
    case aix:            // AIX
    case cphx_immediate: // CPHX immediate
    case sthx_direct:    // STHX direct
    case clr_indexed:    // CLR indexed
    case jmp_indexed16:
    case jmp_indexed8:
    case jmp_indexed:
      if (s.hx_known != word_mask) { unqualified(s, opcode); }
      break;
    case sta_direct:    // STA direct
    case sta_extended:  // STA extended
    case bit_immediate: // BIT immediate
    case cmp_immediate: // CMP immediate
    case cmp_extended:
    case cmp_direct:
    case and_immediate:
    case inc_a:
    case tap: // TAP
      if (s.a_known != byte_mask) { unqualified(s, opcode); }
      break;
    case stack_prefix: return stack(bus, s);
    case mov_immediate_direct: // MOV immediate,direct
    case mov_direct:
    case brclr7:
    case clr_direct:   // CLR direct
    case bra:          // BRA
    case bcs:          // BCS
    case beq:          // BEQ
    case jsr_extended: // JSR extended
    case jmp_direct:
    case jmp_extended:
    case ais:        // AIS
    case nop:        // NOP
    case bgnd:       // BGND
    case cli:        // CLI
    case sei:        // SEI
    case clc:        // CLC
    case sec: break; // SEC
    default: unqualified(s, opcode);
  }
  return result;
}
}
