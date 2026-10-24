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
}

std::uint8_t ccr_after(const Bus &bus, const Hcs08State &before) {
  constexpr unsigned all = 0xff;
  constexpr unsigned vnz = 0x86;
  constexpr unsigned interrupt = 8;
  constexpr unsigned carry = 1;
  if (before.ccr_known == all) { return all; }
  bus.check_execute({.address = before.pc, .length = 1});
  const auto opcode = bus.peek(before.pc);
  unsigned defined = 0;
  switch (opcode) {
    case ldx_immediate:                      // LDX immediate
    case lda_immediate:                      // LDA immediate
    case sta_extended: defined = vnz; break; // STA extended
    case txs:                                // TXS
    case nop:                                // NOP
    case bgnd: break;                        // BGND
    case tap: defined = all; break;          // TAP
    case cli:                                // CLI
    case sei: defined = interrupt; break;    // SEI
    case clc:                                // CLC
    case sec: defined = carry; break;        // SEC
    default:
      throw Fault("HCS08 partial-CCR instruction operation address=" + hex(before.pc) +
                  " pc=" + hex(before.pc) + " opcode=" + hex(opcode) +
                  " ccr-known=" + hex(before.ccr_known) +
                  ": unknown flags or unqualified partial-register semantics");
  }
  return static_cast<std::uint8_t>(before.ccr_known | defined);
}
}
