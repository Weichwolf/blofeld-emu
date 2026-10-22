// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/hc12_cycles.hpp"
#include "hc12_cycle_internal.hpp"

#include <array>
#include <cstdint>
#include <string_view>

namespace blofeld::detail {
namespace {
constexpr unsigned address_mask = 0xffff;
constexpr unsigned byte_moves = 8;
constexpr unsigned last_word_move = 5;
constexpr unsigned last_byte_move = 13;
constexpr unsigned branch_base = 0x20;
constexpr unsigned opcode_group_mask = 0xf0;
constexpr unsigned low_nibble = 15;
constexpr unsigned op_aba = 0x06;
constexpr unsigned op_daa = 0x07;
constexpr unsigned op_tab = 0x0e;
constexpr unsigned op_tba = 0x0f;
constexpr unsigned op_idiv = 0x10;
constexpr unsigned op_fdiv = 0x11;
constexpr unsigned op_emuls = 0x13;
constexpr unsigned op_edivs = 0x14;
constexpr unsigned op_idivs = 0x15;
constexpr unsigned op_sba = 0x16;
constexpr unsigned op_cba = 0x17;

constexpr unsigned first_minmax = 0x18;
constexpr unsigned last_minmax = 0x1f;

struct MinmaxSlots {
  std::string_view byte_read;
  std::string_view word_read;
  std::string_view byte_write;
  std::string_view word_write;
  unsigned bytes;
};

struct MinmaxEncoding {
  std::uint8_t post;
  unsigned opcode;
};

Hc12CyclePlan minmax_plan(const MinmaxEncoding encoding) {
  const auto post = encoding.post;
  const auto opcode = encoding.opcode;
  // S12CPUV2 min/max tables include the leading page-two fetch O.
  constexpr std::array<MinmaxSlots, 5> slots{{{.byte_read = "OrPf",
                                               .word_read = "ORPf",
                                               .byte_write = "OrPw",
                                               .word_write = "ORPW",
                                               .bytes = 3},
                                              {.byte_read = "OrPO",
                                               .word_read = "ORPO",
                                               .byte_write = "OrPwO",
                                               .word_write = "ORPWO",
                                               .bytes = 4},
                                              {.byte_read = "OfrPP",
                                               .word_read = "OfRPP",
                                               .byte_write = "OfrPwP",
                                               .word_write = "OfRPWP",
                                               .bytes = 5},
                                              {.byte_read = "OfIfrPf",
                                               .word_read = "OfIfRPf",
                                               .byte_write = "OfIfrPw",
                                               .word_write = "OfIfRPW",
                                               .bytes = 3},
                                              {.byte_read = "OfIPrPf",
                                               .word_read = "OfIPRPf",
                                               .byte_write = "OfIPrPw",
                                               .word_write = "OfIPRPW",
                                               .bytes = 5}}};
  constexpr unsigned extended_mask = 0xe4;
  constexpr unsigned extended_index = 0xe0;
  constexpr unsigned indirect_mask = 0xe7;
  constexpr unsigned indirect_d = 0xe7;
  unsigned mode = 0;
  if ((post & extended_mask) == extended_index) {
    const auto tail = post & 3U;
    if (tail < 2) {
      mode = 1;
    } else {
      mode = tail == 2 ? 2U : 4U;
    }
  } else if ((post & indirect_mask) == indirect_d) {
    mode = 3;
  }
  const auto row = slots.at(mode);
  const bool word = (opcode & 2U) != 0;
  if ((opcode & 4U) != 0) {
    return {.slots = word ? row.word_write : row.byte_write, .bytes = row.bytes};
  }
  return {.slots = word ? row.word_read : row.byte_read, .bytes = row.bytes};
}

std::uint8_t code_byte(const Bus &bus, const ExecutionContext context, const unsigned offset) {
  return bus.peek(static_cast<std::uint16_t>((context.pc + offset) & address_mask));
}

Hc12CyclePlan move_plan(const Bus &bus, const ExecutionContext context, const unsigned opcode) {
  const bool word = (opcode & byte_moves) == 0;
  const unsigned form = opcode & 7U;
  if (form != 3 && form != 4) {
    const auto post = code_byte(bus, context, 2);
    if (!hc12_cycle_short_index(post)) {
      throw Fault("MOV requires a single-byte non-indirect indexed operand: " + hex(post, 2));
    }
  }
  if (form == 2 && !hc12_cycle_short_index(code_byte(bus, context, 3))) {
    throw Fault("MOV requires a single-byte non-indirect destination index");
  }
  switch (form) {
    case 0: return {.slots = word ? "OPPW" : "OPwO", .bytes = word ? 5U : 4U};
    case 1: return {.slots = word ? "OPRPW" : "OPrPw", .bytes = 5};
    case 2: return {.slots = word ? "ORPWO" : "OrPwO", .bytes = 4};
    case 3: return {.slots = word ? "OPWPO" : "OPwP", .bytes = word ? 6U : 5U};
    case 4: return {.slots = word ? "ORPWPO" : "OrPwPO", .bytes = 6};
    case 5: return {.slots = word ? "ORPWP" : "OrPwP", .bytes = 5};
    default: throw Fault("Unsupported HC12 MOV addressing form");
  }
}
}

bool hc12_cycle_branch(const unsigned condition, const ExecutionContext context) {
  const auto ccr = context.ccr;
  const bool carry = (ccr & 1U) != 0;
  const bool zero = (ccr & 4U) != 0;
  const bool negative = (ccr & 8U) != 0;
  const bool overflow = (ccr & 2U) != 0;
  switch (condition) {
    case 0: return true;
    case 1: return false;
    case 2: return !carry && !zero;
    case 3: return carry || zero;
    case 4: return !carry;
    case 5: return carry;
    case 6: return !zero;
    case 7: return zero;
    case 8: return !overflow;
    case 9: return overflow;
    case 10: return !negative;
    case 11: return negative;
    case 12: return negative == overflow;
    case 13: return negative != overflow;
    case 14: return !zero && negative == overflow;
    case 15: return zero || negative != overflow;
    default: throw Fault("Invalid HC12 branch condition");
  }
}

Hc12CyclePlan hc12_page2_cycle_plan(const Bus &bus, const ExecutionContext context) {
  const unsigned opcode = code_byte(bus, context, 1);
  if (opcode <= last_word_move || (opcode >= byte_moves && opcode <= last_byte_move)) {
    return move_plan(bus, context, opcode);
  }
  if (opcode >= first_minmax && opcode <= last_minmax) {
    return minmax_plan({.post = code_byte(bus, context, 2), .opcode = opcode});
  }
  if ((opcode & opcode_group_mask) == branch_base) {
    return {.slots = hc12_cycle_branch(opcode & low_nibble, context) ? "OPPP" : "OPO", .bytes = 4};
  }
  switch (opcode) {
    case op_aba:
    case op_tab:
    case op_tba:
    case op_sba:
    case op_cba: return {.slots = "OO", .bytes = 2};
    case op_daa:
    case op_emuls: return {.slots = "OfO", .bytes = 2};
    case op_idiv:
    case op_fdiv:
    case op_edivs:
    case op_idivs: return {.slots = "OffffffffffO", .bytes = 2};
    default: throw Fault("Unsupported HC12 page-two timed opcode " + hex(opcode, 2));
  }
}
}
