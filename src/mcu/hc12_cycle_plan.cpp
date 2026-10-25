// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/hc12_cycle_internal.hpp"
#include "mcu/hc12_cycles.hpp"

#include <cstdint>

namespace blofeld {
namespace {
constexpr unsigned op_rti = 0x0b;
constexpr unsigned op_andcc = 0x10;
constexpr unsigned op_orcc = 0x14;
constexpr unsigned op_transfer = 0xb7;
constexpr unsigned op_prefix = 0x18;
constexpr unsigned op_lea_y = 0x19;
constexpr unsigned op_lea_x = 0x1a;
constexpr unsigned op_lea_s = 0x1b;
constexpr unsigned op_bsr = 7;
constexpr unsigned op_ediv = 0x11;
constexpr unsigned op_mul = 0x12;
constexpr unsigned op_emul = 0x13;
constexpr unsigned op_test_indexed = 0xe7;
constexpr unsigned op_test_extended = 0xf7;
constexpr unsigned op_increment_y = 2;
constexpr unsigned op_decrement_y = 3;
constexpr unsigned op_increment_x = 8;
constexpr unsigned op_decrement_x = 9;
constexpr unsigned op_pull_x = 0x30;
constexpr unsigned op_pull_y = 0x31;
constexpr unsigned op_pull_d = 0x3a;
constexpr unsigned op_pull_a = 0x32;
constexpr unsigned op_pull_b = 0x33;
constexpr unsigned op_pull_ccr = 0x38;
constexpr unsigned op_push_x = 0x34;
constexpr unsigned op_push_y = 0x35;
constexpr unsigned op_push_d = 0x3b;
constexpr unsigned op_push_a = 0x36;
constexpr unsigned op_push_b = 0x37;
constexpr unsigned op_push_ccr = 0x39;
constexpr unsigned op_rts = 0x3d;
constexpr unsigned op_wai = 0x3e;
constexpr unsigned op_swi = 0x3f;
constexpr unsigned op_clear_a = 0x87;
constexpr unsigned op_test_a = 0x97;
constexpr unsigned op_nop = 0xa7;
constexpr unsigned op_clear_b = 0xc7;
constexpr unsigned op_test_b = 0xd7;
enum class Index : std::uint8_t { short_offset, offset9, offset16, indirect_d, indirect16 };

Index index_kind(const std::uint8_t post) {
  constexpr unsigned special_mask = 0xe4;
  constexpr unsigned special = 0xe0;
  if ((post & special_mask) != special) {
    constexpr unsigned indirect_mask = 0xe7;
    constexpr unsigned indirect_d = 0xe7;
    return (post & indirect_mask) == indirect_d ? Index::indirect_d : Index::short_offset;
  }
  switch (post & 3U) {
    case 0:
    case 1: return Index::offset9;
    case 2: return Index::offset16;
    case 3: return Index::indirect16;
    default: throw Fault("Invalid HC12 index classification");
  }
}

Hc12CyclePlan indexed_read(const Index kind, const bool word) {
  switch (kind) {
    case Index::short_offset: return {.slots = word ? "RPf" : "rPf", .bytes = 2};
    case Index::offset9: return {.slots = word ? "RPO" : "rPO", .bytes = 3};
    case Index::offset16: return {.slots = word ? "fRPP" : "frPP", .bytes = 4};
    case Index::indirect_d: return {.slots = word ? "fIfRPf" : "fIfrPf", .bytes = 2};
    case Index::indirect16: return {.slots = word ? "fIPRPf" : "fIPrPf", .bytes = 4};
    default: throw Fault("Invalid HC12 indexed read");
  }
}

Hc12CyclePlan indexed_write(const Index kind, const bool word) {
  switch (kind) {
    case Index::short_offset: return {.slots = word ? "PW" : "Pw", .bytes = 2};
    case Index::offset9: return {.slots = word ? "PWO" : "PwO", .bytes = 3};
    case Index::offset16: return {.slots = word ? "PWP" : "PwP", .bytes = 4};
    case Index::indirect_d: return {.slots = word ? "PIfW" : "PIfw", .bytes = 2};
    case Index::indirect16: return {.slots = word ? "PIPW" : "PIPw", .bytes = 4};
    default: throw Fault("Invalid HC12 indexed write");
  }
}

Hc12CyclePlan bit_plan(const unsigned opcode, const Index kind) {
  const bool branch = (opcode & 2U) != 0;
  const auto high = opcode >> 4U;
  if (high == 4) { return {.slots = branch ? "rPPP" : "rPwO", .bytes = branch ? 4U : 3U}; }
  if (high == 1) { return {.slots = branch ? "rfPPP" : "rPwP", .bytes = branch ? 5U : 4U}; }
  switch (kind) {
    case Index::short_offset: return {.slots = branch ? "rPPP" : "rPwO", .bytes = branch ? 4U : 3U};
    case Index::offset9: return {.slots = branch ? "rfPPP" : "rPwP", .bytes = branch ? 5U : 4U};
    case Index::offset16: return {.slots = branch ? "PrfPPP" : "frPwPO", .bytes = branch ? 6U : 5U};
    case Index::indirect_d:
    case Index::indirect16: throw Fault("Indirect HC12 bit instruction timing is unsupported");
    default: throw Fault("Invalid HC12 bit instruction index mode");
  }
}

Hc12CyclePlan control_plan(const unsigned opcode, const Index kind) {
  if (opcode == 6) { return {.slots = "PPP", .bytes = 3}; }
  if (opcode == op_bsr) { return {.slots = "SPPP", .bytes = 2}; }
  constexpr unsigned extended_call = 0x16;
  constexpr unsigned direct_call = 0x17;
  constexpr unsigned indexed_call = 0x15;
  if (opcode == extended_call) { return {.slots = "SPPP", .bytes = 3}; }
  if (opcode == direct_call) { return {.slots = "SPPP", .bytes = 2}; }
  const bool call = opcode == indexed_call;
  switch (kind) {
    case Index::short_offset: return {.slots = call ? "PPPS" : "PPP", .bytes = 2};
    case Index::offset9: return {.slots = call ? "PPPS" : "PPP", .bytes = 3};
    case Index::offset16: return {.slots = call ? "fPPPS" : "fPPP", .bytes = 4};
    case Index::indirect_d: return {.slots = call ? "fIfPPPS" : "fIfPPP", .bytes = 2};
    case Index::indirect16: return {.slots = call ? "fIfPPPS" : "fIfPPP", .bytes = 4};
    default: throw Fault("Invalid HC12 control transfer timing");
  }
}

Hc12CyclePlan special_plan(const unsigned opcode) {
  const auto high = opcode >> 4U;
  const auto low = opcode & 15U;
  switch (opcode) {
    case op_increment_x:
    case op_increment_y:
    case op_decrement_x:
    case op_decrement_y: return {.slots = "O", .bytes = 1};
    case op_ediv: return {.slots = "ffffffffffO", .bytes = 1};
    case op_mul: return {.slots = "O", .bytes = 1};
    case op_emul: return {.slots = "ffO", .bytes = 1};
    case op_rti: return {.slots = "uUUUUPPP", .bytes = 1};
    case op_andcc:
    case op_orcc:
    case op_transfer: return {.slots = "P", .bytes = 2};
    case op_pull_x:
    case op_pull_y:
    case op_pull_d: return {.slots = "UfO", .bytes = 1};
    case op_pull_a:
    case op_pull_b:
    case op_pull_ccr: return {.slots = "ufO", .bytes = 1};
    case op_push_x:
    case op_push_y:
    case op_push_d: return {.slots = "OS", .bytes = 1};
    case op_push_a:
    case op_push_b:
    case op_push_ccr: return {.slots = "Os", .bytes = 1};
    case op_rts: return {.slots = "UfPPP", .bytes = 1};
    case op_wai: return {.slots = "OSSSSsf", .bytes = 1};
    case op_swi: return {.slots = "VSPSSPSsP", .bytes = 1};
    case op_clear_a:
    case op_test_a:
    case op_nop:
    case op_clear_b:
    case op_test_b: return {.slots = "O", .bytes = 1};
    default:
      if ((high == 4 || high == 5) && low <= 9) { return {.slots = "O", .bytes = 1}; }
      return {};
  }
}

Hc12CyclePlan alu_plan(const unsigned opcode, const Index index) {
  const bool word = (opcode & 15U) == 3 || (opcode & 15U) >= 12;
  switch ((opcode >> 4U) & 3U) {
    case 0: return {.slots = word ? "PO" : "P", .bytes = word ? 3U : 2U};
    case 1: return {.slots = word ? "RPf" : "rPf", .bytes = 2};
    case 2: return indexed_read(index, word);
    case 3: return {.slots = word ? "RPO" : "rPO", .bytes = 3};
    default: throw Fault("Invalid HC12 ALU mode");
  }
}

Hc12CyclePlan store_plan(const unsigned opcode, const Index index) {
  const bool word = (opcode & 15U) >= 12;
  switch (opcode >> 4U) {
    case 5: return {.slots = word ? "PW" : "Pw", .bytes = 2};
    case 6: return indexed_write(index, word);
    case 7: return {.slots = word ? "PWO" : "PwO", .bytes = 3};
    default: throw Fault("Invalid HC12 store mode");
  }
}

Hc12CyclePlan lea_plan(const Index index) {
  switch (index) {
    case Index::short_offset: return {.slots = "Pf", .bytes = 2};
    case Index::offset9: return {.slots = "PO", .bytes = 3};
    case Index::offset16: return {.slots = "PP", .bytes = 4};
    case Index::indirect_d:
    case Index::indirect16: throw Fault("Indirect LEA operands are unsupported");
    default: throw Fault("Invalid HC12 LEA index mode");
  }
}

Hc12CyclePlan modify_plan(const bool extended, const Index index) {
  if (extended) { return {.slots = "rPwO", .bytes = 3}; }
  switch (index) {
    case Index::short_offset: return {.slots = "rPw", .bytes = 2};
    case Index::offset9: return {.slots = "rPwO", .bytes = 3};
    case Index::offset16: return {.slots = "frPwP", .bytes = 4};
    case Index::indirect_d: return {.slots = "fIfrPw", .bytes = 2};
    case Index::indirect16: return {.slots = "fIPrPw", .bytes = 4};
    default: throw Fault("Invalid HC12 read-modify-write index mode");
  }
}

Hc12CyclePlan select_plan(const Bus &bus, const ExecutionContext context) {
  const unsigned opcode = bus.peek(context.pc);
  if (opcode == op_prefix) { return detail::hc12_page2_cycle_plan(bus, context); }
  const auto special = special_plan(opcode);
  if (special.bytes != 0) { return special; }
  constexpr unsigned address_mask = 0xffff;
  const auto post = bus.peek(static_cast<std::uint16_t>((context.pc + 1U) & address_mask));
  const auto index = index_kind(post);
  const auto high = opcode >> 4U;
  const auto low = opcode & 15U;
  if (opcode == op_lea_y || opcode == op_lea_x || opcode == op_lea_s) { return lea_plan(index); }
  if (opcode == op_test_indexed) { return indexed_read(index, false); }
  if (opcode == op_test_extended) { return {.slots = "rPO", .bytes = 3}; }
  if (high == 2) {
    return {.slots = detail::hc12_cycle_branch(low, context) ? "PPP" : "P", .bytes = 2};
  }
  if ((high == 0 || high == 1 || high == 4) && low >= 12) { return bit_plan(opcode, index); }
  if (opcode == 5 || opcode == 6 || opcode == op_bsr || (high == 1 && low >= 5 && low <= 7)) {
    return control_plan(opcode, index);
  }
  if (opcode == 4) { return {.slots = "PPq", .bytes = 3}; }
  if (high >= 8 && low != 7) { return alu_plan(opcode, index); }
  if (high >= 5 && high <= 7 && low >= 9) { return store_plan(opcode, index); }
  if (high == 6 || high == 7) { return modify_plan(high == 7, index); }
  throw Fault("Unsupported HC12 timed opcode " + hex(opcode, 2));
}
}

bool detail::hc12_cycle_short_index(const std::uint8_t post) {
  return index_kind(post) == Index::short_offset;
}

Hc12CyclePlan hc12_cycle_plan(const Bus &bus, const ExecutionContext context) {
  return select_plan(bus, context);
}
}
