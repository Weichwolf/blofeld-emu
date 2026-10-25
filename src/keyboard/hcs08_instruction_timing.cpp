// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "keyboard/hcs08_run_clock.hpp"

#include <array>
#include <cstdint>

namespace blofeld {
namespace {
constexpr unsigned prefix_opcode = 0x9e, address_space = 65536, nibble_mask = 15;
constexpr unsigned cbeqa = 0x41, cbeqx = 0x51, mul = 0x42, div = 0x52;
constexpr unsigned ldhx_immediate = 0x45, ldhx_direct = 0x55, dbnza = 0x4b, dbnzx = 0x5b;
constexpr unsigned mov_direct_direct = 0x4e, mov_direct_indexed = 0x5e;
constexpr unsigned cbeq_direct = 0x31, ldhx_extended = 0x32, cbeq_offset = 0x61;
constexpr unsigned sthx_direct = 0x35, tst_direct = 0x3d, tst_offset = 0x6d;
constexpr unsigned cphx_direct = 0x75, dbnz_direct = 0x3b, dbnz_offset = 0x6b;
constexpr unsigned cphx_extended = 0x3e, nsa = 0x62, daa = 0x72, cphx_immediate = 0x65;
constexpr unsigned mov_immediate_direct = 0x6e, cbeq_indexed = 0x71, mov_indexed_direct = 0x7e;
constexpr unsigned dbnz_indexed = 0x7b, tst_indexed = 0x7d;
constexpr unsigned ldhx_indexed = 0xae, ldhx_offset16 = 0xbe, cphx_stack = 0xf3;
constexpr unsigned ldhx_offset8 = 0xce, ldhx_stack = 0xfe, sthx_stack = 0xff;

Hcs08InstructionTiming memory_form(const unsigned opcode) {
  // Family RM Table6-12: DIR, EXT, IX2, IX1, IX share ALU address costs.
  constexpr std::array<unsigned, 5> bytes{2, 3, 3, 2, 1};
  constexpr std::array<unsigned, 5> periods{3, 4, 4, 3, 3};
  constexpr unsigned first_row = 0xb;
  const auto row = opcode >> 4U;
  const auto low = opcode & nibble_mask;
  auto result = Hcs08InstructionTiming{.bytes = bytes.at(row - first_row),
                                       .periods = periods.at(row - first_row)};
  if (low == 0xd) { result.periods += 2; } // JSR, including indexed without offset.
  if (row == 0xf && (low == 7 || low == 0xf)) { --result.periods; } // STA/STX ,X.
  return result;
}

Hcs08InstructionTiming register_form(const unsigned opcode) {
  switch (opcode) {
    case cbeqa:
    case cbeqx: return {.bytes = 3, .periods = 4};          // CBEQA/X immediate plus relative.
    case mul: return {.bytes = 1, .periods = 5};            // MUL.
    case div: return {.bytes = 1, .periods = 6};            // DIV.
    case ldhx_immediate: return {.bytes = 3, .periods = 3}; // LDHX immediate.
    case ldhx_direct:                                       // LDHX direct shares DBNZA/X cost.
    case dbnza:
    case dbnzx: return {.bytes = 2, .periods = 4};             // DBNZA/X.
    case mov_direct_direct: return {.bytes = 3, .periods = 5}; // MOV direct to direct.
    case mov_direct_indexed:
      return {.bytes = 2, .periods = 5}; // MOV direct to indexed post-increment.
    default: return {.bytes = 1, .periods = 1};
  }
}

Hcs08InstructionTiming modify_form(const unsigned opcode) {
  // These are only rows3/6/7; the register rows4/5 have their own exceptions.
  switch (opcode) {
    case cbeq_direct:
    case ldhx_extended:
    case cbeq_offset: return {.bytes = 3, .periods = 5};
    case sthx_direct:
    case tst_direct:
    case tst_offset: return {.bytes = 2, .periods = 4};
    case cphx_direct: return {.bytes = 2, .periods = 5}; // CPHX direct reads both bytes.
    case dbnz_direct:
    case dbnz_offset: return {.bytes = 3, .periods = 7};
    case cphx_extended: return {.bytes = 3, .periods = 6};
    case nsa:
    case daa: return {.bytes = 1, .periods = 1};
    case cphx_immediate: return {.bytes = 3, .periods = 3};
    case mov_immediate_direct: return {.bytes = 3, .periods = 4};
    case cbeq_indexed:
    case mov_indexed_direct: return {.bytes = 2, .periods = 5};
    case dbnz_indexed: return {.bytes = 2, .periods = 6};
    case tst_indexed: return {.bytes = 1, .periods = 3};
    default:
      if ((opcode >> 4U) == 7) { return {.bytes = 1, .periods = 4}; }
      return {.bytes = 2, .periods = 5};
  }
}

Hcs08InstructionTiming control_form(const unsigned opcode) {
  constexpr unsigned first = 0x80;
  constexpr std::array<unsigned, 16> periods{9, 6, 5, 11, 1, 1, 3, 2, 3, 2, 3, 2, 1, 0, 0, 0};
  // 8D is undefined; STOP/WAIT's indefinite held states need a separate owner.
  const auto cost = periods.at(opcode - first);
  return {.bytes = cost == 0 ? 0U : 1U, .periods = cost};
}

Hcs08InstructionTiming primary(const unsigned opcode) {
  const auto row = opcode >> 4U;
  if (row == 0) { return {.bytes = 3, .periods = 5}; } // BRSET/BRCLR.
  if (row == 1) { return {.bytes = 2, .periods = 5}; } // BSET/BCLR.
  if (row == 2) { return {.bytes = 2, .periods = 3}; } // Relative branches.
  if (row >= 0xb) { return memory_form(opcode); }
  if (row == 4 || row == 5) { return register_form(opcode); }
  if (row == 3 || row == 6 || row == 7) { return modify_form(opcode); }
  if (row == 8) { return control_form(opcode); }
  if (row == 9) {
    constexpr unsigned last_branch = 0x93;
    constexpr unsigned txs = 0x94;
    constexpr unsigned tsx = 0x95;
    constexpr unsigned sthx = 0x96;
    if (opcode <= last_branch) { return {.bytes = 2, .periods = 3}; }
    if (opcode == txs || opcode == tsx) { return {.bytes = 1, .periods = 2}; }
    if (opcode == sthx) { return {.bytes = 3, .periods = 5}; }
    return {.bytes = 1, .periods = 1}; // Prefix9E is handled before this lookup.
  }
  constexpr unsigned undefined = 0xac;
  constexpr unsigned bsr = 0xad;
  if (opcode == undefined) { return {}; }
  return {.bytes = 2, .periods = opcode == bsr ? 5U : 2U}; // BSR or immediate ALU/AIS/AIX.
}

Hcs08InstructionTiming secondary(const unsigned opcode) {
  const auto row = opcode >> 4U;
  const auto low = opcode & nibble_mask;
  if ((row == 0xd || row == 0xe) && (low <= 0xb || low >= 0xe)) {
    return {.bytes = row == 0xd ? 4U : 3U, .periods = row == 0xd ? 5U : 4U};
  }
  if (row == 6 && low != 2 && low != 5 && low != 0xe) {
    if (low == 1) { return {.bytes = 4, .periods = 6}; }
    if (low == 0xb) { return {.bytes = 4, .periods = 8}; }
    return {.bytes = 3, .periods = low == 0xd ? 5U : 6U};
  }
  switch (opcode) {
    case ldhx_indexed: return {.bytes = 2, .periods = 5};  // LDHX ,X.
    case ldhx_offset16: return {.bytes = 4, .periods = 6}; // LDHX16,X.
    case cphx_stack: return {.bytes = 3, .periods = 6};    // CPHX8,SP.
    case ldhx_offset8:
    case ldhx_stack:
    case sthx_stack: return {.bytes = 3, .periods = 5}; // LDHX8,X and LDHX/STHX8,SP.
    default: return {};
  }
}

Hcs08InstructionTiming prefixed(const Bus &bus, const ExecutionContext context) {
  if (context.pc == address_space - 1) { throw Fault("instruction fetch wraps"); }
  bus.check_execute({.address = context.pc, .length = 2});
  const auto opcode = bus.peek(static_cast<std::uint16_t>(context.pc + 1));
  const auto result = secondary(opcode);
  if (result.bytes == 0) {
    throw Fault("unsupported prefix=" + hex(prefix_opcode) + " opcode=" + hex(opcode));
  }
  return result;
}
} // namespace

Hcs08InstructionTiming hcs08_instruction_timing(const Bus &bus,
                                                const ExecutionContext context) try {
  bus.check_execute({.address = context.pc, .length = 1});
  const auto opcode = bus.peek(context.pc);
  const auto result = opcode == prefix_opcode ? prefixed(bus, context) : primary(opcode);
  if (result.bytes == 0) { throw Fault("unsupported opcode=" + hex(opcode)); }
  if (result.bytes > address_space - context.pc) { throw Fault("instruction fetch wraps"); }
  bus.check_execute({.address = context.pc, .length = result.bytes});
  for (unsigned byte = 0; byte < result.bytes; ++byte) {
    bus.note_code_byte(static_cast<std::uint16_t>(context.pc + byte));
  }
  return result;
} catch (const Fault &error) {

  throw Fault("HCS08 instruction timing fetch address=" + hex(context.pc) +
              " pc=" + hex(context.pc) + " ccr=" + hex(context.ccr) + ": " + error.what());
}
} // namespace blofeld
