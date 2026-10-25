// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "keyboard/hcs08_run_clock.hpp"

#include <cstdint>

namespace blofeld {
namespace {
constexpr unsigned ldx_immediate = 0xae, lda_immediate = 0xa6, sta_extended = 0xc7;
constexpr unsigned txs = 0x94, nop = 0x9d, bgnd = 0x82, space_bytes = 65536;
}

Hcs08InstructionTiming hcs08_instruction_timing(const Bus &bus,
                                                const ExecutionContext context) try {
  bus.check_execute({.address = context.pc, .length = 1});
  Hcs08InstructionTiming result;
  const auto opcode = bus.peek(context.pc);
  switch (opcode) {
    case ldx_immediate:
    case lda_immediate: result = {.bytes = 2, .periods = 2}; break;
    case sta_extended: result = {.bytes = 3, .periods = 4}; break;
    case txs: result = {.bytes = 1, .periods = 2}; break;
    case nop: result = {.bytes = 1, .periods = 1}; break;
    case bgnd: result = {.bytes = 1, .periods = 5}; break; // Plus the separately held BDC interval.
    default: throw Fault("unsupported opcode=" + hex(opcode));
  }
  if (result.bytes > space_bytes - context.pc) { throw Fault("instruction fetch wraps"); }
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
