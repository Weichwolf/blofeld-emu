// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/hcs08.hpp"
#include "blofeld/hcs08_bdc.hpp"

#include <cstdint>

namespace blofeld {
namespace {
constexpr unsigned enable_mask = 0x80, breakpoint_mask = 0x20, writable_mask = 0x98;
constexpr unsigned byte_mask = 255, high_shift = 8;
}

std::uint16_t Hcs08Bdc::argument() const {
  return static_cast<std::uint16_t>((static_cast<unsigned>(state_.frame[1]) << high_shift) |
                                    state_.frame[2]);
}

void Hcs08Bdc::reply_byte(const std::uint8_t byte) {
  state_.reply = {byte, 0};
  state_.reply_size = 1;
  state_.reply_cursor = 0;
}

void Hcs08Bdc::reply_word(const std::uint16_t word) {
  state_.reply = {static_cast<std::uint8_t>(word >> high_shift),
                  static_cast<std::uint8_t>(word & byte_mask)};
  state_.reply_size = 2;
  state_.reply_cursor = 0;
}

void Hcs08Bdc::control_write() {
  const auto value = state_.frame[1];
  if ((value & breakpoint_mask) != 0) {
    fail("WRITE_CONTROL", "breakpoint ownership is unsupported");
  }
  const auto enabled = state_.active ? state_.control & enable_mask : value & enable_mask;
  state_.control = static_cast<std::uint8_t>((value & (writable_mask & ~enable_mask)) | enabled);
}

void Hcs08Bdc::register_access() {
  if (!state_.active) { fail("register access", "command requires active background mode"); }
  auto cpu = cpu_.state();
  switch (state_.frame[0]) {
    case hcs08_bdc::write_a: cpu.a = state_.frame[1]; break;
    case hcs08_bdc::write_ccr:
      cpu.ccr = state_.frame[1];
      cpu.ccr_known = Hcs08State::all_ccr;
      break;
    case hcs08_bdc::write_pc:
      cpu.pc = argument();
      cpu.pc_known = true;
      break;
    case hcs08_bdc::write_hx: cpu.hx = argument(); break;
    case hcs08_bdc::write_sp: cpu.sp = argument(); break;
    case hcs08_bdc::read_a: reply_byte(cpu.a); return;
    case hcs08_bdc::read_ccr:
      if (cpu.ccr_known != Hcs08State::all_ccr) {
        fail("READ_CCR", "unknown CPU CCR bits mask=" + hex(cpu.ccr_known));
      }
      reply_byte(cpu.ccr);
      return;
    case hcs08_bdc::read_pc:
      if (!cpu.pc_known) { fail("READ_PC", "unknown CPU PC"); }
      reply_word(cpu.pc);
      return;
    case hcs08_bdc::read_hx: reply_word(cpu.hx); return;
    case hcs08_bdc::read_sp: reply_word(cpu.sp); return;
    default: fail("register access", "unsupported register command");
  }
  cpu_.set_state(cpu);
}

void Hcs08Bdc::execute(const Hcs08BdcGrant grant) {
  switch (state_.frame[0]) {
    case hcs08_bdc::read_status:
      reply_byte(static_cast<std::uint8_t>(state_.control | (state_.active ? 0x40U : 0U)));
      return;
    case hcs08_bdc::write_control: control_write(); return;
    case hcs08_bdc::background:
      if ((state_.control & enable_mask) != 0 && !state_.active &&
          !memory_.secured({.pc = cpu_.state().pc, .ccr = cpu_.state().ccr})) {
        state_.active = true;
      }
      return;
    case hcs08_bdc::go:
      if (!state_.active) { fail("GO", "command requires active background mode"); }
      if (!cpu_.state().pc_known) { fail("GO", "unknown CPU PC; WRITE_PC is required"); }
      cpu_.set_state(cpu_.state()); // Resume genuine registers and clear the consumed BGND request.
      state_.active = false;
      return;
    case hcs08_bdc::write_byte:
    case hcs08_bdc::read_byte: {
      if (!state_.active && grant != Hcs08BdcGrant::reference_instruction_boundary) {
        fail("memory access",
             "running-target bus arbitration is unqualified address=" + hex(argument()));
      }
      const auto cpu = cpu_.state();
      const ExecutionContext context{.pc = cpu.pc, .ccr = cpu.ccr};
      try {
        if (state_.frame[0] == hcs08_bdc::write_byte) {
          memory_.write({.address = argument(), .value = state_.frame[3]}, context);
        } else {
          reply_byte(memory_.read(argument(), context));
        }
      } catch (const Fault &error) {
        if (!cpu.pc_known) { fail("memory access", error.what()); }
        throw;
      }
      return;
    }
    default: register_access(); return;
  }
}
} // namespace blofeld
