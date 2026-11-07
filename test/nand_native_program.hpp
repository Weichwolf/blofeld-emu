// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp_core_fixture.hpp"
#include "dsp_rom_program.hpp"
#include "nand_gpio_pins.hpp"

#include <string>

namespace blofeld::test::nand_native {
inline constexpr unsigned word_mask = 0xffffff, byte_mask = 0xff, carry = 1;
inline constexpr unsigned masked_status = 0xc00300;

inline void require(const bool condition, const std::string &reason) {
  if (!condition) { throw Fault(reason); }
}

inline void
movep(core::Program &program, const unsigned value, const unsigned address, const char space) {
  require(space == 'x' || space == 'y', "Authored NAND caller uses only X/Y peripheral writes");
  program.write(
      {.space = space == 'x' ? DspSpace::x : DspSpace::y, .address = address, .value = value});
}

inline void setup_gpio(core::Program &program) {
  program.immediate(DspRomRegister::sr, masked_status);
  movep(program, port_f_bus, bus_control, 'x');
  movep(program, nand_controls, port_c_direction, 'x');
  movep(program, nand_data, port_c_control, 'x');
  movep(program, idle_c, port_c_data, 'x');
  movep(program, output_f, port_f_direction, 'y');
  movep(program, 0, port_f_data, 'y');
  movep(program, nand_other_f, port_f_control, 'y');
}

struct Latch {
  unsigned value;
  unsigned mode;
};

inline void latch(core::Program &program, const Latch input) {
  movep(program, input.value, port_f_data, 'y');
  movep(program, idle_c | input.mode, port_c_data, 'x');
  const DspAddress we{.space = DspSpace::x, .address = port_c_data};
  program.bit({.operand = we, .index = 9, .action = core::BitAction::clear});
  program.bit({.operand = we, .index = 9, .action = core::BitAction::set});
}

inline void read_byte(core::Program &program) {
  const DspAddress re{.space = DspSpace::x, .address = port_c_data};
  program.bit({.operand = re, .index = 8, .action = core::BitAction::clear});
  // FM MOVEP Y:pp,A: pp=$35, full accumulator selector14, direction=read.
  constexpr unsigned read_f_to_a = 0x094e35;
  // FM MOVE A1,X:(R0)+: selector12, MMM011, RRR000.
  constexpr unsigned store_a1_post = 0x545800;
  program.emit(read_f_to_a);
  program.emit(store_a1_post);
  program.bit({.operand = re, .index = 8, .action = core::BitAction::set});
}

inline void poll_ready(core::Program &program) {
  constexpr unsigned timer1_control = 0xffff8b;
  program.branch({.operand = {.space = DspSpace::x, .address = timer1_control},
                  .index = 12,
                  .target = 0,
                  .kind = core::BranchKind::relative_clear});
}

inline void reset(DspMachine &machine) {
  const core::Driver driver(machine);
  require(driver.state().pc == DspRomProgram::boot,
          "Actual native reset selects the authored caller");
}
}
