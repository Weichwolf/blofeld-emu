// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp_norm_fixture.hpp"
// Retain the existing public pin-name interface for fixture consumers.
#include "nand_gpio_pins.hpp" // IWYU pragma: export

#include <string>

namespace blofeld::test {
inline constexpr unsigned byte_mask = 0xff;

inline void
movep(DspProgram &program, const unsigned value, const unsigned address, const char space) {
  program.emit("movep #>" + number(value) + ',' + space + ":<<" + number(address));
}

inline void setup_gpio(DspProgram &program) {
  program.immediate("sr", masked_status);
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

inline void latch(DspProgram &program, const Latch input) {
  movep(program, input.value, port_f_data, 'y');
  movep(program, idle_c | input.mode, port_c_data, 'x');
  program.emit("bclr #9,x:<<" + number(port_c_data));
  program.emit("bset #9,x:<<" + number(port_c_data));
}

}
