// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp_norm_fixture.hpp"
#include "nand_gpio_fixture.hpp"

#include <string>

namespace blofeld::test {
inline constexpr unsigned port_c_data = 0xffffbd;
inline constexpr unsigned port_c_direction = 0xffffbe;
inline constexpr unsigned port_c_control = 0xffffbf;
inline constexpr unsigned port_f_data = 0xfffff5;
inline constexpr unsigned port_f_direction = 0xfffff6;
inline constexpr unsigned port_f_control = 0xfffff7;
inline constexpr unsigned bus_control = 0xfffffb;
inline constexpr unsigned port_f_bus = 0x01ffff;
inline constexpr unsigned output_f = 0x4ff;
inline constexpr unsigned input_f = 0x400;
inline constexpr unsigned all_inputs_f = 0x3ff;
inline constexpr unsigned idle_c = 0x300;
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
