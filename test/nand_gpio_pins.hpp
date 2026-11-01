// SPDX-License-Identifier: GPL-3.0-only
#pragma once

namespace blofeld::test {
inline constexpr unsigned nand_controls = 0xf00;
inline constexpr unsigned nand_re = 0x100;
inline constexpr unsigned nand_we = 0x200;
inline constexpr unsigned nand_cle = 0x400;
inline constexpr unsigned nand_ale = 0x800;
inline constexpr unsigned nand_data = 0xff;
inline constexpr unsigned nand_other_f = 0x300;
inline constexpr unsigned nand_timer_one = 1;

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

}
