// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "dsp/dsp_interrupts.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"

#include <string>
#include <string_view>

namespace blofeld::test::timer_native {
// FM MOVE immediate R2/R3/R7, MOVEC immediate SR and one-word foreground MOVE.
inline constexpr unsigned move_sr = 0x05f439, move_r2 = 0x62f400, move_r3 = 0x63f400;
inline constexpr unsigned move_r7 = 0x67f400, short_r2 = 0x320700, short_r3 = 0x330800;
inline constexpr unsigned foreground_move = 0x370900;
inline constexpr unsigned effect_bound = 32;

inline void require(const bool condition, const std::string_view message) {
  if (!condition) { throw Fault(std::string(message)); }
}

inline void sample(DspTimers &timers, const unsigned index, const bool level) {
  timers.set_tio_drive(index, level);
  timers.observe_synchronized_tio(index, level);
}

void install_vector(OwnDspMachine &machine, unsigned vector, unsigned opcode);
// Untimed staged vector service. The caller supplies the actually installed
// foreground opcode and independently checks register/source effects afterward.
void service(OwnDspMachine &machine,
             DspInterruptRequest request,
             dsp56300::InstructionWords foreground);
}
