// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_esai.hpp"
#include "esai_fixture.hpp"

#include <string>

namespace blofeld::test::esai_clock {
inline constexpr unsigned output_directions = 0x600000;
inline constexpr unsigned two_slots = 0x200;
inline constexpr unsigned bypass = 0x100;
inline constexpr unsigned base_clock = output_directions | two_slots | bypass | 1U;
inline constexpr unsigned word32 = 0x7d00;
inline constexpr unsigned bit24 = 0xf900;
inline constexpr unsigned clock_polarity = 0x40000;
inline constexpr unsigned frame_polarity = 0x80000;
inline constexpr unsigned high_polarity = 0x100000;
inline constexpr unsigned personal_reset = 0x80000;
inline constexpr auto timing = DspEsaiClockTiming::reference_reset_edges;

inline void level(DspEsai &serial, const bool clock) {
  serial.observe({.high_clock = clock, .timing = timing});
}

inline void cycle(DspEsai &serial) {
  level(serial, true);
  level(serial, false);
}

inline void
configure(DspEsai &serial, const unsigned clock = base_clock, const unsigned format = word32) {
  level(serial, false);
  serial.write(esai::clock, clock);
  serial.write(esai::control, format);
  serial.set_active(true);
}

inline std::string word(const bool first, const bool padded) {
  if (padded) {
    return first ? "10100101110000111001111111111111" : "00010010001101000101011000000000";
  }
  return first ? "101001011100001110011111" : "000100100011010001010110";
}

void native_clock();
}
