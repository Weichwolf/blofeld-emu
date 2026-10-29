// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "board/reference_square_clock.hpp"
#include "dsp/dsp_dax.hpp"
#include "dsp/dsp_esai.hpp"

#include <cstdint>
#include <string>

namespace blofeld::test::square_clock {
// Directed reference wiring only: equal divisors do not prove a shared board net.
struct AudioRig {
  static constexpr unsigned transmit_clock = 0x640301;
  static constexpr unsigned transmit_format = 0x7d00;
  static constexpr unsigned external_256 = 8;
  static constexpr std::uint64_t core_hz = 54000000;
  reference::SquareClock source;
  DspEsai esai{DspEsaiInstance::secondary};
  DspDax dax;
  std::string pcm;
  std::string bmc;
  bool last_sck = false;

  explicit AudioRig(reference::SquareClockConfig config);
  void observe(bool level);
  void tick();
  void reset_endpoints();
  void initialize_endpoints();
};

void serial_checks();
void snapshot_checks();
}
