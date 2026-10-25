// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/serial_pcm.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <ostream>

namespace blofeld::test {
// Untimed native access admissions alternate with explicitly paused-core serial phases.
struct VoiceRepeat {
  unsigned entry = 0;
  unsigned half = 0;
  unsigned words = 0;
  unsigned frames = 0;
  std::uint64_t edge = 0;
  std::uint64_t cycle = 0;
  SerialPcmReceiver receiver{{.frame_sync_high = false}};
  std::array<DspDmaWord, 32> transfers{};
  std::array<SerialPcmFrame, 16> decoded{};

  void append(DspProgram &program);
  static void instruction(DspMachine &machine);
  void begin(DspBus &bus);
  void drain(DspBus &bus, const DspCore &cpu, std::ostream &trace);
  bool operator==(const VoiceRepeat &) const = default;

private:
  [[nodiscard]] DspDmaBusResult grant(DspBus &bus);
  [[nodiscard]] DspDmaBusResult step(DspBus &bus);
  void finish(DspBus &bus, std::uint64_t target);
  void verify(DspBus &bus) const;
  void emit(std::ostream &trace) const;
};
}
