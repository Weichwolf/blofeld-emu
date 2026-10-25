// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/serial_pcm.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <ostream>

namespace blofeld::test {
// Explicit standalone peripheral phases; the core is paused during each transfer.
struct VoicePcm {
  unsigned setup = 0;
  unsigned transfer = 0;
  unsigned enabled = 0;
  std::uint64_t edge = 0;
  std::uint64_t cycle = 0;
  unsigned block = 0;
  unsigned words = 0;
  unsigned frames = 0;
  SerialPcmReceiver receiver{{.frame_sync_high = false}};
  std::array<DspDmaWord, 32> transfers{};
  std::array<SerialPcmFrame, 16> decoded{};

  void append(DspProgram &program);
  void begin(DspBus &bus);
  void baseline(DspBus &bus);
  void drain(DspBus &bus, const DspCore &cpu, std::ostream &trace);
  bool operator==(const VoicePcm &) const = default;

private:
  [[nodiscard]] DspDmaBusResult grant(DspBus &bus);
  [[nodiscard]] DspDmaBusResult step(DspBus &bus);
  void finish(DspBus &bus, std::uint64_t target);
  void verify(DspBus &bus) const;
  void emit(std::ostream &trace) const;
};
}
