// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <ostream>
#include <string>

namespace blofeld::test {
// Explicit paused-core ACI observations; no automatic clock/core scheduling.
struct VoiceDax {
  bool enabled = false;
  unsigned words = 0;
  unsigned total_words = 0;
  unsigned caller_return = 0;
  std::uint64_t edge = 0;
  std::uint64_t original_begin = 0;
  std::uint64_t original_steps = 0;
  std::array<DspDmaWord, 32> transfers{};
  std::string bmc;

  void append(DspProgram &program);
  void begin(DspBus &bus);
  void observe(DspBus &bus, std::uint64_t at);
  void accept(const DspDmaWord &word);
  void verify(DspBus &bus, unsigned half, const std::array<DspDmaWord, 32> &primary) const;
  void emit(std::ostream &trace) const;
  void finish(DspBus &bus, unsigned halves, std::ostream &trace);
  bool operator==(const VoiceDax &) const = default;
};
} // namespace blofeld::test
