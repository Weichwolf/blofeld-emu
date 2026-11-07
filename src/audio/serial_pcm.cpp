// SPDX-License-Identifier: GPL-3.0-only
#include "audio/serial_pcm.hpp"

#include "common/bus.hpp"

namespace blofeld {
void validate_pcm_format(const SerialPcmFormat format) {
  constexpr unsigned maximum_bits = 32;
  if (format.slot_bits == 0 || format.slot_bits > maximum_bits || format.word_bits == 0 ||
      format.word_bits > format.slot_bits || format.channels < 2 ||
      format.channels > SerialPcmFrame::maximum_channels ||
      (format.sample_edge != SerialPcmEdge::rising &&
       format.sample_edge != SerialPcmEdge::falling)) {
    throw Fault("Invalid serial PCM format");
  }
}
}
