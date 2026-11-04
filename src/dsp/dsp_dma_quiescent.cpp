// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_dma_registers.hpp"

#include <algorithm>
#include <string>

namespace blofeld {
bool DspDma::quiescent() const noexcept {
  if (active_) { return false; }
  return std::ranges::none_of(channels_, [](const Channel &channel) {
    return channel.pending || channel.settling != 0 ||
           ((channel.control & dma::enable) != 0 && channel.first_sample);
  });
}

bool DspDma::request_levels_held(const DspDmaInputs inputs) const noexcept {
  return inputs.known_requests == known_requests_ && inputs.requests == requests_;
}

void DspDma::advance_quiescent(const DspDmaInputs inputs) {
  const bool valid = inputs.timing == DspDmaTiming::reference_sampled_cycles &&
                     (inputs.grant == DspDmaGrant::available || inputs.grant == DspDmaGrant::wait);
  const bool forward = cycle_ && inputs.cycle > *cycle_;
  const bool held = inputs.known_requests == known_requests_ && inputs.requests == requests_;
  if (!valid || !forward || !held || !quiescent()) {
    throw Fault("DSP DMA quiescent span at X:" + hex(dma::status, 6) +
                " final-cycle=" + std::to_string(inputs.cycle) +
                ": requires a sampled held-request baseline and no pending transfer/transition");
  }
  // Ordinary sample() already validated the held levels. With no transition,
  // repeated samples change only the consecutive-cycle cursor, including when
  // an enabled channel is waiting for an actual board-owned request edge.
  cycle_ = inputs.cycle;
}
}
