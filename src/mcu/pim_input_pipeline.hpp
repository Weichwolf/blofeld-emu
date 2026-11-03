// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "board/gpio_pads.hpp"

#include <cstdint>

namespace blofeld {

// Functional candidate: advance once per MCU bus edge, independently of reads
// and physical pad changes. This is not a measured silicon pipeline depth.
class PimInputPipeline {
public:
  void advance(const GpioPadSample physical) noexcept {
    sampled_ = input_;
    input_ = physical;
  }

  void reset() noexcept {
    input_ = {};
    sampled_ = {};
  }

  [[nodiscard]] GpioPadSample sample(const std::uint8_t mask) const noexcept {
    const auto valid =
        static_cast<unsigned>(sampled_.known) & ~static_cast<unsigned>(sampled_.conflicts) & mask;
    return {
        .known = valid, .levels = sampled_.levels & valid, .conflicts = sampled_.conflicts & mask};
  }

private:
  GpioPadSample input_;
  GpioPadSample sampled_;
};

}
