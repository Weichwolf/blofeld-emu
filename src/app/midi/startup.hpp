// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "io/midi_port.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace blofeld::app {
// Probe the ordinary bidirectional DIN path; replies always come from firmware.
class MidiStartup {
public:
  struct Readiness {
    bool application = false, pcm = false;
  };

  struct Observations {
    std::uint64_t queries = 0, output_bytes = 0;
    bool identity = false;
  };

  [[nodiscard]] Observations observations() const noexcept { return observations_; }

  [[nodiscard]] std::optional<std::uint64_t> poll(vi::MidiPort &port, Readiness readiness);

private:
  void receive(std::uint8_t byte) noexcept;
  std::array<std::uint8_t, 15> reply_{};
  std::size_t used_ = 0;
  Observations observations_;
  std::optional<std::uint64_t> origin_;
};
}
