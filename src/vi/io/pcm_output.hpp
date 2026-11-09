// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <stop_token>

namespace blofeld::vi {
// Application-owned consumer. Submission happens outside native Core callbacks;
// it may apply backpressure and must honor stop. Accepted spans are fully consumed
// before returning. The host backend needs no knowledge of instrument registers.
struct PcmOutput {
  static constexpr unsigned time_hz = 60000000;
  using Frame = std::array<std::int32_t, 2>; // Signed, unshifted 24-bit PCM.

  struct Block {
    std::span<const Frame> frames;
    std::span<const std::uint64_t> ticks; // Instrument completion, not host presentation.
    std::uint64_t epoch = 0, first_frame = 0;
    unsigned reference_hz = 0, frame_ticks = 0;
    std::uint16_t underruns = 0; // One bit per hardware frame, including retained-data repeats.
  };

  void *owner = nullptr;
  bool (*write)(void *, Block, const std::stop_token &) = nullptr;
  // Runs on the execution owner after its final submission, including unwinding.
  void (*close)(void *) noexcept = nullptr;
};
}
