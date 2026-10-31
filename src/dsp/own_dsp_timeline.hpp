// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_dma.hpp"
#include "dsp/own_dsp_fetch.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
class OwnDspMachine;

enum class OwnDspBoundary : std::uint8_t { before, through };

// Orders completed reference cycles against a caller-owned common timebase.
// External events and their ordering at equal timestamps remain board-owned.
// Reset, checkpoint restoration and direct native execution invalidate the owner.
class OwnDspTimeline {
public:
  OwnDspTimeline(OwnDspMachine &machine, OwnDspAutomaticTiming timing, std::uint64_t epoch_ticks);
  OwnDspTimeline(const OwnDspTimeline &) = delete;
  OwnDspTimeline &operator=(const OwnDspTimeline &) = delete;
  [[nodiscard]] std::uint64_t next_deadline() const;
  [[nodiscard]] std::optional<OwnDspFetchCycleResult>
  advance_one(std::uint64_t horizon, OwnDspBoundary boundary, DspDmaInputs inputs);

  [[nodiscard]] bool faulted() const { return faulted_; }

private:
  void check_owner() const;
  OwnDspMachine &machine_;
  OwnDspFetch fetch_;
  OwnDspAutomaticTiming timing_;
  std::uint64_t epoch_ticks_;
  std::uint64_t expected_ticks_;
  std::uint64_t expected_epoch_;
  std::optional<std::uint64_t> expected_dma_;
  std::uint64_t horizon_;
  OwnDspBoundary boundary_ = OwnDspBoundary::before;
  bool faulted_ = false;
};
}
