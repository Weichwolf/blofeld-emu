// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_dma.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"

#include <cstdint>
#include <memory>
#include <optional>

namespace blofeld {
enum class OwnDspBoundary : std::uint8_t { before, through };

class OwnDspTimelineCheckpoint {
public:
  OwnDspTimelineCheckpoint() = default;
  ~OwnDspTimelineCheckpoint();
  OwnDspTimelineCheckpoint(const OwnDspTimelineCheckpoint &) = default;
  OwnDspTimelineCheckpoint &operator=(const OwnDspTimelineCheckpoint &) = default;

private:
  friend class OwnDspTimeline;
  struct Storage;
  explicit OwnDspTimelineCheckpoint(std::shared_ptr<const Storage> storage);
  std::shared_ptr<const Storage> storage_;
};

// Orders completed reference cycles against a caller-owned common timebase.
// External events and their ordering at equal timestamps remain board-owned.
// Owns chip execution and reset/time controls until destruction or chip-only
// restoration. Direct unclocked execution is rejected while this owner lives.
class OwnDspTimeline {
public:
  OwnDspTimeline(OwnDspMachine &machine,
                 OwnDspAutomaticTiming timing,
                 std::uint64_t epoch_ticks,
                 std::optional<OwnDspResetInputs> initial_reset = std::nullopt);
  // Restore chip/frontend/time together into a staged target. Reconnect external
  // pad readers and board devices before execution; target may change on failure.
  OwnDspTimeline(OwnDspMachine &machine, const OwnDspTimelineCheckpoint &checkpoint);
  OwnDspTimeline(const OwnDspTimeline &) = delete;
  OwnDspTimeline &operator=(const OwnDspTimeline &) = delete;
  // Reset time uses the same common epoch; execution begins after recovery.
  // Assert at the current completed cycle/effect boundary; retain lifetime time.
  // Uncompleted effects are discarded by actual native reset, not retired.
  void begin_hardware_reset(OwnDspResetInputs inputs);
  std::uint64_t advance_reset_to(std::uint64_t horizon);
  void release_hardware_reset();
  [[nodiscard]] std::uint64_t next_deadline() const;
  [[nodiscard]] std::optional<OwnDspFetchCycleResult>
  advance_one(std::uint64_t horizon, OwnDspBoundary boundary, DspDmaInputs inputs);

  [[nodiscard]] bool faulted() const { return faulted_; }

  // Read-only ownership check, including held reset and chip-only restoration.
  void validate_boundary() const { check_owner(); }

  [[nodiscard]] OwnDspTimelineCheckpoint checkpoint() const;

private:
  [[nodiscard]] static const OwnDspTimelineCheckpoint::Storage &
  saved_state(const OwnDspTimelineCheckpoint &checkpoint);
  void check_owner() const;
  [[nodiscard]] std::uint64_t cycle_deadline() const;
  void start_reset(OwnDspResetInputs inputs);
  OwnDspMachine &machine_;
  std::shared_ptr<const bool> cycle_identity_;
  std::unique_ptr<OwnDspFetch> fetch_;
  std::shared_ptr<const bool> reset_identity_;
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
