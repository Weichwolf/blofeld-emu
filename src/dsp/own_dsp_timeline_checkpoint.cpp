// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>

namespace blofeld {
struct OwnDspTimelineCheckpoint::Storage {
  std::optional<OwnDspFetchCheckpoint> fetch;
  std::optional<OwnDspCheckpoint> reset;
  OwnDspAutomaticTiming timing;
  std::uint64_t epoch_ticks;
  std::uint64_t expected_ticks;
  std::uint64_t expected_epoch;
  std::optional<std::uint64_t> expected_dma;
  std::uint64_t horizon;
  OwnDspBoundary boundary;
};

OwnDspTimelineCheckpoint::OwnDspTimelineCheckpoint(std::shared_ptr<const Storage> storage)
    : storage_(std::move(storage)) {}

OwnDspTimelineCheckpoint::~OwnDspTimelineCheckpoint() = default;

const OwnDspTimelineCheckpoint::Storage &
OwnDspTimeline::saved_state(const OwnDspTimelineCheckpoint &checkpoint) {
  if (!checkpoint.storage_) { throw std::invalid_argument("Empty own DSP timeline checkpoint"); }
  return *checkpoint.storage_;
}

OwnDspTimelineCheckpoint OwnDspTimeline::checkpoint() const {
  check_owner();
  return OwnDspTimelineCheckpoint(
      std::make_shared<const OwnDspTimelineCheckpoint::Storage>(OwnDspTimelineCheckpoint::Storage{
          .fetch = fetch_ ? std::optional{fetch_->checkpoint()} : std::nullopt,
          .reset = fetch_ ? std::nullopt : std::optional{machine_.checkpoint()},
          .timing = timing_,
          .epoch_ticks = epoch_ticks_,
          .expected_ticks = expected_ticks_,
          .expected_epoch = expected_epoch_,
          .expected_dma = expected_dma_,
          .horizon = horizon_,
          .boundary = boundary_}));
}

OwnDspTimeline::OwnDspTimeline(OwnDspMachine &machine, const OwnDspTimelineCheckpoint &checkpoint)
    : machine_(machine),
      timing_(saved_state(checkpoint).timing),
      epoch_ticks_(saved_state(checkpoint).epoch_ticks),
      expected_ticks_(saved_state(checkpoint).expected_ticks),
      expected_epoch_(saved_state(checkpoint).expected_epoch),
      expected_dma_(saved_state(checkpoint).expected_dma),
      horizon_(saved_state(checkpoint).horizon),
      boundary_(saved_state(checkpoint).boundary) {
  const auto &state = saved_state(checkpoint);
  if (state.fetch) {
    fetch_ = std::make_unique<OwnDspFetch>(machine_, *state.fetch);
  } else if (state.reset) {
    reset_identity_ = std::make_shared<const bool>(true);
    machine_.restore(*state.reset);
    machine_.captured_fetch_owner_ = reset_identity_;
  } else {
    throw std::invalid_argument("Own DSP timeline checkpoint has no execution or reset state");
  }
  check_owner();
}
}
