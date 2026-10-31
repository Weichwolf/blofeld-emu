// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/serial_pcm.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace blofeld::test {
struct DspPcmConfig {
  std::uint64_t timebase = 0;
  std::uint64_t baseline = 0;
  std::uint64_t start = 0;
  std::uint64_t half_period = 0;
  std::uint64_t observations = 0;
  bool secondary = false;
};

// Trusted in-process fixture state. Host audio sessions are not part of it.
struct DspPcmSnapshot {
  DspPcmConfig config;
  OwnDspTimelineCheckpoint timeline;
  SerialPcmReceiver receiver;
  std::vector<SerialPcmFrame> frames;
  std::uint64_t cursor;
  std::uint64_t cycles;
  std::uint64_t horizon;
  unsigned events_in_period;
};

// Real authored DSP execution and finite external ESAI clock observations.
// Reference effect periods and input-first ties are explicit fixture choices.
class DspPcmRun {
public:
  explicit DspPcmRun(DspPcmConfig config);
  explicit DspPcmRun(const DspPcmSnapshot &saved);
  void advance_to(std::uint64_t horizon, unsigned maximum_cycles);
  [[nodiscard]] DspPcmSnapshot checkpoint() const;

  [[nodiscard]] const OwnDspMachine &machine() const { return machine_; }

  [[nodiscard]] const SerialPcmReceiver &receiver() const { return receiver_; }

  [[nodiscard]] const std::vector<SerialPcmFrame> &frames() const { return frames_; }

  [[nodiscard]] std::uint64_t observed() const { return cursor_; }

  [[nodiscard]] std::uint64_t next_deadline() const { return timeline_->next_deadline(); }

private:
  [[nodiscard]] std::optional<std::uint64_t> next_input() const;
  void observe(std::uint64_t ticks);
  void drain(std::uint64_t through);
  void check_native(const std::optional<dsp56300::InstructionProgress> &progress) const;
  DspPcmConfig config_;
  OwnDspMachine machine_;
  std::unique_ptr<OwnDspTimeline> timeline_;
  SerialPcmReceiver receiver_{SerialPcmFormat{}};
  std::vector<SerialPcmFrame> frames_;
  std::uint64_t cursor_ = 0;
  std::uint64_t cycles_ = 0;
  std::uint64_t horizon_ = 0;
  unsigned events_in_period_ = 0;
  bool faulted_ = false;
};
}
