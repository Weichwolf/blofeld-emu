// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/wav.hpp"
#include "io/pcm_output.hpp"

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <optional>
#include <stop_token>

namespace blofeld::app {
// Read totals and finalize only after joining the DSP execution owner.
class WavConsumer {
public:
  WavConsumer(const std::filesystem::path &path, unsigned rate, bool deferred = false);
  // Control owner publishes boundaries before releasing the MIDI admission frontier.
  void begin_at(std::uint64_t ticks);
  void end_at(std::uint64_t ticks);
  [[nodiscard]] bool has_pcm() const noexcept;
  [[nodiscard]] bool complete() const noexcept;
  [[nodiscard]] vi::PcmOutput output() noexcept;
  void finish();
  [[nodiscard]] std::uint64_t frames() const noexcept;
  // Validated source frames, including discarded startup and post-window PCM.
  [[nodiscard]] std::uint64_t source_frames() const noexcept;

private:
  enum class Phase : std::uint8_t { writing, closed, failed, finished };
  bool write(vi::PcmOutput::Block block, const std::stop_token &stop);
  void validate_time(vi::PcmOutput::Block block);
  host::audio::Wav wav_;
  unsigned rate_;
  bool deferred_;
  std::optional<std::uint64_t> epoch_, first_tick_, last_tick_;
  std::uint64_t input_frames_ = 0;
  std::atomic<std::uint64_t> begin_{0}, end_{std::numeric_limits<std::uint64_t>::max()};
  std::atomic<std::uint64_t> observed_{0};
  std::atomic<bool> pcm_{false}, complete_{false};
  Phase phase_ = Phase::writing;
};
}
