// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/wav.hpp"
#include "io/pcm_output.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <stop_token>

namespace blofeld::app {
// Read totals and finalize only after joining the DSP execution owner.
class WavConsumer {
public:
  WavConsumer(const std::filesystem::path &path, unsigned rate);
  [[nodiscard]] vi::PcmOutput output() noexcept;
  void finish();
  [[nodiscard]] std::uint64_t frames() const noexcept;

private:
  enum class Phase : std::uint8_t { writing, closed, failed, finished };
  bool write(vi::PcmOutput::Block block, const std::stop_token &stop);
  host::audio::Wav wav_;
  unsigned rate_;
  std::optional<std::uint64_t> epoch_;
  Phase phase_ = Phase::writing;
};
}
