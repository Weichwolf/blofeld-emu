// SPDX-License-Identifier: GPL-3.0-only
#include "audio/wav_consumer.hpp"

#include "audio/wav.hpp"
#include "io/pcm_output.hpp"

#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <stop_token>

namespace blofeld::app {
WavConsumer::WavConsumer(const std::filesystem::path &path, const unsigned rate)
    : wav_(path, rate), rate_(rate) {}

vi::PcmOutput WavConsumer::output() noexcept {
  return {.owner = this,
          .write =
              [](void *owner, const vi::PcmOutput::Block block, const std::stop_token &stop) {
                return static_cast<WavConsumer *>(owner)->write(block, stop);
              },
          .close =
              [](void *owner) noexcept {
                auto &consumer = *static_cast<WavConsumer *>(owner);
                if (consumer.phase_ == Phase::writing) { consumer.phase_ = Phase::closed; }
              }};
}

bool WavConsumer::write(const vi::PcmOutput::Block block, const std::stop_token &stop) {
  if (phase_ != Phase::writing) { throw std::logic_error("Offline PCM owner is not active"); }
  if (stop.stop_requested()) { return false; }
  phase_ = Phase::failed;
  if (block.frames.empty() || block.frames.size() > 16 || block.frame_ticks == 0 ||
      std::uint64_t{block.frame_ticks} * rate_ != block.reference_hz) {
    throw std::runtime_error("Unsupported original offline PCM block format/rate");
  }
  if (block.underruns != 0) { throw std::runtime_error("Original offline PCM underrun"); }
  if ((epoch_ && *epoch_ != block.epoch) || block.first_frame != wav_.frames()) {
    throw std::runtime_error("Original offline PCM timeline is discontinuous");
  }
  wav_.write(block.frames);
  epoch_ = block.epoch;
  phase_ = Phase::writing;
  return true;
}

void WavConsumer::finish() {
  if (phase_ != Phase::closed || !epoch_) {
    throw std::logic_error("Offline output requires a closed owner and valid original PCM");
  }
  phase_ = Phase::failed;
  wav_.finish();
  phase_ = Phase::finished;
}

std::uint64_t WavConsumer::frames() const noexcept {
  return wav_.frames();
}
}
