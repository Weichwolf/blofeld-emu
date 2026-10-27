// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/host_audio.hpp"
#include "audio/pcm_sink.hpp"
#include "audio/serial_pcm.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>

namespace blofeld {
enum class AudioRuntimeState : std::uint8_t {
  connecting,
  ready,
  playing,
  finished,
  failed,
  retired,
  cancelled
};
enum class PcmSubmission : std::uint8_t { accepted, full, closed, failed, retired, exhausted };

struct AudioRuntimeOptions {
  HostAudioConfig output;
  std::size_t queue_frames = 4096;
};

// Independent observations of external transport, never a modeled checkpoint.
struct AudioRuntimeStatus {
  AudioRuntimeState state = AudioRuntimeState::connecting;
  std::uint64_t accepted = 0;
  std::uint64_t written = 0;
  std::uint64_t underflows = 0;
  bool waiting_for_source = false;
};

// One ordinary audio worker owns every native client call. One producer submits
// frames; try_submit is bounded/noexcept and never waits or allocates. check,
// accept and finish are ordinary emulation/control-thread operations, not RT.
class AudioRuntime final : public PcmFrameSink {
public:
  explicit AudioRuntime(AudioRuntimeOptions options);
  ~AudioRuntime() override;
  AudioRuntime(const AudioRuntime &) = delete;
  AudioRuntime &operator=(const AudioRuntime &) = delete;
  AudioRuntime(AudioRuntime &&) = delete;
  AudioRuntime &operator=(AudioRuntime &&) = delete;
  [[nodiscard]] PcmSubmission try_submit(const SerialPcmFrame &frame) noexcept;
  void accept(const SerialPcmFrame &frame) override;
  void check() const override;
  void invalidate() noexcept override;
  void finish();
  [[nodiscard]] AudioRuntimeStatus status() const noexcept;

private:
  struct Worker;
  std::unique_ptr<Worker> worker_;
};
}
