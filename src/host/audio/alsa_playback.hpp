// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/playback.hpp"

namespace blofeld::host::audio {
class AlsaPlayback {
public:
  AlsaPlayback(unsigned rate, std::string_view device);
  ~AlsaPlayback();
  AlsaPlayback(const AlsaPlayback &) = delete;
  AlsaPlayback &operator=(const AlsaPlayback &) = delete;

  [[nodiscard]] bool write(std::span<const Frame> frames, const std::stop_token &stop);
  [[nodiscard]] bool reset(const std::stop_token &stop);
  void stop() noexcept;
  void check() const;
  [[nodiscard]] const Grants &grants() const noexcept;
  [[nodiscard]] PlaybackStatus status() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
