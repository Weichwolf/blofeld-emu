// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/buffer.hpp"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>

namespace blofeld::host::audio {
class Wav {
public:
  Wav(const std::filesystem::path &path, unsigned rate);
  ~Wav();
  Wav(const Wav &) = delete;
  Wav &operator=(const Wav &) = delete;
  void write(std::span<const Frame> frames);
  void finish();
  [[nodiscard]] std::uint64_t frames() const noexcept;

private:
  struct State;
  std::unique_ptr<State> state_;
};
}
