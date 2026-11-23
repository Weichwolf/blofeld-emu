// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "audio/pcm_frame.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace blofeld::host::audio {
using FloatFrame = std::array<float, 2>;

class RenderBuffer {
public:
  explicit RenderBuffer(std::size_t capacity);
  [[nodiscard]] std::size_t produce(std::span<const Frame> frames);
  [[nodiscard]] std::size_t render(std::span<FloatFrame> output) noexcept [[clang::nonblocking]];
  [[nodiscard]] std::size_t render(std::span<float> left, std::span<float> right) noexcept
      [[clang::nonblocking]];
  [[nodiscard]] std::size_t queued() const noexcept;

  [[nodiscard]] std::size_t capacity() const noexcept { return frames_.size(); }

  void discard() noexcept;

private:
  template <typename Store>
  [[nodiscard]] std::size_t render_frames(std::size_t frames, Store store) noexcept;
  std::vector<Frame> frames_;
  std::atomic<std::uint64_t> head_{0}, tail_{0};
};
}
