// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/serial_pcm.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace blofeld {
// Offline bounded recording. Copies share immutable history until append.
// Not an RT callback: construction/append may allocate and throw.
class PcmRecording {
public:
  static constexpr std::size_t maximum_capacity = 1048576;
  PcmRecording() = default;
  PcmRecording(SerialPcmFormat format, std::size_t capacity);
  PcmRecording(const PcmRecording &) = default;
  PcmRecording &operator=(const PcmRecording &) = default;
  void append(const SerialPcmFrame &frame);

  // A borrowed view; keep a recording copy if it must survive an append.
  [[nodiscard]] std::span<const SerialPcmFrame> frames() const {
    return frames_ ? std::span<const SerialPcmFrame>(*frames_) : std::span<const SerialPcmFrame>{};
  }

  explicit operator bool() const noexcept { return static_cast<bool>(frames_); }

  [[nodiscard]] const SerialPcmFormat &format() const { return format_; }

  [[nodiscard]] std::size_t capacity() const { return capacity_; }

  bool operator==(const PcmRecording &other) const;

private:
  SerialPcmFormat format_;
  std::size_t capacity_ = 0;
  std::int64_t limit_ = 0;
  std::shared_ptr<std::vector<SerialPcmFrame>> frames_;
};
}
