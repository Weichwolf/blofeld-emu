// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <atomic>
#include <cstdint>
#include <optional>

namespace blofeld::host::audio {
struct RenderPosition {
  std::uint64_t epoch = 0, first_frame = 0, host_ns = 0;
  std::uint32_t frames = 0;
  // Additional host-clock correlation error, not physical device latency.
  std::uint64_t uncertainty_ns = 0;
  bool operator==(const RenderPosition &) const = default;
};

// One writer, bounded concurrent reads; transfer writer ownership only while quiescent.
class RenderPositionStore {
public:
  [[nodiscard]] bool publish(RenderPosition position) noexcept [[clang::nonblocking]];
  [[nodiscard]] bool clear() noexcept [[clang::nonblocking]];
  [[nodiscard]] std::optional<RenderPosition> read() const noexcept;

private:
  std::atomic<std::uint64_t> version_{0}, epoch_{0}, first_frame_{0}, host_ns_{0},
      uncertainty_ns_{0};
  std::atomic<std::uint32_t> frames_{0};
};
}
