// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <optional>

namespace blofeld::host::audio {
class RenderTimeline {
public:
  enum class Error : std::uint8_t { sample_time, host_time, frame_count, discontinuity, exhausted };

  struct Request {
    std::optional<double> sample;
    std::optional<std::uint64_t> host;
    std::uint32_t frames = 0;
  };

  struct Fault {
    Error error;
    Request request;
    std::optional<double> expected;
    std::optional<std::uint64_t> previous_host;
  };

  [[nodiscard]] std::optional<Fault> advance(Request request) noexcept [[clang::nonblocking]];
  void reset() noexcept;

private:
  std::optional<double> next_;
  std::optional<std::uint64_t> host_;
  std::optional<Fault> failure_;
};
}
