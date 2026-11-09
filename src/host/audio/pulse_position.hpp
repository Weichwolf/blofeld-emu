// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/render_position.hpp"

#include <cstdint>
#include <optional>

namespace blofeld::host::audio {
// Correlate a fresh server consumption snapshot with its bounded native query.
// The midpoint is an estimate within that query, not a hardware sample timestamp.
class PulsePosition {
public:
  struct Snapshot {
    std::uint64_t sent_ns = 0, received_ns = 0, submitted_frames = 0;
    std::int64_t read_bytes = 0;
    bool playing = false, corrupt = false;
  };

  explicit PulsePosition(std::uint64_t maximum_query_ns);
  void reset(std::int64_t origin_bytes);
  [[nodiscard]] std::optional<RenderPosition> observe(Snapshot snapshot);

private:
  [[noreturn]] void fail(const char *reason, Snapshot snapshot);
  const std::uint64_t maximum_query_ns_;
  std::int64_t origin_bytes_ = 0;
  std::uint64_t epoch_ = 0, received_ns_ = 0;
  bool initialized_ = false, failed_ = false;
  std::optional<RenderPosition> previous_;
};
}
