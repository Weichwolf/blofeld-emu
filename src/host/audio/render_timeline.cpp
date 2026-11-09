// SPDX-License-Identifier: GPL-3.0-only
#include "audio/render_timeline.hpp"

#include <cmath>
#include <optional>

namespace blofeld::host::audio {
std::optional<RenderTimeline::Fault> RenderTimeline::advance(const Request request) noexcept
    [[clang::nonblocking]] {
  if (failure_) { return failure_; }
  std::optional<Error> error;
  double next = 0;
  if (!request.sample || !std::isfinite(*request.sample)) {
    error = Error::sample_time;
  } else if (!request.host || *request.host == 0 || (host_ && *request.host <= *host_)) {
    error = Error::host_time;
  } else if (request.frames == 0) {
    error = Error::frame_count;
  } else if (next_ && *request.sample != *next_) {
    error = Error::discontinuity;
  } else {
    next = *request.sample + request.frames;
    if (!std::isfinite(next) || next - *request.sample != request.frames) {
      error = Error::exhausted;
    }
  }
  if (error) {
    failure_ =
        Fault{.error = *error, .request = request, .expected = next_, .previous_host = host_};
  } else {
    next_ = next;
    host_ = request.host;
  }
  return failure_;
}

void RenderTimeline::reset() noexcept {
  next_.reset();
  host_.reset();
  failure_.reset();
}
}
