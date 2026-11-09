// SPDX-License-Identifier: GPL-3.0-only
#include "audio/pulse_position.hpp"

#include "audio/buffer.hpp"
#include "audio/render_position.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

namespace blofeld::host::audio {
namespace {
constexpr auto frame_bytes = static_cast<std::int64_t>(sizeof(Frame));
}

PulsePosition::PulsePosition(const std::uint64_t maximum_query_ns)
    : maximum_query_ns_(maximum_query_ns) {
  if (maximum_query_ns == 0) { throw std::invalid_argument("Zero native audio query bound"); }
}

void PulsePosition::reset(const std::int64_t origin_bytes) {
  if (origin_bytes < 0 || origin_bytes % frame_bytes != 0) {
    throw std::runtime_error("Invalid native audio epoch origin");
  }
  if (initialized_ && epoch_ == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("Native audio epoch exhausted");
  }
  if (initialized_) { ++epoch_; }
  origin_bytes_ = origin_bytes;
  received_ns_ = 0;
  initialized_ = true;
  failed_ = false;
  previous_.reset();
}

void PulsePosition::fail(const char *reason, const Snapshot snapshot) {
  failed_ = true;
  throw std::runtime_error(std::string("Native audio observation: ") + reason +
                           "; read-bytes=" + std::to_string(snapshot.read_bytes) +
                           " submitted-frames=" + std::to_string(snapshot.submitted_frames) +
                           " sent-ns=" + std::to_string(snapshot.sent_ns) +
                           " received-ns=" + std::to_string(snapshot.received_ns));
}

std::optional<RenderPosition> PulsePosition::observe(const Snapshot snapshot) {
  if (!initialized_ || failed_) { fail("epoch is unavailable", snapshot); }
  if (snapshot.sent_ns == 0 || snapshot.received_ns < snapshot.sent_ns ||
      snapshot.sent_ns < received_ns_ ||
      snapshot.received_ns - snapshot.sent_ns > maximum_query_ns_) {
    fail("native query is backwards, overlapping or overdue", snapshot);
  }
  received_ns_ = snapshot.received_ns;
  if (snapshot.corrupt) { fail("corrupt native PCM index", snapshot); }
  if (!snapshot.playing) {
    if (previous_) { fail("native consumption stopped", snapshot); }
    return {};
  }
  if (snapshot.read_bytes < origin_bytes_ || snapshot.read_bytes % frame_bytes != 0) {
    fail("native PCM index is before its epoch or unaligned", snapshot);
  }
  const auto consumed =
      static_cast<std::uint64_t>((snapshot.read_bytes - origin_bytes_) / frame_bytes);
  if (consumed > snapshot.submitted_frames) { fail("native PCM exceeds submitted data", snapshot); }
  if (consumed == 0) {
    if (previous_) { fail("native PCM index moved backwards", snapshot); }
    return {};
  }
  const auto first = consumed - 1;
  if (previous_ && first < previous_->first_frame) {
    fail("native PCM index moved backwards", snapshot);
  }
  // Repeated queries of one native index do not create another audio interval.
  if (previous_ && first == previous_->first_frame) { return {}; }
  const auto duration = snapshot.received_ns - snapshot.sent_ns;
  const auto midpoint = snapshot.sent_ns + (duration / 2);
  if (previous_ && midpoint <= previous_->host_ns) { fail("host time did not advance", snapshot); }
  const RenderPosition position{.epoch = epoch_,
                                .first_frame = first,
                                .host_ns = midpoint,
                                .frames = 1,
                                .uncertainty_ns = (duration / 2) + (duration % 2)};
  previous_ = position;
  return position;
}
}
