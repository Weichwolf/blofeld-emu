// SPDX-License-Identifier: GPL-3.0-only
#include "audio/render_position.hpp"

#include <atomic>
#include <cstdint>
#include <limits>
#include <optional>

namespace blofeld::host::audio {
static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
static_assert(std::atomic<std::uint32_t>::is_always_lock_free);

bool RenderPositionStore::publish(const RenderPosition position) noexcept [[clang::nonblocking]] {
  const auto version = version_.load();
  if (version > std::numeric_limits<std::uint64_t>::max() - 2) { return false; }
  // One SC order makes both version checks enclose every atomic payload access.
  version_.store(version + 1);
  epoch_.store(position.epoch);
  first_frame_.store(position.first_frame);
  host_ns_.store(position.host_ns);
  frames_.store(position.frames);
  uncertainty_ns_.store(position.uncertainty_ns);
  version_.store(version + 2);
  return true;
}

bool RenderPositionStore::clear() noexcept [[clang::nonblocking]] {
  return publish({});
}

std::optional<RenderPosition> RenderPositionStore::read() const noexcept {
  constexpr unsigned attempts = 3;
  for (unsigned attempt = 0; attempt < attempts; ++attempt) {
    const auto version = version_.load();
    if (version == 0) { return {}; }
    if ((version & 1U) != 0) { continue; }
    const RenderPosition position{.epoch = epoch_.load(),
                                  .first_frame = first_frame_.load(),
                                  .host_ns = host_ns_.load(),
                                  .frames = frames_.load(),
                                  .uncertainty_ns = uncertainty_ns_.load()};
    if (version_.load() == version) {
      return position.frames == 0 ? std::optional<RenderPosition>{} : position;
    }
  }
  return {};
}
}
