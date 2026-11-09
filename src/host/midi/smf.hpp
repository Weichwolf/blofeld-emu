// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "midi/time.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>

namespace blofeld::host::midi {
class Smf {
public:
  enum class Kind : std::uint8_t { channel, sysex, escape, meta };

  struct Event {
    Time time;
    std::uint16_t track = 0;
    Kind kind = Kind::channel;
    std::uint8_t meta = 0;
    bool first = true, last = true;
    std::span<const std::uint8_t> bytes;
  };

  static constexpr std::size_t packet_bytes = 256;
  explicit Smf(const std::filesystem::path &path, std::optional<std::uint16_t> pattern = {});
  ~Smf();
  Smf(const Smf &) = delete;
  Smf &operator=(const Smf &) = delete;
  // Bytes are borrowed until next(). Only meta payloads are non-transmittable.
  [[nodiscard]] std::optional<Event> next();
  [[nodiscard]] Time time() const noexcept;

private:
  struct State;
  std::unique_ptr<State> state_;
};
}
