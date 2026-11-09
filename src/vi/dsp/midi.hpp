// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "io/midi_port.hpp"

#include <cstdint>
#include <optional>

namespace blofeld::vi::dsp {
// Functional 8N1 input for the firmware's Timer0 receiver. No voltage model or
// per-clock work: expose a level and the next bit boundary in common time units.
class MidiInput {
public:
  explicit MidiInput(MidiPort *port = nullptr) : port_(port) {}

  void reset(std::uint64_t now) noexcept;
  [[nodiscard]] bool advance(std::uint64_t now) noexcept;

  [[nodiscard]] bool connected() const noexcept { return port_ != nullptr; }

  [[nodiscard]] bool level() const noexcept { return level_; }

  [[nodiscard]] std::optional<std::uint64_t> deadline() const noexcept;
  [[nodiscard]] const char *error() const noexcept;

private:
  void sample() noexcept;
  MidiPort *port_;
  std::uint64_t time_ = 0;
  bool level_ = true;
  const char *error_ = nullptr;
};
}
