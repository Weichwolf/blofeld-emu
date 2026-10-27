// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "board/reference_board.hpp"
#include "midi/host_midi_port.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

namespace blofeld::reference {
// Ordinary owner-thread input admission. No host handles, pending bytes or
// session identity enter modeled snapshots. There is no output/USB emulation.
class HostMidiSession {
public:
  HostMidiSession(std::string_view name, std::uint64_t poll_period_ticks);
  [[nodiscard]] std::size_t poll(Board &board);
  void retire() noexcept;

  [[nodiscard]] bool retired() const { return retired_; }

  [[nodiscard]] std::optional<HostMidiAddress> address() const { return port_.address(); }

  [[nodiscard]] std::optional<HostMidiLimits> limits() const { return port_.limits(); }

private:
  void validate(const MidiInputWindow &window);
  std::uint64_t period_;
  HostMidiPort port_;
  std::optional<MidiInputWindow> previous_;
  std::uint64_t next_ = 0;
  bool retired_ = false;
};
}
