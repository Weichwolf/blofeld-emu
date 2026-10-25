// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "midi/host_midi.hpp"

#include <cstdint>
#include <filesystem>
#include <memory>

namespace blofeld::reference {
class Board;

// Non-real-time host runner. No file/device handle enters a board checkpoint.
class HostMidiPump {
public:
  HostMidiPump(std::filesystem::path path, MidiInputKind kind, std::uint64_t timebase_hz);
  void operator()(Board &board);

private:
  std::filesystem::path path_;
  MidiInputKind kind_;
  std::uint64_t period_ = 0;
  std::uint64_t next_ = 0;
  std::unique_ptr<HostMidiInput> input_;
};
}
