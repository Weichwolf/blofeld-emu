// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "io/midi_port.hpp"
#include "midi/smf.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <stop_token>

namespace blofeld::app {
// One pass maps all selected SMF tracks to one explicit DIN destination.
class MidiFile {
public:
  explicit MidiFile(const std::filesystem::path &path, std::optional<std::uint16_t> pattern = {});
  // Keep the EOF barrier until the caller has published its render completion limit.
  [[nodiscard]] std::optional<std::uint64_t>
  feed(vi::MidiPort &port, std::uint64_t origin, const std::stop_token &stop);

private:
  enum class Phase : std::uint8_t { ready, finished, failed };
  void validate(const host::midi::Smf::Event &event);
  host::midi::Smf file_;
  std::optional<std::uint16_t> exclusive_;
  Phase phase_ = Phase::ready;
};
}
