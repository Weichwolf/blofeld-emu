// SPDX-License-Identifier: GPL-3.0-only
#include "app/host_midi_runner.hpp"

#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "midi/host_midi.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <span>
#include <utility>

namespace blofeld::reference {
namespace {
constexpr unsigned polls_per_reference_second = 1000;
constexpr unsigned batch_capacity = 256;
}

HostMidiPump::HostMidiPump(std::filesystem::path path,
                           const MidiInputKind kind,
                           const std::uint64_t timebase_hz)
    : path_(std::move(path)),
      kind_(kind),
      period_(std::max(std::uint64_t{1}, timebase_hz / polls_per_reference_second)) {
  if (timebase_hz == 0) { throw Fault("Host MIDI polling requires a positive reference timebase"); }
}

void HostMidiPump::operator()(Board &board) {
  const auto window = board.midi_window();
  if (!window.started || window.ticks < next_) { return; }
  if (!input_) { input_ = std::make_unique<HostMidiInput>(path_, kind_); }
  if (window.ticks > std::numeric_limits<std::uint64_t>::max() - period_) {
    throw Fault("Host MIDI poll timestamp exhausted");
  }
  next_ = window.ticks + period_;
  std::array<std::uint8_t, batch_capacity> bytes{};
  const auto space = std::min(bytes.size(), window.available);
  const auto received = input_->read(std::span{bytes}.first(space));
  if (received != 0) { board.enqueue_midi(std::span{bytes}.first(received)); }
}
}
