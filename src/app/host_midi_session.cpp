// SPDX-License-Identifier: GPL-3.0-only
#include "app/host_midi_session.hpp"

#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "midi/host_midi_port.hpp"
#include "midi/midi_serial_receiver.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <string_view>

namespace blofeld::reference {
namespace {
std::uint64_t positive_period(const std::uint64_t period) {
  if (period == 0) { throw Fault("Host MIDI session requires a positive poll period"); }
  return period;
}
}

HostMidiSession::HostMidiSession(const std::string_view name, const std::uint64_t poll_period_ticks)
    : period_(positive_period(poll_period_ticks)), port_(name) {}

void HostMidiSession::retire() noexcept {
  port_.retire();
  retired_ = true;
}

void HostMidiSession::validate(const MidiInputWindow &window) {
  if (!window.owner || !window.configured) {
    throw Fault("Host MIDI session requires an owned configured board input");
  }
  if ((previous_ && (window.owner != previous_->owner ||
                     window.restore_revision != previous_->restore_revision ||
                     window.ticks < previous_->ticks)) ||
      (previous_output_ && (window.owner != previous_output_->owner ||
                            window.restore_revision != previous_output_->restore_revision ||
                            window.ticks < previous_output_->ticks))) {
    throw Fault("Host MIDI board owner/restore/time changed; create a new session");
  }
  previous_ = window;
}

void HostMidiSession::validate_output(const MidiOutputWindow &window) {
  if (!window.owner || !window.configured) {
    throw Fault("Host MIDI output requires an owned configured board output");
  }
  if ((previous_output_ && (window.owner != previous_output_->owner ||
                            window.restore_revision != previous_output_->restore_revision ||
                            window.ticks < previous_output_->ticks)) ||
      (previous_ && (window.owner != previous_->owner ||
                     window.restore_revision != previous_->restore_revision ||
                     window.ticks < previous_->ticks))) {
    throw Fault("Host MIDI output owner/restore/time changed; create a new session");
  }
  previous_output_ = window;
}

std::size_t HostMidiSession::flush_output(Board &board) {
  if (retired_) { throw Fault("Retired host MIDI session requires a new port"); }
  std::uint64_t at = 0;
  try {
    const auto window = board.midi_output_window();
    at = window.ticks;
    validate_output(window);
    constexpr unsigned batch_capacity = 256;
    std::array<std::uint8_t, batch_capacity> bytes{};
    if (window.queued_bytes > bytes.size()) {
      throw Fault("Host MIDI output exceeds the bounded native queue");
    }
    for (std::size_t index = 0; index < window.queued_bytes; ++index) {
      const auto byte = board.pop_midi_output();
      if (!byte) { throw Fault("Host MIDI native output queue changed during delivery"); }
      bytes[index] = byte->value;
    }
    if (window.queued_bytes != 0) { port_.write(std::span{bytes}.first(window.queued_bytes)); }
    return window.queued_bytes;
  } catch (const Fault &error) {
    retire();
    throw Fault("Host MIDI output tick=" + std::to_string(at) + ": " + error.what());
  } catch (...) {
    retire();
    throw;
  }
}

std::size_t HostMidiSession::poll(Board &board) {
  if (retired_) { throw Fault("Retired host MIDI session requires a new port"); }
  std::uint64_t at = 0;
  try {
    const auto window = board.midi_window();
    at = window.ticks;
    validate(window); // Check restore/ownership even between transport polls.
    if (board.midi_output_window().configured) { static_cast<void>(flush_output(board)); }
    if (board.finished()) { throw Fault("Host MIDI input exceeds the board execution budget"); }
    if (!window.started || at < next_) { return 0; }
    if (at > std::numeric_limits<std::uint64_t>::max() - period_) {
      throw Fault("Host MIDI poll timestamp exhausted");
    }
    next_ = at + period_;
    constexpr unsigned batch_capacity = 256;
    std::array<std::uint8_t, batch_capacity> bytes{};
    const auto space = std::min(bytes.size(), window.available);
    // An empty span still checks kernel loss under downstream backpressure.
    const auto count = port_.read(std::span{bytes}.first(space));
    if (count != 0) { board.enqueue_midi(std::span{bytes}.first(count)); }
    return count;
  } catch (const Fault &error) {
    retire();
    throw Fault("Host MIDI session tick=" + std::to_string(at) + ": " + error.what());
  } catch (...) {
    retire();
    throw;
  }
}
}
