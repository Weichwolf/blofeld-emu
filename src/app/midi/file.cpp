// SPDX-License-Identifier: GPL-3.0-only
#include "midi/file.hpp"

#include "io/midi_port.hpp"
#include "midi/smf.hpp"
#include "midi/time.hpp"

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <optional>
#include <stdexcept>
#include <stop_token>

namespace blofeld::app {
MidiFile::MidiFile(const std::filesystem::path &path, const std::optional<std::uint16_t> pattern)
    : file_(path, pattern) {}

void MidiFile::validate(const host::midi::Smf::Event &event) {
  if (event.kind == host::midi::Smf::Kind::meta) { return; }
  constexpr std::uint8_t sysex_start = 0xf0;
  constexpr std::uint8_t sysex_end = 0xf7;
  constexpr std::uint8_t realtime = 0xf8;
  for (const auto byte : event.bytes) {
    if (byte >= realtime) { continue; }
    if (byte == sysex_start) {
      if (exclusive_) { throw std::runtime_error("SMF tracks collide inside destination SysEx"); }
      exclusive_ = event.track;
    } else if (exclusive_) {
      if (*exclusive_ != event.track || (byte >= 0x80 && byte != sysex_end)) {
        throw std::runtime_error("SMF tracks interrupt destination SysEx");
      }
      if (byte == sysex_end) { exclusive_.reset(); }
    }
  }
}

std::optional<std::uint64_t>
MidiFile::feed(vi::MidiPort &port, const std::uint64_t origin, const std::stop_token &stop) {
  if (phase_ != Phase::ready || !port.input_limit()) {
    throw std::logic_error("MIDI file requires a fresh source and streaming destination");
  }
  phase_ = Phase::failed;
  try {
    while (const auto event = file_.next()) {
      if (stop.stop_requested()) { return {}; }
      const auto relative = event->time.ceil_ticks(vi::MidiPort::reference_hz);
      if (relative > std::numeric_limits<std::uint64_t>::max() - origin) {
        throw std::overflow_error("SMF event exceeds the instrument timeline");
      }
      const auto ticks = origin + relative;
      validate(*event);
      if (event->kind == host::midi::Smf::Kind::meta) {
        const auto limit = port.input_limit();
        if (!limit || !port.advance_input(std::max(*limit, ticks))) {
          throw std::runtime_error("SMF metadata could not publish its input frontier");
        }
      } else if (!port.write_input(event->bytes, ticks, stop)) {
        if (stop.stop_requested()) { return {}; }
        throw std::runtime_error(port.error() != nullptr ? port.error() : "SMF input was rejected");
      }
    }
    if (stop.stop_requested()) { return {}; }
    if (exclusive_) { throw std::runtime_error("SMF ends inside destination SysEx"); }
    const auto end = port.input_limit();
    if (!end) { throw std::runtime_error("SMF input lost its completion frontier"); }
    phase_ = Phase::finished;
    return end;
  } catch (...) {
    port.abort_input();
    throw;
  }
}
}
