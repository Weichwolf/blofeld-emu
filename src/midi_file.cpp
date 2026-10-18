// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "input.hpp"

#include <algorithm>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>

namespace blofeld::input {
namespace {
constexpr std::uint32_t header_signature = 0x4d546864;
constexpr std::uint32_t track_signature = 0x4d54726b;
constexpr unsigned header_length = 6;
constexpr unsigned maximum_tracks = 1024;
constexpr unsigned program_change = 0xc0;
constexpr unsigned channel_pressure = 0xd0;

struct Track {
  std::uint8_t running = 0;
  Bytes pending;
  bool sysex_open = false;
  bool ended = false;
};

void meta_event(Reader &reader, Track &track) {
  const auto kind = reader.byte();
  const auto payload = reader.take(reader.vlq());
  if (kind == midi_end_track) {
    if (!payload.empty() || !reader.empty()) { throw Fault("Invalid MIDI end of track"); }
    track.ended = true;
  }
}

void sysex_event(Reader &reader,
                 Track &track,
                 const std::uint8_t status,
                 std::vector<Bytes> &messages) {
  track.running = 0;
  if ((status == midi_sysex_start && track.sysex_open) ||
      (status == midi_sysex_end && !track.sysex_open)) {
    throw Fault("Invalid split SysEx sequence");
  }
  track.sysex_open = true;
  const auto payload = reader.take(reader.vlq());
  for (const auto byte : payload) {
    if (byte >= 0x80 && byte != midi_sysex_end) { throw Fault("Non-seven-bit SysEx data"); }
  }
  track.pending.insert(track.pending.end(), payload.begin(), payload.end());
  if (!track.pending.empty() && track.pending.back() == midi_sysex_end) {
    const auto body = std::span(track.pending).first(track.pending.size() - 1);
    if (std::ranges::any_of(body, [](const auto value) { return value >= 0x80; })) {
      throw Fault("Non-seven-bit firmware SysEx data");
    }
    messages.push_back(std::move(track.pending));
    track.pending.clear();
    track.sysex_open = false;
  }
}

void track_event(Reader &reader, Track &track, std::vector<Bytes> &messages) {
  reader.vlq();
  auto status = reader.byte();
  const bool first_data = status < 0x80;
  if (first_data) {
    if (track.running == 0) { throw Fault("MIDI running status without prior channel event"); }
    status = track.running;
  }
  if (status == midi_meta) {
    meta_event(reader, track);
  } else if (status == midi_sysex_start || status == midi_sysex_end) {
    sysex_event(reader, track, status, messages);
  } else if (status >= 0x80 && status < midi_sysex_start) {
    if (track.sysex_open) { throw Fault("Channel event inside unfinished SysEx"); }
    track.running = status;
    const auto kind = status & midi_status_mask;
    const unsigned size = (kind == program_change || kind == channel_pressure) ? 1 : 2;
    const auto data = reader.take(size - static_cast<unsigned>(first_data));
    if (std::ranges::any_of(data, [](const auto value) { return value >= 0x80; })) {
      throw Fault("Invalid MIDI channel data");
    }
  } else {
    throw Fault("Unsupported MIDI system event");
  }
}
}

std::vector<Bytes> sysex_messages(const Bytes &input) {
  Reader file(input);
  if (file.big(4) != header_signature || file.big(4) != header_length) {
    throw Fault("Expected a standard MIDI header");
  }
  const auto format = file.big(2);
  const auto tracks = file.big(2);
  file.big(2);
  if (format > 1 || tracks == 0 || tracks > maximum_tracks || (format == 0 && tracks != 1)) {
    throw Fault("Unsupported MIDI track layout");
  }
  std::vector<Bytes> messages;
  for (std::uint32_t index = 0; index < tracks; ++index) {
    if (file.big(4) != track_signature) { throw Fault("Missing MIDI track"); }
    Reader reader(file.take(file.big(4)));
    Track track;
    while (!reader.empty()) { track_event(reader, track, messages); }
    if (track.sysex_open || !track.ended) { throw Fault("Incomplete MIDI track"); }
  }
  if (!file.empty()) { throw Fault("Trailing MIDI data"); }
  return messages;
}
}
