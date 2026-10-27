// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "midi/midi1_status.hpp"
#include "midi/sequence_codec.hpp"

#include <cstddef>
#include <cstdint>
#include <sound/asequencer.h>
#include <string>

namespace blofeld::detail {
namespace {
constexpr unsigned data_limit = 127, channels = 16, maximum14 = 16383;
constexpr int bend_bias = 8192;

std::uint8_t data(const int value) {
  if (value < 0 || value > static_cast<int>(data_limit)) {
    throw Fault("Host MIDI sequencer data exceeds seven bits");
  }
  return static_cast<std::uint8_t>(value);
}

std::uint8_t status(const unsigned base, const unsigned channel) {
  if (channel >= channels) { throw Fault("Host MIDI sequencer channel exceeds MIDI-1"); }
  return static_cast<std::uint8_t>(base | channel);
}

SequenceBytes
short_message(const unsigned first, const int second, const int third, const std::size_t size = 3) {
  return {.data = {static_cast<std::uint8_t>(first), data(second), data(third)}, .size = size};
}

SequenceBytes control(const snd_seq_event &event) {
  const auto &value = event.data.control;
  switch (event.type) {
    case SNDRV_SEQ_EVENT_CONTROLLER:
      if (value.param > data_limit) { throw Fault("Host MIDI controller exceeds seven bits"); }
      return short_message(
          status(midi1::control_change, value.channel), static_cast<int>(value.param), value.value);
    case SNDRV_SEQ_EVENT_PGMCHANGE:
      return short_message(status(midi1::program_change, value.channel), value.value, 0, 2);
    case SNDRV_SEQ_EVENT_CHANPRESS:
      return short_message(status(midi1::channel_pressure, value.channel), value.value, 0, 2);
    case SNDRV_SEQ_EVENT_PITCHBEND: {
      if (value.value < -bend_bias || value.value >= bend_bias) {
        throw Fault("Host MIDI pitch bend exceeds fourteen bits");
      }
      const auto pitch = static_cast<unsigned>(value.value + bend_bias);
      return short_message(status(midi1::pitch_bend, value.channel),
                           static_cast<int>(pitch & data_limit),
                           static_cast<int>(pitch >> 7U));
    }
    default: throw Fault("Unknown host MIDI control event");
  }
}

SequenceBytes expanded_control(const snd_seq_event &event) {
  const auto &value = event.data.control;
  const bool paired = event.type == SNDRV_SEQ_EVENT_CONTROL14;
  if (value.value < 0 || value.value > static_cast<int>(maximum14) ||
      value.param > (paired ? midi1::last_coarse_controller : maximum14)) {
    throw Fault("Host MIDI compound controller exceeds MIDI-1");
  }
  SequenceBytes result;
  const auto append = [&](const unsigned controller, const unsigned amount) {
    result.data.at(result.size++) = status(midi1::control_change, value.channel);
    result.data.at(result.size++) = static_cast<std::uint8_t>(controller);
    result.data.at(result.size++) = static_cast<std::uint8_t>(amount);
  };
  const auto amount = static_cast<unsigned>(value.value);
  if (paired) {
    append(value.param, amount >> 7U);
    append(value.param + 32U, amount & data_limit);
  } else {
    const auto first = event.type == SNDRV_SEQ_EVENT_REGPARAM ? 101U : 99U;
    append(first, value.param >> 7U);
    append(first - 1, value.param & data_limit);
    append(6, amount >> 7U);
    append(midi1::data_entry_fine, amount & data_limit);
  }
  return result;
}
}

SequenceBytes sequence_bytes(const snd_seq_event &event) {
  switch (event.type) {
    case SNDRV_SEQ_EVENT_NOTEON:
    case SNDRV_SEQ_EVENT_NOTEOFF:
    case SNDRV_SEQ_EVENT_KEYPRESS: {
      unsigned base = midi1::key_pressure;
      if (event.type == SNDRV_SEQ_EVENT_NOTEON) { base = midi1::note_on; }
      if (event.type == SNDRV_SEQ_EVENT_NOTEOFF) { base = midi1::note_off; }
      return short_message(
          status(base, event.data.note.channel), event.data.note.note, event.data.note.velocity);
    }
    case SNDRV_SEQ_EVENT_CONTROLLER:
    case SNDRV_SEQ_EVENT_PGMCHANGE:
    case SNDRV_SEQ_EVENT_CHANPRESS:
    case SNDRV_SEQ_EVENT_PITCHBEND: return control(event);
    case SNDRV_SEQ_EVENT_CONTROL14:
    case SNDRV_SEQ_EVENT_REGPARAM:
    case SNDRV_SEQ_EVENT_NONREGPARAM: return expanded_control(event);
    case SNDRV_SEQ_EVENT_QFRAME:
      return short_message(midi1::quarter_frame, event.data.control.value, 0, 2);
    case SNDRV_SEQ_EVENT_SONGSEL:
      return short_message(midi1::song_select, event.data.control.value, 0, 2);
    case SNDRV_SEQ_EVENT_SONGPOS: {
      const auto value = event.data.control.value;
      if (value < 0 || value > static_cast<int>(maximum14)) {
        throw Fault("Host MIDI song position exceeds fourteen bits");
      }
      const auto amount = static_cast<unsigned>(value);
      return short_message(midi1::song_position,
                           static_cast<int>(amount & data_limit),
                           static_cast<int>(amount >> 7U));
    }
    case SNDRV_SEQ_EVENT_TUNE_REQUEST: return {.data = {midi1::tune_request}, .size = 1};
    case SNDRV_SEQ_EVENT_CLOCK: return {.data = {midi1::clock}, .size = 1};
    case SNDRV_SEQ_EVENT_START: return {.data = {midi1::start}, .size = 1};
    case SNDRV_SEQ_EVENT_CONTINUE: return {.data = {midi1::resume}, .size = 1};
    case SNDRV_SEQ_EVENT_STOP: return {.data = {midi1::stop}, .size = 1};
    case SNDRV_SEQ_EVENT_SENSING: return {.data = {midi1::active_sensing}, .size = 1};
    case SNDRV_SEQ_EVENT_RESET: return {.data = {midi1::reset}, .size = 1};
    default:
      throw Fault("Unsupported host MIDI sequencer event type=" + std::to_string(event.type));
  }
}
}
