// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "midi/sequence_codec.hpp"
#include "midi/sequence_pool.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <sound/asequencer.h>
#include <span>
#include <string>
#include <vector>

namespace {
using blofeld::detail::sequence_bytes;
constexpr std::uint8_t input_note = 63, maximum_data = 127, first_note = 60, second_note = 61;
constexpr std::uint8_t first_velocity = 100, sysex_begin = 0xf0, sysex_end = 0xf7,
                       timing_clock = 0xf8;
constexpr std::uint8_t note_on_status = 0x90, note_off_status = 0x80, pressure_status = 0xa0;
constexpr std::uint8_t undefined_status = 0xf4;
constexpr unsigned parameter_number = 0x123, parameter_amount = 0x456;
constexpr unsigned sysex_data_count = 1025, clock_after_data = 333;

using blofeld::detail::SequenceBytes;
using blofeld::detail::SequenceEncoder;

void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const blofeld::Fault &error) { return error.what(); }
  throw blofeld::Fault("Expected a fixed MIDI protocol refusal");
}

void equals(const SequenceBytes bytes, const std::span<const std::uint8_t> expected) {
  require(bytes.size == expected.size(), "Independent wire message length matches");
  for (std::size_t n = 0; n < bytes.size; ++n) {
    require(bytes.data[n] == expected[n], "Independent MIDI status/data expectation matches");
  }
}

void test_input() {
  for (unsigned channel = 0; channel < 16; ++channel) {
    constexpr std::array<unsigned, 3> types{
        SNDRV_SEQ_EVENT_NOTEON, SNDRV_SEQ_EVENT_NOTEOFF, SNDRV_SEQ_EVENT_KEYPRESS};
    for (const auto type : types) {

      snd_seq_event event{};
      event.type = static_cast<std::uint8_t>(type);
      event.data.note.channel = static_cast<std::uint8_t>(channel);
      event.data.note.note = input_note;
      event.data.note.velocity = maximum_data;
      unsigned family = pressure_status;
      if (type == SNDRV_SEQ_EVENT_NOTEON) { family = note_on_status; }
      if (type == SNDRV_SEQ_EVENT_NOTEOFF) { family = note_off_status; }
      const std::array<std::uint8_t, 3> expected{
          static_cast<std::uint8_t>(family | channel), 63, 127};
      equals(sequence_bytes(event), expected);
    }
  }
  for (const int value : {-8192, -1, 0, 1, 8191}) {
    snd_seq_event event{};
    event.type = SNDRV_SEQ_EVENT_PITCHBEND;
    event.data.control.channel = 9;
    event.data.control.value = value;
    const auto centered = static_cast<unsigned>(value + 8192);
    const std::array<std::uint8_t, 3> expected{
        0xe9, static_cast<std::uint8_t>(centered % 128), static_cast<std::uint8_t>(centered / 128)};
    equals(sequence_bytes(event), expected);
  }
  snd_seq_event event{};
  event.type = SNDRV_SEQ_EVENT_REGPARAM;
  event.data.control.channel = 2;
  event.data.control.param = parameter_number;
  event.data.control.value = parameter_amount;
  const std::array<std::uint8_t, 12> rpn{0xb2, 101, 2, 0xb2, 100, 35, 0xb2, 6, 8, 0xb2, 38, 86};
  equals(sequence_bytes(event), rpn);
  event.type = SNDRV_SEQ_EVENT_NONREGPARAM;
  const std::array<std::uint8_t, 12> nrpn{0xb2, 99, 2, 0xb2, 98, 35, 0xb2, 6, 8, 0xb2, 38, 86};
  equals(sequence_bytes(event), nrpn);
  event.type = SNDRV_SEQ_EVENT_CONTROL14;
  event.data.control.param = 7;
  const std::array<std::uint8_t, 6> control{0xb2, 7, 8, 0xb2, 39, 86};
  equals(sequence_bytes(event), control);
}

void test_output() {
  SequenceEncoder encoder;
  const std::array<std::uint8_t, 6> source{0x93, 60, 100, 61, 0xfe, 7};
  unsigned packets = 0;
  for (const auto byte : source) {
    require(encoder.feed(byte), "Fixed message and realtime bytes are consumed once");
    if (const auto *packet = encoder.pending()) {
      if (packets == 1) {
        require(packet->event.type == SNDRV_SEQ_EVENT_SENSING,
                "Realtime interleaves the incomplete running-status note");
      } else {
        require(packet->event.type == SNDRV_SEQ_EVENT_NOTEON &&
                    packet->event.data.note.channel == 3 &&
                    packet->event.data.note.note == (packets == 0 ? first_note : second_note) &&
                    packet->event.data.note.velocity == (packets == 0 ? first_velocity : 7),
                "Independent NOTEON fields and running status survive realtime");
      }
      ++packets;
      encoder.consume();
    }
  }
  require(packets == 3, "Exactly two notes and one realtime event are emitted");
  const std::array<std::uint8_t, 3> bend{0xe7, 0, 64};
  for (const auto byte : bend) { require(encoder.feed(byte), "Pitch input consumes exactly once"); }
  require(encoder.pending() != nullptr &&
              encoder.pending()->event.type == SNDRV_SEQ_EVENT_PITCHBEND &&
              encoder.pending()->event.data.control.channel == 7 &&
              encoder.pending()->event.data.control.value == 0,
          "Literal E7 00 40 has independent centered pitch value");
  encoder.consume();
}

void test_sysex() {
  SequenceEncoder encoder;
  std::vector<std::uint8_t> wire{sysex_begin};
  for (unsigned n = 0; n < sysex_data_count; ++n) {
    wire.push_back(static_cast<std::uint8_t>(n % 128));
    if (n == clock_after_data) { wire.push_back(timing_clock); }
  }
  wire.push_back(sysex_end);
  std::vector<std::uint8_t> observed;
  std::size_t cursor = 0;
  unsigned fragments = 0;
  unsigned clocks = 0;
  constexpr std::array<std::size_t, 5> lengths{256, 79, 256, 256, 180};
  while (cursor < wire.size()) {
    const bool consumed = encoder.feed(wire[cursor]);
    if (const auto *packet = encoder.pending()) {
      if (packet->event.type == SNDRV_SEQ_EVENT_SYSEX) {
        require(packet->size <= 256 && packet->size != 0,
                "SysEx output fragments retain the fixed payload bound");
        for (std::size_t n = 0; n < packet->size; ++n) { observed.push_back(packet->data[n]); }
        require(packet->size == lengths.at(fragments),
                "Literal fragment boundaries retain realtime order");
        ++fragments;
      } else {
        require(packet->event.type == SNDRV_SEQ_EVENT_CLOCK,
                "Only the original realtime clock appears");
        observed.push_back(timing_clock);
        ++clocks;
      }
      encoder.consume();
    }
    if (consumed) { ++cursor; }
  }
  require(observed == wire && clocks == 1 && fragments == 5,
          "All SysEx bytes and realtime retain original order across bounded fragments");
  require(rejects([&] { static_cast<void>(encoder.feed(1)); }).contains("running status"),
          "EOX cancels running status instead of retaining the earlier F0 prefix");
}

enum class ChannelKind : std::uint8_t { note, control, one_data, bend };

struct ChannelCase {
  std::uint8_t status, type;
  ChannelKind kind;
};

void check_channel_output(const ChannelCase item,
                          const unsigned channel,
                          const std::uint8_t first,
                          const std::uint8_t second) {
  SequenceEncoder encoder;
  require(encoder.feed(static_cast<std::uint8_t>(item.status | channel)), "Status accepted");
  require(encoder.feed(first), "First data accepted");
  if (item.kind != ChannelKind::one_data) { require(encoder.feed(second), "Second data accepted"); }
  const auto *packet = encoder.pending();
  require(packet != nullptr, "The channel message completes");
  if (packet == nullptr) { throw blofeld::Fault("Missing actual channel packet"); }
  require(packet->event.type == item.type && packet->size == 0,
          "Independent channel opcode expectation selects one fixed kernel event");
  if (item.kind == ChannelKind::note) {
    const auto &note = packet->event.data.note;
    require(note.channel == channel && note.note == first && note.velocity == second,
            "Every channel note/pressure status has independently expected fields");
    return;
  }
  int value = second;
  if (item.kind == ChannelKind::one_data) { value = first; }
  if (item.kind == ChannelKind::bend) {
    value = static_cast<int>(first) + (128 * static_cast<int>(second)) - 8192;
  }
  const auto parameter = item.kind == ChannelKind::control ? static_cast<unsigned>(first) : 0U;
  const auto &control = packet->event.data.control;
  require(control.channel == channel && control.value == value && control.param == parameter,
          "Every control/program/pressure/bend field has an independent expectation");
}

void test_channel_output() {
  constexpr std::array<ChannelCase, 7> cases{
      {{.status = 0x80, .type = SNDRV_SEQ_EVENT_NOTEOFF, .kind = ChannelKind::note},
       {.status = 0x90, .type = SNDRV_SEQ_EVENT_NOTEON, .kind = ChannelKind::note},
       {.status = 0xa0, .type = SNDRV_SEQ_EVENT_KEYPRESS, .kind = ChannelKind::note},
       {.status = 0xb0, .type = SNDRV_SEQ_EVENT_CONTROLLER, .kind = ChannelKind::control},
       {.status = 0xc0, .type = SNDRV_SEQ_EVENT_PGMCHANGE, .kind = ChannelKind::one_data},
       {.status = 0xd0, .type = SNDRV_SEQ_EVENT_CHANPRESS, .kind = ChannelKind::one_data},
       {.status = 0xe0, .type = SNDRV_SEQ_EVENT_PITCHBEND, .kind = ChannelKind::bend}}};
  constexpr std::array<std::uint8_t, 4> values{0, 1, 63, 127};
  for (const auto item : cases) {
    for (unsigned channel = 0; channel < 16; ++channel) {
      for (const auto first : values) {
        for (const auto second : values) { check_channel_output(item, channel, first, second); }
      }
    }
  }
}

void test_system() {
  struct System {
    std::uint8_t status, type;
    unsigned length;
    int value;
  };

  constexpr std::array<System, 10> cases{
      {{.status = 0xf1, .type = SNDRV_SEQ_EVENT_QFRAME, .length = 2, .value = 35},
       {.status = 0xf2, .type = SNDRV_SEQ_EVENT_SONGPOS, .length = 3, .value = 547},
       {.status = 0xf3, .type = SNDRV_SEQ_EVENT_SONGSEL, .length = 2, .value = 35},
       {.status = 0xf6, .type = SNDRV_SEQ_EVENT_TUNE_REQUEST, .length = 1, .value = 0},
       {.status = 0xf8, .type = SNDRV_SEQ_EVENT_CLOCK, .length = 1, .value = 0},
       {.status = 0xfa, .type = SNDRV_SEQ_EVENT_START, .length = 1, .value = 0},
       {.status = 0xfb, .type = SNDRV_SEQ_EVENT_CONTINUE, .length = 1, .value = 0},
       {.status = 0xfc, .type = SNDRV_SEQ_EVENT_STOP, .length = 1, .value = 0},
       {.status = 0xfe, .type = SNDRV_SEQ_EVENT_SENSING, .length = 1, .value = 0},
       {.status = 0xff, .type = SNDRV_SEQ_EVENT_RESET, .length = 1, .value = 0}}};
  for (const auto item : cases) {
    const std::array<std::uint8_t, 3> wire{item.status, 35, 4};
    SequenceEncoder encoder;
    for (unsigned n = 0; n < item.length; ++n) {
      require(encoder.feed(wire[n]), "System byte accepted");
    }
    const auto *packet = encoder.pending();
    require(packet != nullptr && packet->event.type == item.type,
            "Literal system status selects expected type");
    if (item.length > 1) {
      require(packet->event.data.control.value == item.value &&
                  packet->event.data.control.channel == 0,
              "System common data retains independent seven/fourteen-bit value");
    }
    snd_seq_event source{};
    source.type = item.type;
    source.data.control.value = item.value;
    equals(sequence_bytes(source), std::span{wire}.first(item.length));
  }
}

void test_pool_negotiation() {
  using blofeld::detail::negotiate_sequence_pool;
  constexpr int preference = 4096;
  constexpr int normal_current = 200;
  constexpr int maximum_requests = 14;
  constexpr std::array<int, 9> ceilings{2, 3, 7, 31, 199, 200, 2000, 4096, 8192};
  for (const auto ceiling : ceilings) {
    auto actual = std::min(normal_current, ceiling);
    int requests = 0;
    const auto ignored = [&](const int request) {
      ++requests;
      if (request <= ceiling) { actual = request; }
      return actual;
    };
    const auto result = negotiate_sequence_pool(preference, actual, ignored);
    require(result == std::min(preference, ceiling) && result == actual &&
                requests <= maximum_requests,
            "Ignored oversized requests converge to independent host ceilings within a bound");
    actual = std::min(normal_current, ceiling);
    const auto clamped = [&](const int request) {
      actual = std::min(request, ceiling);
      return actual;
    };
    require(negotiate_sequence_pool(preference, actual, clamped) == std::min(preference, ceiling),
            "Clamped grants retain actual capacity instead of the preference");
  }
  const auto fixed = [](int) { return normal_current; };
  require(negotiate_sequence_pool(preference, normal_current, fixed) == normal_current,
          "A fixed host grant remains usable without an invented maximum");
  require(!rejects([&] { static_cast<void>(negotiate_sequence_pool(1, 1, fixed)); }).empty() &&
              !rejects([&] { static_cast<void>(negotiate_sequence_pool(2, 0, fixed)); }).empty() &&
              !rejects([&] { static_cast<void>(negotiate_sequence_pool(2, 1, {})); }).empty(),
          "Invalid preferences, current dimensions and callbacks refuse before probing");
  for (const int grant : {0, 1}) {
    require(!rejects([&] {
               static_cast<void>(negotiate_sequence_pool(2, 1, [grant](int) { return grant; }));
             }).empty(),
            "Invalid or too-small actual grants fail visibly");
  }
  require(rejects([&] {
            static_cast<void>(negotiate_sequence_pool(2, 1, [](int) -> int {
              throw blofeld::Fault("Directed backend request failure");
            }));
          }).contains("backend request failure"),
          "Backend failures propagate instead of being treated as smaller grants");
}

void test_fragment_limits() {
  constexpr std::array<std::size_t, 5> limits{1, 2, 7, 31, 256};
  constexpr std::array<std::uint8_t, 13> wire{0xf0, 0x7d, 1, 2, 3, 0xf8, 4, 5, 6, 7, 8, 9, 0xf7};
  for (const auto limit : limits) {
    SequenceEncoder encoder(limit);
    std::vector<std::uint8_t> observed;
    std::size_t position = 0;
    while (position < wire.size()) {
      const auto consumed = encoder.feed(wire[position]);
      if (const auto *packet = encoder.pending()) {
        if (packet->event.type == SNDRV_SEQ_EVENT_SYSEX) {
          require(packet->size > 0 && packet->size <= limit,
                  "Each SysEx packet fits the negotiated host payload bound");
          const auto payload = std::span{packet->data}.first(packet->size);
          observed.insert(observed.end(), payload.begin(), payload.end());
        } else {
          require(packet->event.type == SNDRV_SEQ_EVENT_CLOCK,
                  "The literal realtime byte remains between its original adjacent bytes");
          observed.push_back(timing_clock);
        }
        encoder.consume();
      }
      if (consumed) { ++position; }
    }
    require(std::ranges::equal(observed, wire),
            "Even one-byte host fragments retain every SysEx byte and realtime position");
  }
  constexpr std::size_t excessive_limit = 257;
  for (const auto limit : {std::size_t{0}, excessive_limit}) {
    require(!rejects([&] { const SequenceEncoder invalid(limit); }).empty(),
            "Zero and out-of-container fragment limits refuse before input");
  }
}

void test_failures() {
  snd_seq_event event{};
  event.type = SNDRV_SEQ_EVENT_NOTEON;
  event.data.note.channel = 16;
  require(!rejects([&] { static_cast<void>(sequence_bytes(event)); }).empty(),
          "Channel 16 refuses");
  event.data.note.channel = 0;
  event.data.note.velocity = 128;
  require(!rejects([&] { static_cast<void>(sequence_bytes(event)); }).empty(),
          "Non-seven-bit data refuses");
  event.type = SNDRV_SEQ_EVENT_NOTE;
  require(!rejects([&] { static_cast<void>(sequence_bytes(event)); }).empty(),
          "Duration is not a wire message");
  SequenceEncoder encoder;
  require(!rejects([&] { static_cast<void>(encoder.feed(undefined_status)); }).empty(),
          "Undefined status refuses");
  SequenceEncoder other;
  require(other.feed(sysex_begin), "Start SysEx accepted");
  require(!rejects([&] { static_cast<void>(other.feed(note_on_status)); }).empty(),
          "Interrupted SysEx refuses visibly");
}
}

int main() {
  try {
    test_input();
    test_output();
    test_channel_output();
    test_system();
    test_sysex();
    test_pool_negotiation();
    test_fragment_limits();
    test_failures();
    std::cout
        << "Independent MIDI-1 event, running-status, realtime and SysEx codec checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
