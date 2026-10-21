// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/serial_pcm.hpp"
#include "serial_pcm_fixture.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <string>

namespace blofeld::test::pcm {
namespace {
void literal_words_and_edges() {
  for (const auto edge : {SerialPcmEdge::rising, SerialPcmEdge::falling}) {
    for (const bool polarity : {false, true}) {
      Wire wire({.slot_bits = 8, .word_bits = 4, .sample_edge = edge, .frame_sync_high = polarity});
      wire.baseline();
      wire.feed("1010????0110????", "1111111100000000");
      require(wire.frames.size() == 1 && wire.receiver.completed_frames() == 1,
              "Exactly one complete two-slot frame is delivered");
      const auto &frame = wire.frames.front();
      require(frame.channels == 2 && frame.samples[0] == -6 && frame.samples[1] == 6 &&
                  frame.first_tick == 2 && frame.last_tick == 32,
              "Literal MSB-first signed words ignore padding and retain actual sample times");
      wire.feed("0111111110000000", "1111111100000000");
      constexpr unsigned second_frame_tick = 34;
      require(wire.frames.size() == 2 && wire.frames.back().samples[0] == 7 &&
                  wire.frames.back().samples[1] == -8 &&
                  wire.frames.back().first_tick == second_frame_tick,
              "Consecutive frames preserve slot order and full word-length FS");
    }
  }
}

void signed_widths_and_channel_bound() {
  constexpr unsigned maximum = 32;
  for (unsigned width = 1; width <= maximum; ++width) {
    Wire wire({.slot_bits = width, .word_bits = width});
    wire.baseline();
    wire.feed("1" + std::string(width - 1, '0') + "0" + std::string(width - 1, '1'),
              std::string(width, '1') + std::string(width, '0'));
    const auto sign = std::int64_t{1} << (width - 1);
    require(wire.frames.size() == 1 && wire.frames[0].samples[0] == -sign &&
                wire.frames[0].samples[1] == sign - 1,
            "Every signed width includes its independent minimum and maximum");
  }
  Wire wire({.slot_bits = 1, .word_bits = 1, .channels = maximum});
  wire.baseline();
  wire.feed("10101010101010101010101010101010", "10000000000000000000000000000000");
  require(wire.frames.size() == 1 && wire.frames[0].channels == maximum,
          "The bounded receiver supports exactly 32 serial slots");
  for (unsigned slot = 0; slot < maximum; ++slot) {
    require(wire.frames[0].samples.at(slot) == (slot % 2 == 0 ? -1 : 0),
            "Each one-bit signed channel retains literal slot order");
  }
}

void midframe_join_and_incomplete_prefix() {
  Wire wire({.slot_bits = 4, .word_bits = 4});
  wire.baseline(true);
  wire.feed("??", "11");
  require(!wire.receiver.locked() && wire.frames.empty(),
          "An initial active FS never fabricates a first-bit boundary");
  wire.feed("????", "0000");
  require(!wire.receiver.locked(), "Inactive FS only arms future frame detection");
  wire.feed("1010011", "1111000");
  require(wire.frames.empty() && wire.receiver.completed_frames() == 0,
          "A truncated frame is not filled or delivered");
  wire.feed("0", "0");
  require(wire.frames.size() == 1 && wire.frames[0].samples[0] == -6 &&
              wire.frames[0].samples[1] == 6 && wire.frames[0].first_tick == 14,
          "After a mid-word join the next complete frame retains literal payload");
}

void snapshots_and_failure_recovery() {
  const std::string bits = "1010????0110????0111????1000????";
  const std::string sync = "11111111000000001111111100000000";
  for (const unsigned prefix : {0U, 3U, 6U, 10U, 16U, 31U, 32U}) {
    Wire wire({.slot_bits = 8, .word_bits = 4});
    wire.baseline();
    wire.feed(bits.substr(0, prefix), sync.substr(0, prefix));
    const auto saved = wire;
    wire.feed(bits.substr(prefix), sync.substr(prefix));
    const auto expected = wire.frames;
    for (unsigned replay = 0; replay < 3; ++replay) {
      wire = saved;
      wire.feed(bits.substr(prefix), sync.substr(prefix));
      require(wire.frames == expected && wire.receiver.completed_frames() == 2,
              "Partial word, padding, slot and frame boundary copies replay exactly");
    }
  }
  Wire wire({.slot_bits = 4, .word_bits = 4});
  wire.baseline();
  wire.feed("10", "11");
  const auto saved = wire;
  const auto reason = rejects([&] { wire.feed("?", "1"); });
  require(wire.receiver.faulted() && reason.find("slot=0 bit=2") != std::string::npos &&
              reason.find("Unknown or undriven payload bit") != std::string::npos,
          "Unknown payload retains precise sampled-bit context and quarantines the receiver");
  require(rejects([&] { wire.feed("1", "1"); }).find("quarantined") != std::string::npos,
          "A failed receiver cannot silently resume");
  const auto failed = wire.receiver;
  auto copy = failed;
  require(copy.faulted(), "Copies retain failure quarantine");
  wire = saved;
  wire.feed("100110", "110000");
  require(wire.frames.size() == 1 && wire.frames[0].samples[0] == -6,
          "A trusted pre-fault copy restores the real partial word");
  wire.receiver.reset();
  wire.frames.clear();
  wire.ticks = 0;
  wire.baseline();
  wire.feed("01111000", "11110000");
  require(wire.frames.size() == 1 && wire.frames[0].samples[0] == 7 &&
              wire.receiver.completed_frames() == 1,
          "Explicit reset discards the old stream and requires a new baseline");
}

void strict_signal_and_frame_failures() {
  for (const auto &sync : {"11100000", "11111000", "11110001", "1111000000001111"}) {
    Wire wire({.slot_bits = 4, .word_bits = 4});
    wire.baseline();
    const auto reason =
        rejects([&] { wire.feed(std::string(std::string(sync).size(), '0'), sync); });
    require(reason.find("Unexpected word-length frame sync") != std::string::npos &&
                wire.receiver.faulted(),
            "Early, late and missing frame boundaries reject instead of regrouping samples");
  }
  for (unsigned missing = 0; missing < 2; ++missing) {
    SerialPcmReceiver receiver({});
    const auto reason = rejects([&] {
      (void)receiver.observe(0,
                             {.clock = missing == 0 ? std::nullopt : std::optional{false},
                              .frame_sync = missing == 1 ? std::nullopt : std::optional{false}});
    });
    require(receiver.faulted() && reason.find("clock/frame sync") != std::string::npos,
            "Unknown routing is never interpreted as a low clock or inactive frame");
  }
  Wire wire({.slot_bits = 4, .word_bits = 4});
  wire.baseline();
  wire.feed("00001111", "11110000");
  require(wire.frames[0].samples[0] == 0 && wire.frames[0].samples[1] == -1,
          "Known zero and negative-one data remain real samples");
  const auto before = wire.receiver.completed_frames();
  (void)rejects([&] { wire.feed("0", "0"); });
  require(wire.receiver.completed_frames() == before,
          "A missing next-frame boundary cannot emit an extra frame");
}

void formats_times_and_nonedges() {
  for (const auto format : {SerialPcmFormat{.slot_bits = 0},
                            SerialPcmFormat{.slot_bits = 33},
                            SerialPcmFormat{.word_bits = 0},
                            SerialPcmFormat{.slot_bits = 8, .word_bits = 9},
                            SerialPcmFormat{.channels = 1},
                            SerialPcmFormat{.channels = 33},
                            SerialPcmFormat{.sample_edge = SerialPcmEdge::unqualified}}) {
    require(rejects([&] { const SerialPcmReceiver receiver(format); }) ==
                "Invalid serial PCM format",
            "Unsupported formats reject before any observation");
  }
  for (const std::uint64_t last : {0U, 9U, 10U}) {
    SerialPcmReceiver receiver({});
    (void)receiver.observe(10, {.clock = false, .frame_sync = false});
    require(rejects([&] {
              (void)receiver.observe(last, {.clock = true, .frame_sync = true});
            }).find("Non-increasing") != std::string::npos,
            "Equal or backwards observations do not invent clock edges");
  }
  SerialPcmReceiver receiver({});
  (void)receiver.observe(std::numeric_limits<std::uint64_t>::max(),
                         {.clock = false, .frame_sync = false});
  require(receiver.completed_frames() == 0 && !receiver.faulted(),
          "The last representable baseline is valid and creates no synthetic successor");
  SerialPcmReceiver stationary({});
  for (std::uint64_t tick = 0; tick < 10; ++tick) {
    require(!stationary.observe(tick, {.clock = false, .frame_sync = tick % 2 == 0}),
            "Sync changes without a sample edge deliver no sample");
  }
}

void explicitly_ordered_observations() {
  SerialPcmReceiver receiver({.slot_bits = 1, .word_bits = 1});
  (void)receiver.observe_ordered({.ticks = 10}, {.clock = false, .frame_sync = false});
  require(!receiver.observe_ordered({.ticks = 10, .order = 1},
                                    {.clock = true, .frame_sync = true, .data = true}),
          "Explicit order can capture a first slot without moving absolute time");
  const auto saved = receiver;
  (void)receiver.observe_ordered({.ticks = 10, .order = 2}, {.clock = false, .frame_sync = false});
  const auto frame = receiver.observe_ordered({.ticks = 10, .order = 3},
                                              {.clock = true, .frame_sync = false, .data = false});
  require(frame && frame->samples[0] == -1 && frame->samples[1] == 0 && frame->first_tick == 10 &&
              frame->last_tick == 10 && frame->first_order == 1 && frame->last_order == 3,
          "Literal same-tick slots retain their actual causal stamps, not invented time");
  const auto completed = receiver;
  receiver = saved;
  (void)receiver.observe_ordered({.ticks = 10, .order = 2}, {.clock = false, .frame_sync = false});
  require(receiver.observe_ordered({.ticks = 10, .order = 3},
                                   {.clock = true, .frame_sync = false, .data = false}) == frame &&
              receiver == completed,
          "A trusted copy retains ordinal, partial word and all receiver state");
  require(rejects([&] {
            (void)receiver.observe_ordered({.ticks = 10, .order = 3},
                                           {.clock = false, .frame_sync = false});
          }).find("Non-increasing") != std::string::npos,
          "An identical ordinal cannot replay an observation as a new edge");
  SerialPcmReceiver boundary({});
  (void)boundary.observe_ordered({.ticks = 10, .order = std::numeric_limits<std::uint64_t>::max()},
                                 {.clock = false, .frame_sync = false});
  require(!boundary.observe_ordered({.ticks = 11}, {.clock = false, .frame_sync = false}),
          "A later absolute tick starts its own ordinal without arithmetic carry");
  require(rejects([&] {
            (void)boundary.observe_ordered(
                {.ticks = 10, .order = std::numeric_limits<std::uint64_t>::max()},
                {.clock = false, .frame_sync = false});
          }).find("Non-increasing") != std::string::npos,
          "An ordinal never permits absolute time to move backwards");
}
}
}

int main() {
  try {
    using namespace blofeld::test::pcm;
    literal_words_and_edges();
    signed_widths_and_channel_bound();
    midframe_join_and_incomplete_prefix();
    snapshots_and_failure_recovery();
    strict_signal_and_frame_failures();
    formats_times_and_nonedges();
    explicitly_ordered_observations();
    native_serial_pcm();
    std::cout << "Serial PCM literals, signed widths, framing, quarantine, snapshots and native "
                 "ESAI passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
