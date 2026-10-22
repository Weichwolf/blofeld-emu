// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/firmware.hpp"
#include "original_usb_peer_fixture.hpp"
#include "usb_host_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <vector>

namespace blofeld::test::usb_peer {
using usb_host::require;

namespace {
struct Record {
  Event event;
  unsigned length;
};

constexpr std::array<Record, 16> records{{
    {.event = {0, 0x90, 60, 100}, .length = 0},
    {.event = {1, 0x91, 61, 99}, .length = 0},
    {.event = {2, 0xf1, 7, 0x55}, .length = 2},
    {.event = {3, 0xf2, 1, 2}, .length = 3},
    {.event = {4, 0xf0, 1, 2}, .length = 3},
    {.event = {5, 0xf7, 0x55, 0x66}, .length = 1},
    {.event = {6, 1, 0xf7, 0x66}, .length = 2},
    {.event = {7, 1, 2, 0xf7}, .length = 3},
    {.event = {8, 0x80, 60, 100}, .length = 3},
    {.event = {9, 0x90, 60, 100}, .length = 3},
    {.event = {10, 0xa2, 62, 98}, .length = 3},
    {.event = {11, 0xb3, 7, 97}, .length = 3},
    {.event = {12, 0xc4, 5, 0xa5}, .length = 2},
    {.event = {13, 0xd5, 96, 0xa5}, .length = 2},
    {.event = {14, 0xe6, 1, 95}, .length = 3},
    {.event = {15, 0xf8, 0x55, 0x66}, .length = 1},
}};
constexpr unsigned case_count = 169;
constexpr std::array<unsigned, 9> raw_counts{0, 1, 2, 3, 5, 7, 127, 128, 255};
constexpr std::array<unsigned, 5> frame_counts{0, 1, 2, 3, 8};
constexpr std::array<unsigned, 3> frame_flags{0, 0x50, 0xc5};
constexpr unsigned note_record = 9;

Case events(const std::vector<Record> &supplied) {
  Case result;
  bool active = true;
  for (const auto &record : supplied) {
    const auto offset = static_cast<unsigned>(result.packet.size());
    result.packet.insert(result.packet.end(), record.event.begin(), record.event.end());
    if (record.length == 0) { active = false; }
    if (!active) { continue; }
    result.headers.push_back(offset);
    result.lengths.push_back(record.length);
    result.expected.insert(
        result.expected.end(), record.event.begin() + 1, record.event.begin() + 1 + record.length);
    result.activity = true;
    result.multi = result.multi || record.length > 1;
  }
  return result;
}

unsigned execute(const Firmware &firmware, const unsigned id, const Case &input) {
  Rig rig(firmware, input);
  rig.capture_boundary();
  const auto saved = rig.save();
  std::cout << "PREFIX " << id << ' ' << saved.stats.instructions << ' ' << saved.stats.irqs << ' '
            << saved.stats.strobes << '\n';
  std::optional<Outcome> first;
  for (unsigned attempt = 0; attempt < 3; ++attempt) {
    if (attempt != 0) { rig.restore(saved); }
    rig.complete();
    const auto actual = rig.outcome();
    if (!first) { first = actual; }
    require(actual == *first,
            "Two joint restores reproduce complete native CPU/RAM/USB knowledge, pads and strobes");
    const auto &stats = rig.stats();
    std::cout << "CASE " << id << ' ' << attempt << ' ' << stats.instructions << ' ' << stats.irqs
              << ' ' << stats.strobes << ' ' << (input.missing_j || input.truncated) << '\n';
  }
  return id + 1;
}

unsigned gated(const Firmware &firmware, unsigned id, const Case &base) {
  for (const bool rtie : {false, true}) {
    for (const auto phase : {5U, 7U}) {
      auto input = base;
      input.rtie = rtie;
      input.phase = phase;
      id = execute(firmware, id, input);
    }
  }
  return id;
}

void suite(const Firmware &firmware) {
  unsigned id = 0;
  for (const auto &record : records) { id = gated(firmware, id, events({record})); }
  const std::array<std::vector<Record>, 7> sequences{{
      {records[9], records[8]},
      {records[9], records[0], records[8]},
      {records[9], records[1], records[8]},
      {records[15], records[12], records[13]},
      std::vector<Record>(16, records[9]),
      {records[0], records[9]},
      {records[1], records[15]},
  }};
  for (const auto &sequence : sequences) { id = gated(firmware, id, events(sequence)); }
  for (const bool rtie : {false, true}) {
    auto input = events({records[note_record]});
    input.repeat = true;
    input.rtie = rtie;
    id = execute(firmware, id, input);
  }
  for (const auto count : raw_counts) {
    auto input = events(std::vector<Record>(32, records[note_record]));
    input.mode = Mode::forward;
    input.count = count;
    const auto consumed = count == 0 || count >= 128 ? 0U : (count + 3) / 4;
    input.headers.resize(consumed);
    input.lengths.resize(consumed);
    input.expected.resize(static_cast<std::size_t>(consumed) * 3);
    input.activity = consumed != 0;
    input.multi = consumed != 0;
    id = gated(firmware, id, input);
  }
  constexpr std::array<std::uint8_t, 8> frame{0x90, 60, 100, 1, 2, 3, 4, 5};
  for (const auto count : frame_counts) {
    for (const auto flags : frame_flags) {
      const auto length = std::max(1U, count);
      const Case input{.mode = Mode::frame,
                       .packet = {frame.begin(), frame.end()},
                       .expected = {frame.begin(), frame.begin() + length},
                       .headers = {},
                       .lengths = {},
                       .count = count,
                       .ccr = flags,
                       .activity = true};
      id = execute(firmware, id, input);
    }
  }
  for (unsigned index = 8; index <= 14; ++index) {
    for (const auto flags : {0x40U, 0x50U}) {
      const auto &record = records[index];
      const Case input{
          .mode = Mode::dispatch,
          .packet = {record.event.begin() + 1, record.event.end()},
          .expected = {record.event.begin() + 1, record.event.begin() + 1 + record.length},
          .headers = {},
          .lengths = {},
          .count = record.length,
          .ccr = flags,
          .activity = true,
          .clear_tail = record.length == 2};
      id = execute(firmware, id, input);
    }
  }
  for (const auto mode : {Mode::usb, Mode::frame, Mode::dispatch}) {
    auto input = events({records[note_record]});
    input.missing_j = true;
    input.mode = mode;
    if (mode != Mode::usb) {
      input.packet = {records[note_record].event.begin() + 1, records[note_record].event.end()};
      input.headers.clear();
      input.lengths.clear();
      input.count = 3;
      input.multi = false;
    }
    id = execute(firmware, id, input);
  }
  for (unsigned size = 1; size < 4; ++size) {
    auto input = events({records[note_record]});
    input.truncated = true;
    input.packet.resize(size);
    input.expected.resize(size - 1);
    id = execute(firmware, id, input);
  }
  id = gated(firmware, id, events({}));
  require(id == case_count, "The complete fixed original peer/OUT matrix executes");
}
}
}

int main(const int count, char **arguments) {
  try {
    blofeld::test::usb_host::require(count == 2, "Usage: original-usb-peer original.zip");
    blofeld::test::usb_peer::suite(blofeld::Firmware::load(arguments[1]));
    std::cout
        << "SUMMARY cases=169 restores=338 own-gpio-endpoints=1 original-boot=0 physical-peer=0\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
