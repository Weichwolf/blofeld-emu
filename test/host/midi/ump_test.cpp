// SPDX-License-Identifier: GPL-3.0-only
#include "midi/ump.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>

namespace {
using blofeld::host::midi::DinEncoder;
using blofeld::host::midi::Ump;
constexpr std::uint64_t timestamp = 123456;

struct Vector {
  std::array<std::uint32_t, 2> words;
  std::uint8_t word_count;
  std::array<std::uint8_t, 8> bytes;
  std::uint8_t byte_count;
};

constexpr Vector note{
    .words = {0x20903c64, 0}, .word_count = 1, .bytes = {0x90, 60, 100}, .byte_count = 3};
constexpr std::array vectors{
    note,
    Vector{.words = {0x20ef007f, 0}, .word_count = 1, .bytes = {0xef, 0, 127}, .byte_count = 3},
    Vector{.words = {0x20c00500, 0}, .word_count = 1, .bytes = {0xc0, 5}, .byte_count = 2},
    Vector{.words = {0x10f27f01, 0}, .word_count = 1, .bytes = {0xf2, 127, 1}, .byte_count = 3},
    Vector{.words = {0x10f17f00, 0}, .word_count = 1, .bytes = {0xf1, 127}, .byte_count = 2},
    Vector{.words = {0x10ff0000, 0}, .word_count = 1, .bytes = {0xff}, .byte_count = 1},
    Vector{.words = {0x30034301, 0x7f000000},
           .word_count = 2,
           .bytes = {0xf0, 0x43, 1, 0x7f, 0xf7},
           .byte_count = 5},
    Vector{.words = {0x30060102, 0x03040506},
           .word_count = 2,
           .bytes = {0xf0, 1, 2, 3, 4, 5, 6, 0xf7},
           .byte_count = 8},
    Vector{.words = {0x30000000, 0}, .word_count = 2, .bytes = {0xf0, 0xf7}, .byte_count = 2}};
constexpr Vector start{.words = {0x30164301, 0x02030405},
                       .word_count = 2,
                       .bytes = {0xf0, 0x43, 1, 2, 3, 4, 5},
                       .byte_count = 7};
constexpr Vector timing{
    .words = {0x10f80000, 0}, .word_count = 1, .bytes = {0xf8}, .byte_count = 1};
constexpr Vector continued{.words = {0x30260102, 0x03040506},
                           .word_count = 2,
                           .bytes = {1, 2, 3, 4, 5, 6},
                           .byte_count = 6};
constexpr Vector end{
    .words = {0x30320102, 0}, .word_count = 2, .bytes = {1, 2, 0xf7}, .byte_count = 3};
constexpr Vector other_group{
    .words = {0x2f803c40, 0}, .word_count = 1, .bytes = {0x80, 60, 64}, .byte_count = 3};

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void expect(Ump &decoder, const Vector &vector) {
  const auto packet = decoder.decode(std::span(vector.words).first(vector.word_count), timestamp);
  require(
      packet && packet->timestamp == timestamp &&
          std::ranges::equal(packet->data(), std::span(vector.bytes).first(vector.byte_count)) &&
          decoder.error() == nullptr,
      "UMP conversion preserves exact DIN bytes and the native timestamp");
}

void messages() {
  Ump decoder(0);
  for (const auto &vector : vectors) { expect(decoder, vector); }
  expect(decoder, start);
  expect(decoder, timing);
  constexpr std::array<std::uint32_t, 2> ignored{0x31000100, 0};
  require(!decoder.decode(ignored, timestamp) && decoder.error() == nullptr,
          "An unselected group's SysEx does not disturb the selected DIN stream");
  expect(decoder, continued);
  expect(decoder, end);
  Ump other(15);
  expect(other, other_group);
}

void encoded(const std::span<const std::uint8_t> bytes,
             const std::span<const std::uint32_t> expected,
             const std::uint8_t group = 0) {
  DinEncoder encoder(group);
  const auto *word = expected.data();
  const auto *limit = expected.data() + expected.size();
  for (const auto byte : bytes) {
    const auto batch = encoder.push(byte);
    for (const auto native : batch.data()) {
      require(word != limit && native == *word, "Unexpected outgoing UMP word");
      ++word;
    }
  }
  require(word == limit, "Missing outgoing UMP word");
}

void outgoing() {
  constexpr std::array<std::uint8_t, 12> running{
      0x90, 60, 100, 61, 0xf8, 0, 0xc0, 5, 6, 0xf2, 0x7f, 1};
  constexpr std::array<std::uint32_t, 6> channel{
      0x20903c64, 0x10f80000, 0x20903d00, 0x20c00500, 0x20c00600, 0x10f27f01};
  encoded(running, channel);
  constexpr auto group = std::uint8_t{15};
  encoded(std::span(other_group.bytes).first(other_group.byte_count),
          std::span(other_group.words).first(other_group.word_count),
          group);
  constexpr std::array<std::uint8_t, 10> complete{0xf0, 0xf7, 0xf0, 1, 2, 3, 4, 5, 6, 0xf7};
  constexpr std::array<std::uint32_t, 4> complete_words{0x30000000, 0, 0x30060102, 0x03040506};
  encoded(complete, complete_words);
  constexpr std::array<std::uint8_t, 12> fragmented{0xf0, 0x43, 1, 0xf8, 2, 3, 4, 5, 6, 7, 8, 0xf7};
  constexpr std::array<std::uint32_t, 7> fragments{
      0x30124301, 0, 0x10f80000, 0x30260203, 0x04050607, 0x30310800, 0};
  encoded(fragmented, fragments);
  constexpr std::array<std::uint8_t, 13> interrupted{
      0xf0, 1, 0xf7, 0x8c, 0xf8, 0xf0, 2, 0xf7, 0x22, 0x90, 60, 0xc0, 7};
  constexpr std::array<std::uint32_t, 6> resynchronized{
      0x30010100, 0, 0x10f80000, 0x30010200, 0, 0x20c00700};
  encoded(interrupted, resynchronized);
  const auto rejects = [](const std::span<const std::uint8_t> bytes) {
    DinEncoder encoder(0);
    try {
      for (const auto byte : bytes) { static_cast<void>(encoder.push(byte)); }
    } catch (const std::runtime_error &) { return; }
    throw std::runtime_error("Malformed DIN stream was accepted");
  };
  constexpr std::array<std::uint8_t, 1> undefined{0xf9};
  constexpr std::array<std::uint8_t, 3> broken_sysex{0xf0, 1, 0x90};
  rejects(broken_sysex);
  rejects(undefined);
}

void malformed() {
  constexpr std::array<std::uint32_t, 12> invalid{0x40903c64,
                                                  0x20908064,
                                                  0x20c00501,
                                                  0x10f40000,
                                                  0x10f28000,
                                                  0x10f80100,
                                                  0x10f10101,
                                                  0x20703c00,
                                                  0x30400000,
                                                  0x30070000,
                                                  0x30210000,
                                                  0x30010080};
  for (const auto word : invalid) {
    Ump decoder(0);
    const std::array words{word, std::uint32_t{0}};
    const auto length = word >> 28U == 3 ? 2U : 1U;
    require(!decoder.decode(std::span(words).first(length), timestamp) &&
                decoder.error() != nullptr,
            "Unsupported protocol, bad status/data/padding and orphaned SysEx fail");
    const auto *reason = decoder.error();
    require(!decoder.decode(std::span(note.words).first(note.word_count), timestamp) &&
                decoder.error() == reason,
            "A failed conversion cannot resume with an inconsistent destination stream");
  }
  Ump short_packet(0);
  constexpr std::array<std::uint32_t, 1> truncated{0x30000000};
  require(!short_packet.decode(truncated, timestamp) && short_packet.error() != nullptr,
          "Truncated 64-bit SysEx cannot read beyond its packet");
  Ump empty(0);
  require(!empty.decode({}, timestamp) && empty.error() != nullptr, "Empty input fails");
  constexpr std::array interruptions{0x20903c64U, 0x10f60000U};
  for (const auto interruption : interruptions) {
    Ump decoder(0);
    expect(decoder, start);
    require(!decoder.decode(std::array{interruption}, timestamp) && decoder.error() != nullptr,
            "Only actual realtime messages can interleave with unfinished SysEx");
  }
}
}

int main() try {
  messages();
  malformed();
  outgoing();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
