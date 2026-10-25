// SPDX-License-Identifier: GPL-3.0-only
#include "firmware/firmware.hpp"
#include "original_midi_dispatch_fixture.hpp"
#include "original_midi_forward_fixture.hpp"
#include "usb_host_fixture.hpp"

#include <array>
#include <cstddef>
#include <exception>
#include <iostream>
#include <optional>
#include <utility>
#include <vector>

namespace blofeld::test::midi_dispatch {
namespace mf = midi_forward;
using usb_host::require;

namespace {
struct Message {
  Event direct;
  Event mirror;
  unsigned length;
};

constexpr std::array<Message, 7> messages{{
    {.direct = {8, 0x80, 60, 100}, .mirror = {8, 0x80, 60, 100}, .length = 3},
    {.direct = {9, 0x91, 61, 99}, .mirror = {9, 0x91, 61, 99}, .length = 3},
    {.direct = {10, 0xa2, 62, 98}, .mirror = {10, 0xa2, 62, 98}, .length = 3},
    {.direct = {11, 0xb3, 7, 97}, .mirror = {11, 0xb3, 7, 97}, .length = 3},
    {.direct = {12, 0xc4, 5, 0}, .mirror = {12, 0xc4, 5, 0x49}, .length = 2},
    {.direct = {13, 0xd5, 96, 0}, .mirror = {13, 0xd5, 96, 0x49}, .length = 2},
    {.direct = {14, 0xe6, 1, 95}, .mirror = {14, 0xe6, 1, 95}, .length = 3},
}};
constexpr std::array<unsigned, 7> ordinary_routes{0, 1, 2, 3, 4, 6, 7};
constexpr std::array<unsigned, 7> chunk_routes{0x10, 0x11, 0x12, 0x13, 0x1a, 0x1b, 0x1f};
constexpr std::array<unsigned, 4> high_routes{0x80, 0x81, 0x82, 0x83};
constexpr unsigned case_count = 242;

Bytes payload(const Event &event) {
  return {event.begin() + 1, event.end()};
}

Case ordinary(const Message &message,
              const unsigned selector,
              const unsigned cursor,
              const bool mirror) {
  Case result{.source = payload(message.direct),
              .mutated = payload(message.direct),
              .tx = {},
              .events = {},
              .packets = {},
              .uart_calls = {},
              .usb_calls = {},
              .selector = selector,
              .cursor = cursor,
              .mirror = mirror};
  if (message.length == 2) { result.source[2] = mf::canary; }
  const bool uart = (selector & 1U) != 0;
  const bool usb = !uart && (selector & 2U) != 0;
  if (uart) {
    result.tx = {result.mutated.begin(), result.mutated.begin() + message.length};
    result.uart_calls.push_back(
        {.pointer = source, .argument = message.length, .bytes = result.tx});
  }
  if (usb) {
    result.events.push_back(message.direct);
    result.usb_calls.push_back({.pointer = source, .argument = selector, .bytes = result.mutated});
    if (selector != 6) { result.packets.push_back(4); }
  }
  if (mirror) {
    result.events.push_back(message.mirror);
    result.packets.push_back(4);
    result.usb_calls.push_back(
        {.pointer = mf::stack - 1, .argument = 0, .bytes = payload(message.mirror), .ordinal = 1});
  }
  return result;
}

struct Chunk {
  Bytes source;
  Bytes mutated;
  Bytes tx;
  std::vector<Event> events;
  unsigned count = 3;
  unsigned pointer_advance = 0;
  unsigned deferred_packet = 0;
};

std::array<Chunk, 6> chunks() {
  constexpr unsigned eox = 0xf7;
  constexpr unsigned note_on = 0x90;
  constexpr unsigned note = 60;
  constexpr unsigned pc = 0xc4;
  constexpr unsigned pressure = 0xd5;
  constexpr unsigned padding = 0x55;
  constexpr unsigned sysex = 0xf0;
  return {{
      {.source = {eox, 1, 2},
       .mutated = {eox, 1, 2},
       .tx = {eox, 1, 2},
       .events = {{5, eox, 1, 2}},
       .deferred_packet = 4},
      {.source = {note_on, eox, 2},
       .mutated = {note_on, eox, 2},
       .tx = {note_on, eox, 2},
       .events = {{9, note_on, eox, 2}}},
      {.source = {note_on, note, eox},
       .mutated = {note_on, note, eox},
       .tx = {note_on, note, eox},
       .events = {{9, note_on, note, eox}},
       .pointer_advance = 3},
      {.source = {sysex, 1, 2, 3, 4, eox},
       .mutated = {sysex, 1, 2, 3, 4, eox},
       .tx = {sysex, 1, 2, 3, 4, eox},
       .events = {{4, sysex, 1, 2}, {7, 3, 4, eox}},
       .pointer_advance = 6,
       .deferred_packet = 8},
      {.source = {pc, 5, eox, 1, eox, padding},
       .mutated = {pc, 5, 0, 1, eox, padding},
       .tx = {pc, 5, 1, eox},
       .events = {{12, pc, 5, 0}, {6, 1, eox, padding}},
       .count = 2,
       .pointer_advance = 3,
       .deferred_packet = 8},
      {.source = {pressure, eox, mf::canary},
       .mutated = {pressure, eox, 0},
       .tx = {pressure, eox},
       .events = {{13, pressure, eox, 0}},
       .count = 2},
  }};
}

Case chunk_case(const Chunk &chunk, const unsigned selector, const unsigned cursor) {
  Case result{.source = chunk.source,
              .mutated = chunk.mutated,
              .tx = {},
              .events = {},
              .packets = {},
              .uart_calls = {},
              .usb_calls = {},
              .selector = selector,
              .cursor = cursor,
              .pointer_advance = chunk.pointer_advance};
  const bool uart = (selector & 1U) != 0;
  const bool usb = (selector & 2U) != 0;
  if (uart) { result.tx = chunk.tx; }
  unsigned ordinal = 0;
  for (unsigned index = 0; index < chunk.events.size(); ++index) {
    const auto begin = chunk.mutated.begin() + (static_cast<std::ptrdiff_t>(index) * 3);
    const auto pointer = source + (index * 3);
    if (uart) {
      result.uart_calls.push_back({.pointer = pointer,
                                   .argument = chunk.count,
                                   .bytes = {begin, begin + chunk.count},
                                   .ordinal = ordinal++});
    }
    if (usb) {
      result.usb_calls.push_back({.pointer = pointer,
                                  .argument = selector,
                                  .bytes = {begin, begin + 3},
                                  .ordinal = ordinal++});
    }
  }
  if (usb) {
    result.events = chunk.events;
    if (selector == chunk_routes.back()) {
      if (chunk.deferred_packet != 0) { result.packets.push_back(chunk.deferred_packet); }
    } else {
      result.packets.assign(chunk.events.size(), 4);
    }
  }
  return result;
}

unsigned execute(const Firmware &firmware, const unsigned id, Case input) {
  Rig rig(firmware, std::move(input));
  rig.capture_boundary();
  const auto saved = rig.save();
  std::cout << "PREFIX " << id << ' ' << saved.stats.instructions << ' ' << saved.stats.usb_irqs
            << ' ' << saved.stats.sci_irqs << '\n';
  std::optional<Outcome> first;
  for (unsigned attempt = 0; attempt < 3; ++attempt) {
    if (attempt != 0) { rig.restore(saved); }
    rig.complete();
    const auto actual = rig.outcome();
    if (!first) { first = actual; }
    require(actual == *first,
            "Two joint restores reproduce full CPU/RAM/USB/SCI and raw callee history");
    const auto &stats = rig.stats();
    std::cout << "CASE " << id << ' ' << attempt << ' ' << stats.instructions << ' '
              << stats.usb_irqs << ' ' << stats.sci_irqs << ' ' << stats.usb_calls << ' '
              << stats.uart_calls << ' ' << rig.own_instructions() << '\n';
  }
  return id + 1;
}

void suite(const Firmware &firmware) {
  unsigned id = 0;
  for (const auto &message : messages) {
    for (const auto route : ordinary_routes) {
      for (const auto cursor : {0U, mf::ring_mask}) {
        id = execute(firmware, id, ordinary(message, route, cursor, false));
      }
    }
  }
  for (const auto route : high_routes) {
    id = execute(firmware, id, ordinary(messages[1], route, mf::ring_mask, false));
  }
  for (const auto &message : messages) {
    for (const auto route : {1U, 3U, 7U}) {
      for (const auto cursor : {0U, mf::ring_mask}) {
        id = execute(firmware, id, ordinary(message, route, cursor, true));
      }
    }
  }
  for (const auto &chunk : chunks()) {
    for (const auto route : chunk_routes) {
      for (const auto cursor : {0U, mf::ring_mask}) {
        id = execute(firmware, id, chunk_case(chunk, route, cursor));
      }
    }
  }
  for (const auto &message : messages) {
    for (const auto cursor : {0U, mf::ring_mask}) {
      auto input = ordinary(message, 3, cursor, true);
      input.after_dispatch = true;
      id = execute(firmware, id, std::move(input));
    }
  }
  require(id == case_count, "The complete fixed dispatcher matrix executes");
}
}
}

int main(const int count, char **arguments) {
  try {
    blofeld::test::usb_host::require(count == 2, "Usage: original-midi-dispatch original.zip");
    blofeld::test::midi_dispatch::suite(blofeld::Firmware::load(arguments[1]));
    std::cout << "SUMMARY cases=242 restores=484 own-caller=1 original-boot=0\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
