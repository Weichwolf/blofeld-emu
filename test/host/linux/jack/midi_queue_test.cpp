// SPDX-License-Identifier: GPL-3.0-only
#include "linux/jack/midi_queue.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <vector>

namespace {
using blofeld::host::audio::JackMidiQueue;

void require(const bool valid, const char *message) {
  if (!valid) { throw std::runtime_error(message); }
}

void wrap() {
  JackMidiQueue queue;
  const std::vector<std::uint8_t> fill(JackMidiQueue::capacity - 2, 0x01);
  constexpr std::array<std::uint8_t, 6> sysex{0xf0, 0x7e, 0x7f, 0x06, 0x01, 0xf7};
  constexpr std::uint64_t frame = 37;
  const std::span expected(sysex);
  require(queue.push(13, fill) && !queue.push(frame, sysex),
          "A complete event must fit before the writer publishes any bytes");
  queue.consume(fill.size());
  require(queue.push(frame, sysex), "A released byte ring can wrap an entire SysEx event");
  const auto message = queue.peek();
  require(message && message->frame == frame &&
              std::ranges::equal(message->first, expected.first(2)) &&
              std::ranges::equal(message->second, expected.subspan(2)),
          "The frame offset and every byte survive wrapping");
  queue.consume(2);
  const auto suffix = queue.peek();
  require(suffix && suffix->frame == frame && suffix->size() == 4,
          "DIN can release a prefix while retaining the event frame");
  queue.consume(4);
  require(!queue.peek(), "Each event is retired exactly once");
}
}

int main() try { wrap(); } catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
