// SPDX-License-Identifier: GPL-3.0-only
#include "io/midi_port.hpp"
#include "midi/startup.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {
using blofeld::app::MidiStartup;
using blofeld::vi::MidiPort;
constexpr std::array<std::uint8_t, 15> identity{
    0xf0, 0x7e, 3, 6, 2, 0x3e, 0x13, 0, 0, 0, '1', '.', '2', '4', 0xf7};
constexpr std::uint64_t interval = 6000000;
constexpr std::uint64_t character_ticks = 19200;
constexpr std::uint8_t realtime_clock = 0xf8;
constexpr std::array<std::uint8_t, 4> truncated{0xf0, 0x7e, 0, 0xf7};

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void send(MidiPort &port, const auto &bytes) {
  for (const auto byte : bytes) {
    require(port.write_output({.byte = byte}),
            "Publish authored identity bytes from the output owner");
  }
}

void rejects(const auto &action) {
  bool rejected = false;
  try {
    action();
  } catch (const std::exception &) { rejected = true; }
  require(rejected, "Reject invalid destination state");
}

void startup_silence() {
  MidiPort port(MidiPort::InputMode::stream);
  MidiStartup startup;
  require(
      !startup.poll(port, {}) && port.input_limit() == interval && !port.peek_input(),
      "Before application entry the host publishes silence rather than overwriting boot status");
  port.publish_time(interval);
  require(!startup.poll(port, {.pcm = true}) && port.input_limit() == interval * 2 &&
              !port.peek_input() && startup.observations().queries == 0,
          "Early PCM alone cannot authorize an inquiry into firmware still booting");
  port.publish_time(interval * 2);
  require(!startup.poll(port, {.application = true, .pcm = true}) && port.peek_input() &&
              startup.observations().queries == 1,
          "Only actual application entry and PCM enable the ordinary identity query");
}

struct ConcurrentReadiness {
  MidiPort &port;
  bool application = false;

  static MidiStartup::Readiness read(void *owner) noexcept {
    auto &self = *static_cast<ConcurrentReadiness *>(owner);
    const MidiStartup::Readiness observed{.application = self.application, .pcm = true};
    self.application = true;
    self.port.publish_time(interval);
    return observed;
  }
};

void concurrent_readiness() {
  MidiPort port(MidiPort::InputMode::stream);
  MidiStartup startup;
  require(!startup.poll(port, {}) && port.input_limit() == interval,
          "Establish the first silent input interval");
  ConcurrentReadiness source{.port = port};
  require(!startup.poll(port, &source, ConcurrentReadiness::read) &&
              port.input_limit() == interval && startup.observations().queries == 0 &&
              !port.peek_input(),
          "Progress published after the readiness snapshot cannot authorize another silent window");
  require(!startup.poll(port, &source, ConcurrentReadiness::read) &&
              port.input_limit() == interval * 2 && startup.observations().queries == 1 &&
              port.peek_input() == MidiPort::Event{.ticks = interval, .byte = identity.front()},
          "The next coherent observation sends the actual query at the original frontier");
}

void readiness() {
  MidiPort port(MidiPort::InputMode::stream);
  MidiStartup startup;
  require(!startup.poll(port, {.application = true, .pcm = true}) && port.input_limit() == interval,
          "Grant an initial query window");
  constexpr std::array<std::uint8_t, 6> query{0xf0, 0x7e, 0x7f, 6, 1, 0xf7};
  for (const auto receiver : {MidiPort::Receiver::mcu, MidiPort::Receiver::dsp}) {
    std::uint64_t index = 0;
    for (const auto byte : query) {
      const auto event = port.peek_input(receiver);
      require(event && event->byte == byte && event->ticks == index * character_ticks,
              "Original DIN query has explicit ordered byte timing for both firmware observers");
      port.pop_input(receiver);
      ++index;
    }
  }
  require(!startup.poll(port, {.application = true, .pcm = true}) && port.input_limit() == interval,
          "PCM alone and host polling do not advance guest time");
  for (const auto byte : identity) {
    send(port, std::array{byte, realtime_clock});
    const auto ready = startup.poll(port, {.application = true});
    require(!ready, "Identity without PCM cannot establish a musical origin");
  }
  require(startup.poll(port, {.application = true, .pcm = true}) == interval,
          "Both real paths select the committed frontier");
  const auto observed = startup.observations();
  require(observed.identity && observed.queries == 1 &&
              observed.output_bytes == identity.size() * 2,
          "Startup observations count only actual queries and received bytes");
  port.publish_time(interval);
  require(startup.poll(port, {.application = true}) == interval && port.input_limit() == interval,
          "The selected origin is stable; do not send more probes");
}

void invalid_identity() {
  for (const unsigned offset : {1U, 3U, 4U, 5U, 6U, 7U, 8U, 9U}) {
    MidiPort port(MidiPort::InputMode::stream);
    MidiStartup startup;
    static_cast<void>(startup.poll(port, {.application = true, .pcm = true}));
    auto wrong = identity;
    wrong[offset] ^= 1;
    send(port, wrong);
    require(!startup.poll(port, {.application = true, .pcm = true}),
            "Unrelated or malformed identities do not establish readiness");
    send(port, identity);
    require(startup.poll(port, {.application = true, .pcm = true}) == interval,
            "A valid subsequent reply recovers framing");
  }
  MidiPort port(MidiPort::InputMode::stream);
  MidiStartup startup;
  static_cast<void>(startup.poll(port, {.application = true, .pcm = true}));
  port.publish_time(interval);
  require(!startup.poll(port, {.application = true, .pcm = true}) &&
              port.input_limit() == 2 * interval,
          "Queries repeat only after actual guest progress");
  send(port, truncated);
  require(!startup.poll(port, {.application = true, .pcm = true}),
          "Truncated replies do not establish readiness");
}
}

int main() try {
  startup_silence();
  concurrent_readiness();
  readiness();
  invalid_identity();
  MidiPort live;
  MidiStartup startup;
  rejects([&] { static_cast<void>(startup.poll(live, {.application = true, .pcm = true})); });
  MidiPort aborted(MidiPort::InputMode::stream);
  aborted.abort_input();
  rejects([&] { static_cast<void>(startup.poll(aborted, {.application = true, .pcm = true})); });
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
