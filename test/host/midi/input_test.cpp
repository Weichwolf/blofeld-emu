// SPDX-License-Identifier: GPL-3.0-only
#include "midi/input.hpp"
#include "midi/ump.hpp"

#include <CoreFoundation/CFString.h>
#include <CoreMIDI/MIDIMessages.h>
#include <CoreMIDI/MIDIServices.h>
#include <MacTypes.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <mach/kern_return.h>
#include <mach/mach_time.h>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>

namespace {
using blofeld::host::midi::Input;
constexpr std::uint8_t note = 60, on_velocity = 100, off_velocity = 64;
constexpr std::uint8_t note_on = 0x90, note_off = 0x80;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

struct VirtualSource {
  MIDIClientRef client = 0;
  MIDIEndpointRef endpoint = 0;
  std::int32_t id = 0;

  VirtualSource() {
    require(MIDIClientCreate(CFSTR("blofeld unit source"), nullptr, nullptr, &client) == noErr,
            "Create a real CoreMIDI test client");
    require(MIDISourceCreateWithProtocol(
                client, CFSTR("blofeld unit DIN"), kMIDIProtocol_1_0, &endpoint) == noErr &&
                MIDIObjectGetIntegerProperty(endpoint, kMIDIPropertyUniqueID, &id) == noErr,
            "Create an isolated native virtual source with an actual identifier");
  }

  ~VirtualSource() { MIDIClientDispose(client); }

  void send(const std::span<const std::uint32_t> words, const std::uint64_t timestamp) const {
    MIDIEventList events{};
    auto *packet = MIDIEventListInit(&events, kMIDIProtocol_1_0);
    packet =
        MIDIEventListAdd(&events, sizeof(events), packet, timestamp, words.size(), words.data());
    require(packet != nullptr && MIDIReceivedEventList(endpoint, &events) == noErr,
            "Dispatch actual timestamped MIDI through CoreMIDI");
  }
};

void native_delivery() {
  const VirtualSource source;
  const auto sources = Input::sources();
  require(
      std::ranges::any_of(
          sources, [&](const auto &entry) { return entry.id == source.id && !entry.name.empty(); }),
      "Native discovery reports the actual selected endpoint");
  Input input({.source = source.id});
  constexpr std::size_t rounds = 4096;
  const auto before = Input::now();
  const auto timestamp = mach_absolute_time();
  const auto after = Input::now();
  mach_timebase_info_data_t timebase{};
  require(mach_timebase_info(&timebase) == KERN_SUCCESS,
          "Query independent native timestamp scale");
  const auto expected_time = (timestamp / timebase.denom * timebase.numer) +
                             (timestamp % timebase.denom * timebase.numer / timebase.denom);
  require(before <= expected_time && expected_time <= after,
          "The source and queried host clock share the same epoch and scale");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  for (std::size_t index = 0; index < rounds; ++index) {
    const std::array words{MIDI1UPNoteOn(0, 0, note, on_velocity),
                           MIDI1UPNoteOff(0, 0, note, off_velocity)};
    source.send(words, timestamp);
    for (const std::uint8_t status : {note_on, note_off}) {
      std::optional<blofeld::host::midi::Packet> event;
      while (!(event = input.read())) {
        require(std::chrono::steady_clock::now() < deadline, "Native delivery cannot stall");
        std::this_thread::yield();
      }
      const auto velocity =
          static_cast<std::uint8_t>(status == note_on ? on_velocity : off_velocity);
      require(event->timestamp == expected_time &&
                  std::ranges::equal(event->data(), std::array{status, note, velocity}),
              "Native callbacks and bounded queue reuse preserve each note and timestamp");
    }
  }
  input.check();
  require(!input.read(), "Native input creates no extra data");
}

void sysex_packets() {
  const VirtualSource source;
  Input input({.source = source.id});
  const auto timestamp = mach_absolute_time();
  const auto start = MIDI1UPSysEx(0, kMIDISysExStatusStart, 6, 1, 2, 3, 4, 5, 6);
  const auto end = MIDI1UPSysEx(0, kMIDISysExStatusEnd, 2, 7, 8, 0, 0, 0, 0);
  const std::array first{MIDI1UPNoteOn(15, 0, note, on_velocity),
                         start.word0,
                         start.word1,
                         MIDI1UPSystemCommon(0, kMIDIStatusTimingClock, 0, 0)};
  const std::array last{end.word0, end.word1};
  MIDIEventList events{};
  auto *packet = MIDIEventListInit(&events, kMIDIProtocol_1_0);
  packet = MIDIEventListAdd(&events, sizeof(events), packet, timestamp, first.size(), first.data());
  require(packet != nullptr, "Build the first selected-group packet and an unselected note");
  packet =
      MIDIEventListAdd(&events, sizeof(events), packet, timestamp + 1, last.size(), last.data());
  require(packet != nullptr && MIDIReceivedEventList(source.endpoint, &events) == noErr,
          "Dispatch actual split SysEx in two native timestamped packets");
  constexpr std::array<std::array<std::uint8_t, 7>, 3> expected{
      {{0xf0, 1, 2, 3, 4, 5, 6}, {0xf8}, {7, 8, 0xf7}}};
  constexpr std::array<std::size_t, 3> sizes{7, 1, 3};
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  std::uint64_t previous = 0;
  for (std::size_t index = 0; index < expected.size(); ++index) {
    std::optional<blofeld::host::midi::Packet> message;
    while (!(message = input.read())) {
      require(std::chrono::steady_clock::now() < deadline, "Native split SysEx cannot stall");
      std::this_thread::yield();
    }
    require(message->timestamp >= previous &&
                std::ranges::equal(message->data(), std::span(expected[index]).first(sizes[index])),
            "Selected-group SysEx retains framing and realtime interleaving across native packets");
    previous = message->timestamp;
  }
  require(!input.read(), "The other group's note is not routed to this DIN input");
}

void expect_failure(const Input &input, const std::string_view reason) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  for (;;) {
    try {
      input.check();
    } catch (const std::runtime_error &error) {
      require(std::string_view(error.what()).contains(reason), "Report the actual native failure");
      return;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      throw std::runtime_error("Native MIDI failure was not reported: " + std::string(reason));
    }
    std::this_thread::yield();
  }
}

void malformed_and_disconnect() {
  {
    const VirtualSource source;
    const Input input({.source = source.id});
    source.send(std::array{MIDI1UPNoteOn(0, 0, note, on_velocity)}, mach_absolute_time());
    source.send(std::array{MIDI1UPNoteOn(0, 0, note, on_velocity)}, 1);
    expect_failure(input, "timestamp");
  }
  {
    const VirtualSource source;
    const Input input({.source = source.id});
    require(MIDIEndpointDispose(source.endpoint) == noErr, "Remove the selected actual source");
    expect_failure(input, "disconnected");
  }
  {
    const VirtualSource source;
    const Input input({.source = source.id});
    constexpr std::size_t batches = 32;
    constexpr std::size_t batch_words = 64;
    std::array<std::uint32_t, batch_words> words{};
    words.fill(MIDI1UPNoteOn(0, 0, note, on_velocity));
    const auto timestamp = mach_absolute_time();
    for (std::size_t index = 0; index < batches; ++index) { source.send(words, timestamp); }
    expect_failure(input, "overflow");
  }
}
}

int main(const int argc, char **argv) try {
  require(argc == 2 || (argc == 3 && std::string_view(argv[2]) == "--initialize-runtime"),
          "Supply an artifact directory and optional runtime initialization selection");
  if (argc == 3) { static_cast<void>(Input::sources()); }
  native_delivery();
  sysex_packets();
  malformed_and_disconnect();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
