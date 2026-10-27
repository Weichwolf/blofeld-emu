// SPDX-License-Identifier: GPL-3.0-only
#include "app/host_midi_session.hpp"
#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "host_midi_session_fixture.hpp"
#include "midi/host_midi_port.hpp"
#include "midi/midi_serial.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <fcntl.h>
#include <iostream>
#include <optional>
#include <sound/asequencer.h>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::session;
constexpr int client_limit = 253;
constexpr unsigned skipped = 77, suffix_steps = 8000;
constexpr std::array<std::uint8_t, 14> expected{
    0x90, 62, 99, 0xb0, 74, 111, 0xc0, 12, 0xf8, 0xf0, 0x7d, 1, 2, 0xf7};
constexpr std::array<std::uint8_t, 3> note{0x90, 62, 99};

HostMidiAddress address(const reference::HostMidiSession &session) {
  const auto result = session.address();
  if (!result) { throw Fault("Expected active virtual input port"); }
  return *result;
}

class Sender {
public:
  Sender() {
    fd_ = ::open("/dev/snd/seq", O_RDWR | O_NONBLOCK | O_CLOEXEC);
    require(fd_ >= 0, "Independent virtual MIDI sender opens");
    try {
      int client = -1;
      require(::ioctl(fd_, SNDRV_SEQ_IOCTL_CLIENT_ID, &client) == 0 && client >= 0 &&
                  client < client_limit,
              "Independent sender has an actual kernel client identity");
      snd_seq_port_info port{};
      port.addr.client = static_cast<std::uint8_t>(client);
      port.capability = SNDRV_SEQ_PORT_CAP_READ;
      port.type = SNDRV_SEQ_PORT_TYPE_MIDI_GENERIC | SNDRV_SEQ_PORT_TYPE_APPLICATION;
      require(::ioctl(fd_, SNDRV_SEQ_IOCTL_CREATE_PORT, &port) == 0,
              "Independent sender port is created");
      source_ = {.client = static_cast<unsigned>(client), .port = port.addr.port};
    } catch (...) {
      ::close(fd_);
      throw;
    }
  }

  ~Sender() { ::close(fd_); }

  Sender(const Sender &) = delete;
  Sender &operator=(const Sender &) = delete;

  void send(snd_seq_event event,
            const HostMidiAddress target,
            const std::span<const std::uint8_t> payload = {}) const {
    event.source = {.client = static_cast<std::uint8_t>(source_.client),
                    .port = static_cast<std::uint8_t>(source_.port)};
    event.dest = {.client = static_cast<std::uint8_t>(target.client),
                  .port = static_cast<std::uint8_t>(target.port)};
    event.queue = SNDRV_SEQ_QUEUE_DIRECT;
    if (!payload.empty()) {
      event.flags = SNDRV_SEQ_EVENT_LENGTH_VARIABLE;
      event.data.ext.len = static_cast<unsigned>(payload.size());
      event.data.ext.ptr = nullptr;
    }
    std::vector<std::uint8_t> record(sizeof(event) + payload.size());
    std::memcpy(record.data(), &event, sizeof(event));
    std::ranges::copy(payload, std::span{record}.subspan(sizeof(event)).begin());
    require(::write(fd_, record.data(), record.size()) == static_cast<ssize_t>(record.size()),
            "Only explicit owned virtual destinations receive the valid literal event");
  }

  void note_on(const HostMidiAddress target) const {
    snd_seq_event event{};
    event.type = SNDRV_SEQ_EVENT_NOTEON;
    event.data.note.note = note.at(1);
    event.data.note.velocity = note.back();
    send(event, target);
  }

private:
  int fd_ = -1;
  HostMidiAddress source_;
};

void native_bytes(Fixture &f,
                  reference::HostMidiSession &session,
                  const unsigned received_pc,
                  const std::span<const std::uint8_t> bytes) {
  std::size_t received = 0;
  std::size_t admitted = 0;
  while (received < bytes.size() && !f.board.finished()) {
    admitted += session.poll(f.board);
    f.board.advance();
    const auto state = f.board.status();
    if (state.mcu_state_known && state.mcu.pc == received_pc) {
      require(state.mcu.d >> 8U == bytes[received],
              "Actual native SCI status/data reads match independent literal MIDI bytes");
      ++received;
    }
  }
  require(received == bytes.size() && admitted == bytes.size(),
          "Every host byte is admitted once and received by the native MCU");
}

void host_events_to_native_sci() {
  for (const bool small : {false, true}) {
    const Program program(small);
    Fixture f(config(small), program.code);
    reference::HostMidiSession session("Blofeld emulator native input test", 1);
    require(session.poll(f.board) == 0 && !f.board.status().started,
            "Before initialization the host session cannot advance the native owner");
    const Sender sender;
    const auto target = address(session);
    sender.note_on(target);
    snd_seq_event event{};
    event.type = SNDRV_SEQ_EVENT_CONTROLLER;
    event.data.control.param = expected.at(4);
    event.data.control.value = expected.at(5);
    sender.send(event, target);
    event = {};
    event.type = SNDRV_SEQ_EVENT_PGMCHANGE;
    event.data.control.value = expected.at(7);
    sender.send(event, target);
    event = {};
    event.type = SNDRV_SEQ_EVENT_CLOCK;
    sender.send(event, target);
    event = {};
    event.type = SNDRV_SEQ_EVENT_SYSEX;
    sender.send(event, target, std::span{expected}.last(5));
    f.board.initialize();
    native_bytes(f, session, program.received_pc, expected);
    session.retire();
    require(session.retired() && !session.address() && !session.limits(),
            "Explicit retirement removes the real kernel endpoint and capabilities");
  }
}

void same_time_restore_and_owner_retirement() {
  const Program program;
  Fixture f(config(), program.code);
  f.board.initialize();
  const auto saved = f.board.checkpoint();
  const auto old_window = f.board.midi_window();
  constexpr std::uint64_t distant_poll = 1000000;
  reference::HostMidiSession session("Blofeld emulator stale input test", distant_poll);
  require(session.poll(f.board) == 0, "Initial binding sees no fabricated input");
  const Sender sender;
  sender.note_on(address(session));
  f.board.restore(saved);
  const auto new_window = f.board.midi_window();
  require(new_window.ticks == old_window.ticks &&
              new_window.restore_revision == old_window.restore_revision + 1,
          "External restore revision advances even when native time did not change");
  const auto failure = rejects([&] { static_cast<void>(session.poll(f.board)); });
  require(failure.contains("owner/restore/time") && session.retired() && !session.address() &&
              f.board.status().midi_bytes == 0,
          "Same-time restore retires the unread old host queue before its next scheduled poll");
  reference::HostMidiSession fresh("Blofeld emulator fresh input test", 1);
  require(fresh.poll(f.board) == 0, "A fresh transport cannot replay the retired kernel backlog");
  sender.note_on(address(fresh));
  native_bytes(f, fresh, program.received_pc, note);
  Fixture other(config(), program.code);
  static_cast<void>(rejects([&] { static_cast<void>(fresh.poll(other.board)); }));
  require(fresh.retired() && !fresh.address() && !other.board.status().faulted,
          "A foreign native owner cannot adopt a live MIDI session or alter guest state");
}

void backpressure_preserves_pending_message() {
  const std::vector<std::uint8_t> idle{lds, 0x20, 0, wai};
  Fixture f(config(), idle);
  f.board.initialize();
  const std::array<std::uint8_t, MidiSerialInput::capacity> full{};
  f.board.enqueue_midi(full);
  reference::HostMidiSession session("Blofeld emulator bounded input test", 1);
  const Sender sender;
  sender.note_on(address(session));
  require(session.poll(f.board) == 0 && f.board.midi_window().available == 0,
          "A full modeled wire queue cannot consume an unread kernel message");
  for (const auto byte : note) {
    while (f.board.midi_window().available == 0) { f.board.advance(); }
    require(f.board.midi_window().available == 1 && session.poll(f.board) == 1 &&
                f.board.midi_window().available == 0,
            "Each newly free byte admits exactly one byte of the pending MIDI message");
    std::ostringstream hex;
    hex << std::hex << static_cast<unsigned>(byte);
    require(f.trace.str().ends_with("bytes=" + hex.str() + "\n"),
            "Partial kernel decode retains the independent 90/3e/63 ordering");
  }
  require(f.board.status().midi_bytes == full.size() + note.size(),
          "Backpressure never drops, duplicates or substitutes an admitted byte");
}

void guest_fault_retires_input() {
  auto c = config();
  c.midi_reference.reset();
  const Program program;
  Fixture f(c, program.code);
  f.board.initialize();
  reference::HostMidiSession session("Blofeld emulator strict SCI input test", 1);
  const Sender sender;
  sender.note_on(address(session));
  require(session.poll(f.board) == note.size(), "One real host message enters the timed wire");
  const auto failure = rejects([&] {
    for (unsigned i = 0; i < suffix_steps; ++i) { f.board.advance(); }
  });
  require(failure.contains("Unverified SCI stop-to-data"),
          "Host transport cannot mask the existing unresolved hardware transfer phase");
  static_cast<void>(rejects([&] { static_cast<void>(session.poll(f.board)); }));
  require(session.retired() && !session.address(), "A faulted guest cannot retain a live input");
}

void source_and_host_failures() {
  static_cast<void>(rejects([] { const reference::HostMidiSession invalid("invalid period", 0); }));
  const std::vector<std::uint8_t> idle{lds, 0x20, 0, wai};
  auto c = config();
  c.midi_sci_timed.reset();
  c.midi_reference.reset();
  Fixture absent(c, idle);
  reference::HostMidiSession unconfigured("Blofeld emulator absent input test", 1);
  static_cast<void>(rejects([&] { static_cast<void>(unconfigured.poll(absent.board)); }));
  require(unconfigured.retired() && !absent.board.status().faulted,
          "An absent modeled route cannot silently accept host input or change native state");
  c = config();
  c.operations = 0;
  Fixture exhausted(c, idle);
  exhausted.board.initialize();
  reference::HostMidiSession finished("Blofeld emulator finished input test", 1);
  const Sender sender;
  sender.note_on(address(finished));
  static_cast<void>(rejects([&] { static_cast<void>(finished.poll(exhausted.board)); }));
  require(finished.retired() && exhausted.board.status().midi_bytes == 0,
          "An exhausted owner cannot consume pending host traffic");
  Fixture trace_failure(config(), idle);
  trace_failure.board.initialize();
  reference::HostMidiSession traced("Blofeld emulator trace failure input test", 1);
  sender.note_on(address(traced));
  trace_failure.trace.setstate(std::ios::badbit);
  const auto failure = rejects([&] { static_cast<void>(traced.poll(trace_failure.board)); });
  const auto state = trace_failure.board.status();
  require(traced.retired() && state.faulted && state.midi_bytes == note.size() &&
              state.midi_queued == note.size() && state.mcu_cycles == 0 &&
              failure.contains("trace output failed"),
          "Failure after host consumption quarantines the native queue and forbids retry");
  Fixture host_failure(config(), idle);
  host_failure.board.initialize();
  reference::HostMidiSession unsupported("Blofeld emulator unsupported input test", 1);
  snd_seq_event event{};
  event.type = SNDRV_SEQ_EVENT_USR0;
  sender.send(event, address(unsupported));
  const auto transport = rejects([&] { static_cast<void>(unsupported.poll(host_failure.board)); });
  require(unsupported.retired() && !unsupported.address() && !host_failure.board.status().faulted &&
              host_failure.board.status().midi_bytes == 0 &&
              transport.contains("Unsupported host MIDI sequencer event"),
          "A real unsupported kernel event retires its session before changing native input");
}

void tool_input() {
  const Program program;
  Fixture f(config(), program.code);
  f.board.initialize();
  reference::HostMidiSession session("Blofeld emulator standard tool input", 1);
  const auto target = address(session);
  std::cout << "MIDI input=" << target.client << ':' << target.port << '\n' << std::flush;
  std::string command;
  require(static_cast<bool>(std::getline(std::cin, command)) && command == "sent",
          "External standard-tool driver confirms delivery only to the owned input port");
  native_bytes(f, session, program.received_pc, expected);
  session.retire();
  std::cout << "Standard MIDI tool/native SCI bytes passed\n" << std::flush;
}
}

int main(const int count, char **arguments) {
  if (::access("/dev/snd/seq", R_OK | W_OK) != 0) {
    std::cout << "SKIP: ALSA sequencer unavailable\n";
    return skipped;
  }
  try {
    if (count == 2 && std::string_view(arguments[1]) == "--tool-input") {
      tool_input();
      return 0;
    }
    require(count == 1, "Unexpected host-session test argument");
    host_events_to_native_sci();
    same_time_restore_and_owner_retirement();
    backpressure_preserves_pending_message();
    guest_fault_retires_input();
    source_and_host_failures();
    std::cout << "Host MIDI session/native SCI, bounded input, restore and fault checks pass\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
