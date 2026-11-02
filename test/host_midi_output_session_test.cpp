// SPDX-License-Identifier: GPL-3.0-only
#include "app/host_midi_session.hpp"
#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "host_midi_session_fixture.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_midi_output.hpp"
#include "midi/host_midi_port.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <fcntl.h>
#include <iostream>
#include <sound/asequencer.h>
#include <span>
#include <string_view>
#include <sys/ioctl.h>
#include <unistd.h>
#include <utility>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::session;
constexpr unsigned skipped = 77;
constexpr int client_limit = 253;
constexpr unsigned first_note = 60, first_velocity = 100;
constexpr std::uint8_t poll_retry = 0xf9, wait_retry = 0xfd;
constexpr std::uint64_t distant_poll = 1000000000;
constexpr std::uint8_t clock_byte = 0xf8;
constexpr std::array<std::uint8_t, 12> stream{
    0x90, 60, 100, 61, 101, clock_byte, 0xf0, 0x7d, 1, 2, 3, 0xf7};
constexpr std::array<std::uint8_t, 6> sysex{0xf0, 0x7d, 1, 2, 3, 0xf7};

class Receiver {
public:
  explicit Receiver(const HostMidiAddress source) {
    fd_ = ::open("/dev/snd/seq", O_RDWR | O_NONBLOCK | O_CLOEXEC);
    require(fd_ >= 0, "Independent local MIDI observer opens");
    try {
      int client = -1;
      require(::ioctl(fd_, SNDRV_SEQ_IOCTL_CLIENT_ID, &client) == 0 && client >= 0 &&
                  client < client_limit,
              "Observer identity comes from the kernel");
      snd_seq_port_info port{};
      port.addr.client = static_cast<std::uint8_t>(client);
      port.capability = SNDRV_SEQ_PORT_CAP_WRITE | SNDRV_SEQ_PORT_CAP_SUBS_WRITE;
      port.type = SNDRV_SEQ_PORT_TYPE_APPLICATION;
      require(::ioctl(fd_, SNDRV_SEQ_IOCTL_CREATE_PORT, &port) == 0,
              "Independent observer port is created");
      snd_seq_port_subscribe route{};
      route.sender = {.client = static_cast<std::uint8_t>(source.client),
                      .port = static_cast<std::uint8_t>(source.port)};
      route.dest = port.addr;
      require(::ioctl(fd_, SNDRV_SEQ_IOCTL_SUBSCRIBE_PORT, &route) == 0,
              "Only the new session's owned virtual source is connected");
    } catch (...) {
      ::close(fd_);
      throw;
    }
  }

  ~Receiver() { ::close(fd_); }

  Receiver(const Receiver &) = delete;
  Receiver &operator=(const Receiver &) = delete;

  struct Record {
    snd_seq_event event{};
    std::vector<std::uint8_t> data;
  };

  [[nodiscard]] std::vector<Record> read() const {
    std::vector<Record> result;
    constexpr std::size_t header = sizeof(snd_seq_event);
    std::array<std::uint8_t, header * 1024> buffer{};
    while (true) {
      const auto count = ::read(fd_, buffer.data(), buffer.size());
      if (count < 0 && errno == EAGAIN) { break; }
      if (count < 0 && errno == EINTR) { continue; }
      require(count > 0, "Observer read has no hidden transport loss");
      const auto size = static_cast<std::size_t>(count);
      std::size_t at = 0;
      while (at < size) {
        require(size - at >= header, "Kernel observation contains a whole event header");
        Record record;
        std::memcpy(&record.event, buffer.data() + at, header);
        at += header;
        const auto length = record.event.flags & SNDRV_SEQ_EVENT_LENGTH_MASK;
        if (length == SNDRV_SEQ_EVENT_LENGTH_VARIABLE) {
          const auto bytes = record.event.data.ext.len;
          const auto padded = ((std::size_t{bytes} + header - 1) / header) * header;
          require(padded <= size - at, "Kernel SysEx payload and alignment are complete");
          const auto data = std::span(buffer).subspan(at, bytes);
          record.data.assign(data.begin(), data.end());
          at += padded;
        } else {
          require(length == SNDRV_SEQ_EVENT_LENGTH_FIXED, "No external payload pointers accepted");
        }
        if (record.event.type != SNDRV_SEQ_EVENT_PORT_SUBSCRIBED &&
            record.event.type != SNDRV_SEQ_EVENT_PORT_UNSUBSCRIBED) {
          result.push_back(std::move(record));
        }
      }
    }
    return result;
  }

private:
  int fd_ = -1;
};

HostMidiAddress address(const reference::HostMidiSession &session) {
  const auto result = session.address();
  if (!result) { throw Fault("The session requires an actual current endpoint"); }
  return *result;
}

reference::Config output_config(const bool small = false) {
  auto result = config(small);
  result.midi_output = small ? SciTransmitPin::ps5 : SciTransmitPin::pr3;
  return result;
}

std::vector<std::uint8_t> transmit(const std::span<const std::uint8_t> bytes,
                                   const bool small = false) {
  Program p;
  p.code = {lds, 0x20, 0};
  p.write(hc12::modrr, small ? 1 : 0);
  p.write(hc12::sci_baud_low, divisor);
  p.write(hc12::sci_control2, 8);
  for (const auto byte : bytes) {
    // Native TDRE polling precedes each data write. No host drives the TX pad.
    p.code.insert(p.code.end(),
                  {load,
                   0,
                   static_cast<std::uint8_t>(hc12::sci_status1),
                   bit_test,
                   0x80,
                   equal_branch,
                   poll_retry});
    p.write(hc12::sci_data_low, byte);
  }
  p.code.insert(p.code.end(), {wai, branch, wait_retry});
  return p.code;
}

void queue_clock(Fixture &f) {
  while (f.board.midi_output_window().queued_bytes == 0 && !f.board.finished()) {
    f.board.advance();
  }
  require(f.board.midi_output_window().queued_bytes == 1,
          "Actual reset/SCI/WAI publishes one complete clock byte");
}

void connected(const bool small) {
  Fixture f(output_config(small), transmit(stream, small));
  reference::HostMidiSession session("Blofeld native MIDI output test", distant_poll);
  const Receiver receiver(address(session));
  require(session.poll(f.board) == 0, "No host input or output is fabricated before reset");
  while (f.board.midi_output_window().transmitted_bytes < stream.size() && !f.board.finished()) {
    f.board.advance();
    require(session.poll(f.board) == 0, "Output service never invents received input bytes");
  }
  const auto final = f.board.status();
  require(final.midi_output_bytes == stream.size() && final.midi_output_queued == 0 &&
              final.midi_bytes == 0 && final.waiting && !session.retired(),
          "All actual SCI bytes drain despite a distant input poll deadline");
  const auto messages = receiver.read();
  require(messages.size() == 4, "Two notes, a clock and one complete SysEx reach the kernel");
  for (unsigned i = 0; i < 2; ++i) {
    const auto &event = messages.at(i).event;
    require(event.type == SNDRV_SEQ_EVENT_NOTEON && event.data.note.channel == 0 &&
                event.data.note.note == first_note + i &&
                event.data.note.velocity == first_velocity + i,
            "Independent event fields preserve native status/running-status note ordering");
  }
  require(messages.at(2).event.type == SNDRV_SEQ_EVENT_CLOCK &&
              messages.back().event.type == SNDRV_SEQ_EVENT_SYSEX &&
              std::ranges::equal(messages.back().data, sysex),
          "Independent raw kernel records retain clock and complete literal SysEx");
  require(session.flush_output(f.board) == 0 && receiver.read().empty(),
          "An empty model queue cannot repeat an already delivered host prefix");
}

void restore_and_owner() {
  const std::array<std::uint8_t, 1> clock{clock_byte};
  Fixture f(output_config(), transmit(clock));
  queue_clock(f);
  const auto queued = f.board.checkpoint();
  reference::HostMidiSession old("Blofeld MIDI restore output test", 1);
  const Receiver receiver(address(old));
  require(old.flush_output(f.board) == 1, "One queued byte is consumed exactly once");
  const auto first = receiver.read();
  require(first.size() == 1 && first.front().event.type == SNDRV_SEQ_EVENT_CLOCK,
          "The independent observer sees the original delivered clock");
  f.board.restore(queued);
  const auto failure = rejects([&] { static_cast<void>(old.flush_output(f.board)); });
  require(failure.contains("owner/restore/time") && old.retired() && !old.address() &&
              f.board.midi_output_window().queued_bytes == 1 && receiver.read().empty(),
          "Same-time restore retires before consuming or replaying restored guest output");
  reference::HostMidiSession fresh("Blofeld MIDI fresh output test", 1);
  const Receiver fresh_receiver(address(fresh));
  require(fresh.flush_output(f.board) == 1 && fresh_receiver.read().size() == 1,
          "A new explicit session may consume the restored model's queued byte");
  auto input_only = output_config();
  input_only.midi_output.reset();
  Fixture foreign(input_only, transmit(clock));
  static_cast<void>(rejects([&] { static_cast<void>(fresh.poll(foreign.board)); }));
  require(fresh.retired() && foreign.board.status().midi_bytes == 0,
          "Output binding also prevents a foreign input-only owner adopting the endpoint");
}

void completion_and_failures() {
  const std::array<std::uint8_t, 1> clock{clock_byte};
  Fixture f(output_config(), transmit(clock));
  while (!f.board.finished()) { f.board.advance(); }
  reference::HostMidiSession final("Blofeld final MIDI output test", 1);
  const Receiver receiver(address(final));
  require(final.flush_output(f.board) == 1 && receiver.read().size() == 1,
          "Final output can drain after the operation budget without advancing native time");
  Fixture absent(config(), transmit(clock));
  reference::HostMidiSession unavailable("Blofeld absent MIDI output test", 1);
  const auto missing = rejects([&] { static_cast<void>(unavailable.flush_output(absent.board)); });
  require(missing.contains("configured board output") && unavailable.retired() &&
              !absent.board.status().faulted,
          "An unconfigured output refuses visibly without changing guest state");
  Fixture trace(output_config(), transmit(clock));
  queue_clock(trace);
  reference::HostMidiSession traced("Blofeld MIDI trace output test", 1);
  const Receiver trace_receiver(address(traced));
  trace.trace.setstate(std::ios::badbit);
  const auto failed = rejects([&] { static_cast<void>(traced.flush_output(trace.board)); });
  require(failed.contains("trace failed after queue retirement") && traced.retired() &&
              trace.board.status().faulted && trace_receiver.read().empty(),
          "A failure after model retirement quarantines the session and emits no fake response");
}
}

int main() {
  if (::access("/dev/snd/seq", R_OK | W_OK) != 0) {
    std::cout << "SKIP: ALSA sequencer unavailable\n";
    return skipped;
  }
  try {
    connected(false);
    connected(true);
    restore_and_owner();
    completion_and_failures();
    std::cout << "Native MCU MIDI output to owned host port: both pads, running status, SysEx, "
                 "independent kernel records, restore, final drain and failure retirement pass\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
