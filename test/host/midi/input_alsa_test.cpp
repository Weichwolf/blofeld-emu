// SPDX-License-Identifier: GPL-3.0-only
#include "midi/input.hpp"
#include "midi/ump.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <fcntl.h>
#include <iostream>
#include <optional>
#include <sound/asequencer.h>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <sys/ioctl.h>
#include <system_error>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {
using blofeld::host::midi::Input;
using blofeld::host::midi::Packet;
constexpr unsigned note_on = 0x90, note_off = 0x80;
constexpr unsigned note = 60, velocity = 96;
constexpr unsigned data_mask = 127, byte_mask = 255;
constexpr std::uint8_t exclusive_begin = 0xf0, exclusive_end = 0xf7, clock_status = 0xf8;
constexpr std::uint64_t uncertainty = 500000;

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw std::runtime_error(std::string(message)); }
}

struct Descriptor {
  int value = ::open("/dev/snd/seq", O_RDWR | O_CLOEXEC | O_NONBLOCK);

  Descriptor() { require(value >= 0, "Open isolated native test client"); }

  ~Descriptor() { ::close(value); }

  Descriptor(const Descriptor &) = delete;
  Descriptor &operator=(const Descriptor &) = delete;
};

void control(const int descriptor, const unsigned long request, void *value) {
  if (::ioctl(descriptor, request, value) < 0) {
    throw std::system_error(
        errno, std::generic_category(), "Native test control " + std::to_string(request));
  }
}

class Source {
public:
  explicit Source(const bool ump = true) {
    control(fd_.value, SNDRV_SEQ_IOCTL_CLIENT_ID, &client_);
    snd_seq_client_info client{};
    client.client = client_;
    control(fd_.value, SNDRV_SEQ_IOCTL_GET_CLIENT_INFO, &client);
    client.midi_version = ump ? SNDRV_SEQ_CLIENT_UMP_MIDI_1_0 : SNDRV_SEQ_CLIENT_LEGACY_MIDI;
    control(fd_.value, SNDRV_SEQ_IOCTL_SET_CLIENT_INFO, &client);
    control(fd_.value, SNDRV_SEQ_IOCTL_GET_CLIENT_INFO, &client);
    ump_ = client.midi_version == SNDRV_SEQ_CLIENT_UMP_MIDI_1_0;
    snd_seq_port_info port{};
    port.addr.client = static_cast<unsigned char>(client_);
    constexpr std::string_view name = "Isolated native MIDI unit source";
    std::ranges::copy(name, port.name);
    port.capability = SNDRV_SEQ_PORT_CAP_READ | SNDRV_SEQ_PORT_CAP_SUBS_READ;
    port.type = SNDRV_SEQ_PORT_TYPE_APPLICATION | SNDRV_SEQ_PORT_TYPE_MIDI_GENERIC;
    control(fd_.value, SNDRV_SEQ_IOCTL_CREATE_PORT, &port);
    port_ = port.addr;
  }

  [[nodiscard]] std::int32_t id() const { return (client_ << 8) | port_.port; }

  [[nodiscard]] bool universal() const { return ump_; }

  [[nodiscard]] bool send(const std::span<const std::uint32_t> words) const {
    if (!ump_) { return send_legacy(words); }
    snd_seq_ump_event event{};
    event.flags = SNDRV_SEQ_EVENT_UMP | SNDRV_SEQ_TIME_STAMP_REAL;
    event.queue = SNDRV_SEQ_QUEUE_DIRECT;
    event.source = port_;
    event.dest.client = SNDRV_SEQ_ADDRESS_SUBSCRIBERS;
    constexpr unsigned false_source_seconds = 0xffffffff;
    event.time.time.tv_sec =
        false_source_seconds; // The receiver must use its own monotonic receipt clock.
    require(words.size() <= std::size(event.ump), "Bound native test UMP storage");
    std::ranges::copy(words, event.ump);
    const auto written = ::write(fd_.value, &event, sizeof(event));
    if (written < 0 && (errno == ENOSPC || errno == ENOMEM || errno == EAGAIN)) { return false; }
    require(written == static_cast<decltype(written)>(sizeof(event)),
            "Write actual UMP source event");
    return true;
  }

  [[nodiscard]] bool send_legacy(const std::span<const std::uint32_t> words) const {
    constexpr unsigned type_shift = 28;
    constexpr unsigned status_shift = 16;
    const auto status = (words[0] >> status_shift) & byte_mask;
    if ((words[0] >> type_shift) == 3) {
      const auto descriptor = status;
      const auto count = descriptor & 15U;
      const auto kind = descriptor >> 4;
      std::vector<std::uint8_t> bytes;
      if (kind < 2) { bytes.push_back(exclusive_begin); }
      for (unsigned index = 0; index < count; ++index) {
        const auto word = index < 2 ? 0U : 1U;
        const auto shift = (index < 2 ? 1 - index : 5 - index) * 8;
        bytes.push_back(static_cast<std::uint8_t>((words[word] >> shift) & byte_mask));
      }
      if (kind == 0 || kind == 3) { bytes.push_back(exclusive_end); }
      return legacy_sysex(bytes);
    }
    snd_seq_event event{};
    event.type =
        (status & exclusive_begin) == note_on ? SNDRV_SEQ_EVENT_NOTEON : SNDRV_SEQ_EVENT_NOTEOFF;
    if (status == clock_status) { event.type = SNDRV_SEQ_EVENT_CLOCK; }
    event.flags = SNDRV_SEQ_TIME_STAMP_REAL;
    constexpr unsigned false_source_seconds = 0xffffffff;
    event.time.time.tv_sec = false_source_seconds;
    event.queue = SNDRV_SEQ_QUEUE_DIRECT;
    event.source = port_;
    event.dest.client = SNDRV_SEQ_ADDRESS_SUBSCRIBERS;
    event.data.note.channel = static_cast<std::uint8_t>(status & 15U);
    event.data.note.note = static_cast<std::uint8_t>((words[0] >> 8U) & data_mask);
    event.data.note.velocity = static_cast<std::uint8_t>(words[0] & data_mask);
    const auto written = ::write(fd_.value, &event, sizeof(event));
    if (written < 0 && (errno == ENOSPC || errno == ENOMEM || errno == EAGAIN)) { return false; }
    require(written == static_cast<decltype(written)>(sizeof(event)),
            "Write actual legacy source event");
    return true;
  }

  void legacy_note(const std::uint8_t pitch = note) const {
    snd_seq_event event{};
    event.type = SNDRV_SEQ_EVENT_NOTEON;
    event.queue = SNDRV_SEQ_QUEUE_DIRECT;
    event.source = port_;
    event.dest.client = SNDRV_SEQ_ADDRESS_SUBSCRIBERS;
    event.data.note.note = pitch;
    event.data.note.velocity = velocity;
    require(::write(fd_.value, &event, sizeof(event)) == static_cast<long>(sizeof(event)),
            "Write original legacy ALSA note event");
  }

  [[nodiscard]] bool legacy_sysex(const std::span<const std::uint8_t> data) const {
    snd_seq_event event{};
    event.type = SNDRV_SEQ_EVENT_SYSEX;
    event.flags = SNDRV_SEQ_EVENT_LENGTH_VARIABLE;
    event.queue = SNDRV_SEQ_QUEUE_DIRECT;
    event.source = port_;
    event.dest.client = SNDRV_SEQ_ADDRESS_SUBSCRIBERS;
    event.data.ext.len = static_cast<unsigned>(data.size());
    std::vector<std::uint8_t> encoded(sizeof(event) + data.size());
    std::memcpy(encoded.data(), &event, sizeof(event));
    std::ranges::copy(data, encoded.begin() + sizeof(event));
    const auto written = ::write(fd_.value, encoded.data(), encoded.size());
    if (written < 0 && (errno == ENOSPC || errno == ENOMEM || errno == EAGAIN)) { return false; }
    require(written == static_cast<long>(encoded.size()), "Write actual variable legacy SysEx");
    return true;
  }

  void legacy_event(snd_seq_event event) const {
    event.queue = SNDRV_SEQ_QUEUE_DIRECT;
    event.source = port_;
    event.dest.client = SNDRV_SEQ_ADDRESS_SUBSCRIBERS;
    require(::write(fd_.value, &event, sizeof(event)) == static_cast<long>(sizeof(event)),
            "Write an independent typed legacy MIDI event");
  }

  void remove() const {
    snd_seq_port_info port{};
    port.addr = port_;
    control(fd_.value, SNDRV_SEQ_IOCTL_DELETE_PORT, &port);
  }

private:
  Descriptor fd_;
  int client_ = -1;
  snd_seq_addr port_{};
  bool ump_ = false;
};

std::uint32_t
voice(const unsigned status, const unsigned pitch, const unsigned value, const unsigned group = 0) {
  constexpr unsigned type_shift = 28;
  constexpr unsigned group_shift = 24;
  return (2U << type_shift) | (group << group_shift) | (status << 16) | (pitch << 8) | value;
}

Packet await(Input &input) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  for (;;) {
    if (const auto packet = input.read()) { return *packet; }
    require(std::chrono::steady_clock::now() < deadline, "Native input cannot stall");
    std::this_thread::yield();
  }
}

void delivery_and_clock() {
  const Source source;
  const auto sources = Input::sources();
  require(std::ranges::any_of(
              sources, [&](const auto &s) { return s.id == source.id() && !s.name.empty(); }),
          "Discover the actual isolated source ID and name");
  Input input({.source = source.id()});
  std::uint64_t previous = 0;
  constexpr unsigned rounds = 64;
  constexpr unsigned batch = 32;
  for (unsigned round = 0; round < rounds; ++round) {
    const auto before = Input::now();
    for (unsigned index = 0; index < batch; ++index) {
      require(source.send(std::array{voice(note_on, note, index)}) &&
                  source.send(std::array{voice(note_off, note, index)}),
              "Bounded test sender has queue room");
    }
    const auto after = Input::now();
    for (unsigned index = 0; index < batch; ++index) {
      for (const auto status : {note_on, note_off}) {
        const auto packet = await(input);
        require(packet.timestamp >= previous && packet.timestamp + uncertainty >= before &&
                    packet.timestamp <= after + uncertainty,
                "Native receipt timestamps retain monotonic order within the measured source "
                "delivery interval");
        require(std::ranges::equal(packet.data(),
                                   std::array{static_cast<std::uint8_t>(status),
                                              static_cast<std::uint8_t>(note),
                                              static_cast<std::uint8_t>(index)}),
                "Native input preserves actual channel note data through queue wrap");
        previous = packet.timestamp;
      }
    }
  }
  require(!input.read(), "Input creates no extra notes");
  const auto before = Input::now();
  require(source.send(std::array{voice(note_off, note, 0)}), "Send a delayed-read note");
  const auto sent = Input::now();
  constexpr std::chrono::milliseconds delayed_read{25};
  std::this_thread::sleep_for(delayed_read);
  const auto packet = await(input);
  constexpr std::uint64_t delayed_read_ns = 20000000;
  require(packet.timestamp + uncertainty >= before && packet.timestamp <= sent + uncertainty &&
              Input::now() - packet.timestamp > delayed_read_ns,
          "The native owner timestamps receipt before a deliberately delayed application dequeue");
}

void sysex_and_groups() {
  const Source source;
  Input input({.source = source.id()});
  if (source.universal()) {
    require(source.send(std::array{voice(note_on, note, velocity, 1)}), "Send an unselected group");
  } else {
    try {
      const Input unavailable({.source = source.id(), .group = 1});
      throw std::logic_error("Legacy ALSA must reject unavailable cable groups");
    } catch (const std::runtime_error &error) {
      require(std::string_view(error.what()).contains("group 0"),
              "Report the actual legacy group grant");
    }
  }
  constexpr std::array<std::uint32_t, 2> begin{0x30160102, 0x03040506};
  constexpr std::array<std::uint32_t, 1> realtime{0x10f80000};
  constexpr std::array<std::uint32_t, 2> end{0x30320708, 0};
  require(source.send(begin), "Send native SysEx start");
  require(source.send(realtime), "Send realtime during native SysEx");
  require(source.send(end), "Send native SysEx end");
  constexpr std::array<std::array<std::uint8_t, 7>, 3> expected{
      {{0xf0, 1, 2, 3, 4, 5, 6}, {0xf8}, {7, 8, 0xf7}}};
  constexpr std::array<std::size_t, 3> sizes{7, 1, 3};
  std::vector<std::uint8_t> actual;
  std::vector<std::uint8_t> wanted;
  for (std::size_t index = 0; index < expected.size(); ++index) {
    const auto data = std::span(expected[index]).first(sizes[index]);
    wanted.insert(wanted.end(), data.begin(), data.end());
  }
  while (actual.size() < wanted.size()) {
    const auto packet = await(input);
    actual.insert(actual.end(), packet.data().begin(), packet.data().end());
  }
  require(actual == wanted, "Native SysEx and realtime retain complete byte-stream framing/order");
  require(!input.read(), "The other group is not accidentally routed");
}

void legacy_conversion() {
  const Source source(false);
  Input input({.source = source.id()});
  source.legacy_note();
  require(
      std::ranges::equal(await(input).data(), std::array<std::uint8_t, 3>{note_on, note, velocity}),
      "Kernel native conversion delivers legacy DIN/DAW note events");
  constexpr std::array<std::uint8_t, 11> data{0xf0, 0x7d, 1, 2, 3, 4, 5, 6, 7, 8, 0xf7};
  require(source.legacy_sysex(data), "Native source accepts the bounded legacy SysEx event");
  std::vector<std::uint8_t> actual;
  while (actual.size() < data.size()) {
    const auto packet = await(input);
    actual.insert(actual.end(), packet.data().begin(), packet.data().end());
  }
  require(std::ranges::equal(actual, data) && !input.read(),
          "Actual legacy variable SysEx converts through UMP without adding or losing bytes");
}

void channel_and_system_events() {
  struct Case {
    unsigned type, parameter;
    int value;
    std::array<std::uint8_t, 12> bytes;
    std::size_t size;
  };

  constexpr std::array cases{
      Case{.type = SNDRV_SEQ_EVENT_CONTROLLER,
           .parameter = 74,
           .value = 127,
           .bytes = {0xb3, 74, 127},
           .size = 3},
      Case{.type = SNDRV_SEQ_EVENT_PGMCHANGE,
           .parameter = 0,
           .value = 127,
           .bytes = {0xc3, 127},
           .size = 2},
      Case{.type = SNDRV_SEQ_EVENT_CHANPRESS,
           .parameter = 0,
           .value = 64,
           .bytes = {0xd3, 64},
           .size = 2},
      Case{.type = SNDRV_SEQ_EVENT_PITCHBEND,
           .parameter = 0,
           .value = -8192,
           .bytes = {0xe3, 0, 0},
           .size = 3},
      Case{.type = SNDRV_SEQ_EVENT_PITCHBEND,
           .parameter = 0,
           .value = 8191,
           .bytes = {0xe3, 127, 127},
           .size = 3},
      Case{.type = SNDRV_SEQ_EVENT_SONGPOS,
           .parameter = 0,
           .value = 257,
           .bytes = {0xf2, 1, 2},
           .size = 3},
      Case{.type = SNDRV_SEQ_EVENT_SONGSEL,
           .parameter = 0,
           .value = 127,
           .bytes = {0xf3, 127},
           .size = 2},
      Case{.type = SNDRV_SEQ_EVENT_QFRAME,
           .parameter = 0,
           .value = 127,
           .bytes = {0xf1, 127},
           .size = 2},
      Case{.type = SNDRV_SEQ_EVENT_CONTROL14,
           .parameter = 1,
           .value = 257,
           .bytes = {0xb3, 1, 2, 0xb3, 33, 1},
           .size = 6},
      Case{.type = SNDRV_SEQ_EVENT_REGPARAM,
           .parameter = 257,
           .value = 259,
           .bytes = {0xb3, 101, 2, 0xb3, 100, 1, 0xb3, 6, 2, 0xb3, 38, 3},
           .size = 12},
      Case{.type = SNDRV_SEQ_EVENT_NONREGPARAM,
           .parameter = 257,
           .value = 259,
           .bytes = {0xb3, 99, 2, 0xb3, 98, 1, 0xb3, 6, 2, 0xb3, 38, 3},
           .size = 12},
      Case{.type = SNDRV_SEQ_EVENT_START, .parameter = 0, .value = 0, .bytes = {0xfa}, .size = 1},
      Case{
          .type = SNDRV_SEQ_EVENT_CONTINUE, .parameter = 0, .value = 0, .bytes = {0xfb}, .size = 1},
      Case{.type = SNDRV_SEQ_EVENT_STOP, .parameter = 0, .value = 0, .bytes = {0xfc}, .size = 1},
      Case{.type = SNDRV_SEQ_EVENT_TUNE_REQUEST,
           .parameter = 0,
           .value = 0,
           .bytes = {0xf6},
           .size = 1},
      Case{.type = SNDRV_SEQ_EVENT_SENSING, .parameter = 0, .value = 0, .bytes = {0xfe}, .size = 1},
      Case{.type = SNDRV_SEQ_EVENT_RESET, .parameter = 0, .value = 0, .bytes = {0xff}, .size = 1}};
  const Source source(false);
  Input input({.source = source.id()});
  snd_seq_event pressure{};
  pressure.type = SNDRV_SEQ_EVENT_KEYPRESS;
  pressure.data.note.channel = 3;
  pressure.data.note.note = note;
  constexpr std::uint8_t pressure_value = 75;
  pressure.data.note.velocity = pressure_value;
  source.legacy_event(pressure);
  constexpr std::array<std::uint8_t, 3> expected_pressure{0xa3, note, pressure_value};
  require(std::ranges::equal(await(input).data(), expected_pressure),
          "Native polyphonic pressure retains its distinct note/pressure operands");
  for (const auto &item : cases) {
    snd_seq_event event{};
    event.type = static_cast<snd_seq_event_type_t>(item.type);
    event.data.control.channel = 3;
    event.data.control.param = item.parameter;
    event.data.control.value = item.value;
    source.legacy_event(event);
    std::vector<std::uint8_t> actual;
    while (actual.size() < item.size) {
      const auto packet = await(input);
      actual.insert(actual.end(), packet.data().begin(), packet.data().end());
    }
    require(std::ranges::equal(actual, std::span(item.bytes).first(item.size)) && !input.read(),
            "Independent typed legacy channel/system input has exact MIDI wire bytes");
  }
}

void expect_failure(const Input &input, const std::string_view reason) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  for (;;) {
    try {
      input.check();
    } catch (const std::runtime_error &error) {
      require(std::string_view(error.what()).contains(reason),
              "Report the actual persistent native failure");
      return;
    }
    require(std::chrono::steady_clock::now() < deadline,
            "The native input owner must report its fault: " + std::string(reason));
    std::this_thread::yield();
  }
}

void faults() {
  {
    const Source source;
    const Input input({.source = source.id()});
    source.remove();
    expect_failure(input, "disconnected");
    expect_failure(input, "disconnected");
  }
  {
    const Source source(false);
    const Input input({.source = source.id()});
    constexpr std::size_t oversized = 65536;
    std::vector<std::uint8_t> data(oversized, 1);
    data.front() = exclusive_begin;
    data.back() = exclusive_end;
    require(!source.legacy_sysex(data),
            "An atomic oversized native event exceeds the granted kernel pool");
    expect_failure(input, "kernel input overflow");
    expect_failure(input, "kernel input overflow");
  }
  {
    const Source source;
    const Input input({.source = source.id()});
    constexpr unsigned batches = 16;
    constexpr unsigned events = 64;
    for (unsigned batch = 0; batch < batches; ++batch) {
      for (unsigned index = 0; index < events; ++index) {
        require(source.send(std::array{voice(note_on, note, velocity)}),
                "Fill one bounded input batch");
      }
      constexpr std::chrono::milliseconds owner_delivery{1};
      std::this_thread::sleep_for(owner_delivery);
      input.check();
    }
    require(source.send(std::array{voice(note_off, note, 0)}),
            "Kernel has room after explicit batch pumping");
    expect_failure(input, "input queue overflow");
  }
  {
    const Source source(false);
    const Input input({.source = source.id()});
    constexpr std::uint8_t invalid_pitch = 128;
    source.legacy_note(invalid_pitch);
    expect_failure(input, "range");
    expect_failure(input, "range");
  }
}
}

int main() try {
  delivery_and_clock();
  sysex_and_groups();
  legacy_conversion();
  channel_and_system_events();
  faults();
  std::cout << "Native ALSA input, timestamp, queue, SysEx and disconnect checks passed\n";
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
