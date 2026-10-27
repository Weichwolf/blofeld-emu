// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "midi/host_midi_port.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <fcntl.h>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <sound/asequencer.h>
#include <span>
#include <string>
#include <string_view>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/types.h>
#include <unistd.h>
#include <utility>
#include <vector>

namespace {
using blofeld::HostMidiAddress;
using blofeld::HostMidiPort;
constexpr int client_limit = 253, timeout_ms = 1000, skipped = 77;
constexpr std::uint8_t incoming_note = 62, incoming_velocity = 99, outgoing_controller = 74;
constexpr std::uint8_t outgoing_value = 111, sysex_begin = 0xf0, sysex_end = 0xf7,
                       timing_clock = 0xf8;
constexpr unsigned large_payload = 8193, overflow_note = 60;
constexpr std::size_t read_fragment = 113;

HostMidiAddress active_address(const HostMidiPort &port) {
  const auto address = port.address();
  if (!address) { throw blofeld::Fault("Expected a live host MIDI address"); }
  return *address;
}

void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const blofeld::Fault &error) { return error.what(); }
  throw blofeld::Fault("Expected a directed host transport refusal");
}

std::size_t descriptors() {
  return static_cast<std::size_t>(std::distance(
      std::filesystem::directory_iterator("/proc/self/fd"), std::filesystem::directory_iterator{}));
}

struct Message {
  snd_seq_event event{};
  std::vector<std::uint8_t> data;
};

class Fixture {
public:
  Fixture() {
    fd = ::open("/dev/snd/seq", O_RDWR | O_NONBLOCK | O_CLOEXEC);
    require(fd >= 0, "A real local ALSA sequencer is required; do not silently skip");
    try {
      int client = -1;
      require(::ioctl(fd, SNDRV_SEQ_IOCTL_CLIENT_ID, &client) == 0 && client >= 0 &&
                  client < client_limit,
              "Own independent fixture client is created");
      snd_seq_port_info port{};
      port.addr.client = static_cast<std::uint8_t>(client);
      port.capability = SNDRV_SEQ_PORT_CAP_READ | SNDRV_SEQ_PORT_CAP_WRITE |
                        SNDRV_SEQ_PORT_CAP_SUBS_READ | SNDRV_SEQ_PORT_CAP_SUBS_WRITE;
      port.type = SNDRV_SEQ_PORT_TYPE_MIDI_GENERIC | SNDRV_SEQ_PORT_TYPE_APPLICATION;
      require(::ioctl(fd, SNDRV_SEQ_IOCTL_CREATE_PORT, &port) == 0, "Own fixture port is created");
      address = {.client = static_cast<unsigned>(client), .port = port.addr.port};
    } catch (...) {
      ::close(fd);
      throw;
    }
  }

  ~Fixture() { ::close(fd); }

  Fixture(const Fixture &) = delete;
  Fixture &operator=(const Fixture &) = delete;

  void connect(const HostMidiAddress source, const HostMidiAddress destination) const {
    snd_seq_port_subscribe route{};
    route.sender = {.client = static_cast<std::uint8_t>(source.client),
                    .port = static_cast<std::uint8_t>(source.port)};
    route.dest = {.client = static_cast<std::uint8_t>(destination.client),
                  .port = static_cast<std::uint8_t>(destination.port)};
    require(::ioctl(fd, SNDRV_SEQ_IOCTL_SUBSCRIBE_PORT, &route) == 0,
            "Only explicit test-owned virtual ports are connected");
  }

  [[nodiscard]] bool try_send(snd_seq_event event,
                              const HostMidiAddress target,
                              const std::span<const std::uint8_t> payload = {}) const {
    event.source = {.client = static_cast<std::uint8_t>(address.client),
                    .port = static_cast<std::uint8_t>(address.port)};
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
    const auto written = ::write(fd, record.data(), record.size());
    if (written < 0 && errno == EAGAIN) { return false; }
    if (written != static_cast<ssize_t>(record.size())) {
      throw blofeld::Fault("Independent UAPI send type=" + std::to_string(event.type) +
                           " payload=" + std::to_string(payload.size()) + " result=" +
                           std::to_string(written) + " errno=" + std::to_string(errno));
    }
    return true;
  }

  void send(const snd_seq_event event,
            const HostMidiAddress target,
            const std::span<const std::uint8_t> payload = {}) const {
    require(try_send(event, target, payload), "Unexpected backpressure in directed MIDI delivery");
  }

  [[nodiscard]] std::vector<Message> receive() const {
    pollfd pending{.fd = fd, .events = POLLIN, .revents = 0};
    require(::poll(&pending, 1, timeout_ms) > 0,
            "Local virtual delivery completes within a bounded wait");
    std::array<std::uint8_t, sizeof(snd_seq_event) * 1024> bytes{};
    const auto count = ::read(fd, bytes.data(), bytes.size());
    require(count > 0, "Independent kernel event read succeeds");
    const auto size = static_cast<std::size_t>(count);
    std::vector<Message> messages;
    std::size_t offset = 0;
    while (offset < size) {
      require(sizeof(snd_seq_event) <= size - offset, "Kernel event header is complete");
      Message message;
      std::memcpy(&message.event, bytes.data() + offset, sizeof(snd_seq_event));
      offset += sizeof(snd_seq_event);
      if ((message.event.flags & SNDRV_SEQ_EVENT_LENGTH_MASK) == SNDRV_SEQ_EVENT_LENGTH_VARIABLE) {
        const auto length = static_cast<std::size_t>(message.event.data.ext.len);
        const auto padded =
            ((length + sizeof(snd_seq_event) - 1) / sizeof(snd_seq_event)) * sizeof(snd_seq_event);
        require(padded <= size - offset, "Actual kernel variable-record alignment is retained");
        const auto payload = std::span{bytes}.subspan(offset, length);
        message.data.assign(payload.begin(), payload.end());
        offset += padded;
      }
      if (message.event.type != SNDRV_SEQ_EVENT_PORT_SUBSCRIBED &&
          message.event.type != SNDRV_SEQ_EVENT_PORT_UNSUBSCRIBED) {
        messages.push_back(std::move(message));
      }
    }
    return messages;
  }

  [[nodiscard]] snd_seq_client_pool pool(const HostMidiAddress target) const {
    snd_seq_client_pool info{};
    info.client = static_cast<int>(target.client);
    require(::ioctl(fd, SNDRV_SEQ_IOCTL_GET_CLIENT_POOL, &info) == 0,
            "Independent kernel API observes the actual destination pool");
    return info;
  }

  [[nodiscard]] bool exists(const HostMidiAddress target) const {
    snd_seq_port_info port{};
    port.addr = {.client = static_cast<std::uint8_t>(target.client),
                 .port = static_cast<std::uint8_t>(target.port)};
    if (::ioctl(fd, SNDRV_SEQ_IOCTL_GET_PORT_INFO, &port) == 0) { return true; }
    if (errno == ENOENT) { return false; }
    require(errno == ENXIO, "An absent client has the documented ENXIO result");
    snd_seq_client_info client{};
    client.client = static_cast<int>(target.client);
    require(::ioctl(fd, SNDRV_SEQ_IOCTL_GET_CLIENT_INFO, &client) < 0 && errno == ENOENT,
            "The port lookup refers to an actually removed client, not another ioctl failure");
    return false;
  }

  int fd = -1;
  HostMidiAddress address;
};

void test_admission() {
  const auto before = descriptors();
  const std::string embedded("bad\0name", 8);
  for (const auto &name :
       std::array<std::string, 3>{std::string{}, std::string(64, 'x'), embedded}) {
    require(rejects([&] { const HostMidiPort invalid(name); }).contains("non-NUL"),
            "Names reject before acquiring an external session");
  }
  HostMidiPort valid("Blofeld emulator local bounds test");
  const auto address = active_address(valid);
  std::array<std::uint8_t, HostMidiPort::maximum_transfer + 1> large{};
  require(rejects([&] { static_cast<void>(valid.read(large)); }).contains("bounded") &&
              rejects([&] { valid.write(large); }).contains("bounded") &&
              valid.address() == address && !valid.faulted(),
          "Caller span refusals do not consume input, emit output or retire the live session");
  valid.retire();
  require(descriptors() == before,
          "Both failed admission and normal retirement close every handle");
}

void test_native_exchange() {
  const Fixture fixture;
  HostMidiPort port("Blofeld emulator local MIDI test");
  const auto address = active_address(port);
  require(fixture.exists(address), "The ordinary kernel API sees the exported application port");
  const auto pool = fixture.pool(address);
  const auto limits = port.limits();
  if (!limits) { throw blofeld::Fault("A live host session must report its actual grants"); }
  require(limits->input_events == static_cast<std::size_t>(pool.input_pool) &&
              limits->output_events == static_cast<std::size_t>(pool.output_pool) &&
              pool.output_room == 1,
          "Public negotiated limits match independent current kernel pool readback");
  std::cout << "granted input-events=" << limits->input_events
            << " output-events=" << limits->output_events << '\n';
  snd_seq_event source{};
  source.type = SNDRV_SEQ_EVENT_NOTEON;
  source.data.note.channel = 5;
  source.data.note.note = incoming_note;
  source.data.note.velocity = incoming_velocity;
  fixture.send(source, address);
  require(port.read({}) == 0, "An empty destination leaves genuine pending input intact");
  std::array<std::uint8_t, 1> byte{};
  const std::array<std::uint8_t, 3> expected{0x95, 62, 99};
  for (const auto value : expected) {
    require(port.read(byte) == 1 && byte[0] == value,
            "Independent kernel NOTEON becomes exact bytes through one-byte downstream admission");
  }
  require(port.read(byte) == 0, "No byte is padded or repeated after kernel input exhaustion");
  fixture.connect(address, fixture.address);
  const std::array<std::uint8_t, 6> output{0xb4, 74, 111, 0xe2, 1, 64};
  port.write(output);
  std::vector<Message> messages;
  for (unsigned attempt = 0; messages.size() < 2 && attempt < 4; ++attempt) {
    auto batch = fixture.receive();
    for (auto &message : batch) { messages.push_back(std::move(message)); }
  }
  require(messages.size() == 2 && messages[0].event.type == SNDRV_SEQ_EVENT_CONTROLLER &&
              messages[0].event.data.control.channel == 4 &&
              messages[0].event.data.control.param == outgoing_controller &&
              messages[0].event.data.control.value == outgoing_value &&
              messages[1].event.type == SNDRV_SEQ_EVENT_PITCHBEND &&
              messages[1].event.data.control.channel == 2 &&
              messages[1].event.data.control.value == 1,
          "Independent kernel records retain the literal controller and pitch-bend expectations");
  port.retire();
  require(!port.address() && !port.limits() && !fixture.exists(address) && !port.faulted(),
          "Normal retirement removes the actual exported port");
  require(rejects([&] { port.write(output); }).contains("new active session"),
          "A retired endpoint cannot silently reconnect or repeat host output");
}

void test_sysex_exchange() {
  const Fixture fixture;
  HostMidiPort port("Blofeld emulator local SysEx test");
  const auto address = active_address(port);
  std::vector<std::uint8_t> sysex{sysex_begin};
  for (unsigned n = 0; n < large_payload; ++n) {
    sysex.push_back(static_cast<std::uint8_t>(n % 128));
  }
  sysex.push_back(sysex_end);
  snd_seq_event source{};
  source.type = SNDRV_SEQ_EVENT_SYSEX;
  const auto sender = fixture.pool(fixture.address);
  const auto receiver = fixture.pool(address);
  const auto cells = std::min(sender.output_pool, receiver.input_pool);
  require(cells >= 2, "Actual pools can hold a variable event plus payload");
  // Test-side legacy ABI cell accounting determines whether a single large
  // event is possible. Smaller hosts still transfer the entire logical SysEx.
  const auto capacity = static_cast<std::size_t>(cells - 1) * sizeof(snd_seq_event);
  std::cout << "SysEx sender-events=" << sender.output_pool
            << " receiver-events=" << receiver.input_pool << " event-capacity=" << capacity << '\n';
  std::vector<std::uint8_t> observed;
  std::array<std::uint8_t, read_fragment> buffer{};
  std::size_t sent = 0;
  while (sent < sysex.size()) {
    const auto count = std::min(capacity, sysex.size() - sent);
    fixture.send(source, address, std::span{sysex}.subspan(sent, count));
    sent += count;
    while (observed.size() < sent) {
      const auto size = port.read(buffer);
      require(size != 0, "A synchronously queued event supplies the remaining logical bytes");
      const auto chunk = std::span{buffer}.first(size);
      observed.insert(observed.end(), chunk.begin(), chunk.end());
    }
  }
  require(observed == sysex,
          "An actual long kernel SysEx event survives bounded downstream fragments");
  fixture.connect(address, fixture.address);
  constexpr std::array<std::uint8_t, 9> reply{0xf0, 0x7d, 1, 2, 0xf8, 3, 4, 5, 0xf7};
  for (const auto value : reply) { port.write(std::span{&value, 1}); }
  observed.clear();
  for (unsigned attempt = 0; observed.size() < reply.size() && attempt < 4; ++attempt) {
    for (const auto &message : fixture.receive()) {
      if (message.event.type == SNDRV_SEQ_EVENT_SYSEX) {
        observed.insert(observed.end(), message.data.begin(), message.data.end());
      } else {
        require(message.event.type == SNDRV_SEQ_EVENT_CLOCK,
                "Only the actual realtime event interleaves");
        observed.push_back(timing_clock);
      }
    }
  }
  require(std::ranges::equal(observed, reply),
          "Kernel delivery preserves partial SysEx/realtime order");
}

void test_failure_sessions() {
  const Fixture fixture;
  HostMidiPort port("Blofeld emulator local refusal test");
  const auto address = active_address(port);
  snd_seq_event event{};
  event.type = SNDRV_SEQ_EVENT_USR0;
  fixture.send(event, address);
  std::array<std::uint8_t, 8> data{};
  require(rejects([&] { static_cast<void>(port.read(data)); }).contains("Unsupported host MIDI"),
          "A valid non-MIDI user event refuses without pretending it is a device message");
  require(port.faulted() && !port.address() && !fixture.exists(address),
          "A transport failure retires its actual kernel endpoint");
  HostMidiPort output("Blofeld emulator local output refusal");
  constexpr std::array<std::uint8_t, 1> undefined{0xf5};
  require(!rejects([&] { output.write(undefined); }).empty() && output.faulted() &&
              !output.address(),
          "Undefined firmware output bytes fail visibly and retire the host session");
}

void test_overrun() {
  const Fixture fixture;
  HostMidiPort port("Blofeld emulator local overrun test");
  snd_seq_event event{};
  event.type = SNDRV_SEQ_EVENT_NOTEON;
  event.data.note.note = overflow_note;
  event.data.note.velocity = 1;
  const auto address = active_address(port);
  const auto granted = fixture.pool(address);
  bool backpressure = false;
  for (int n = 0; n <= granted.input_pool; ++n) {
    if (!fixture.try_send(event, address)) {
      backpressure = true;
      break;
    }
  }
  require(backpressure, "A full actual receiver pool reports directed sender backpressure");
  std::array<std::uint8_t, 3> bytes{};
  require(
      rejects([&] { static_cast<void>(port.read(bytes)); }).contains("FIFO overflow") &&
          port.faulted() && !port.address(),
      "Actual kernel input-pool overflow is visible and cannot continue as a successful stream");
}

void discovery_command(const std::string_view expected) {
  std::string command;
  require(static_cast<bool>(std::getline(std::cin, command)) && command == expected,
          "The discovery owner must request the actual transport operation");
}

int discovery() {
  HostMidiPort source("Blofeld emulator discovery source");
  HostMidiPort destination("Blofeld emulator discovery destination");
  const auto from = active_address(source);
  const auto to = active_address(destination);
  std::cout << "MIDI source=" << from.client << ':' << from.port << " destination=" << to.client
            << ':' << to.port << '\n'
            << std::flush;
  discovery_command("exchange");
  constexpr std::array<std::uint8_t, 3> wire{0x92, 61, 91};
  source.write(wire);
  std::array<std::uint8_t, 3> observed{};
  require(destination.read(observed) == wire.size() && observed == wire,
          "Ordinary aconnect routing delivers the literal note bytes between owned virtual ports");
  std::cout << "exchange passed\n" << std::flush;
  discovery_command("retire");
  source.retire();
  destination.retire();
  require(!source.address() && !destination.address(),
          "Discovery retirement closes both host ports");
  std::cout << "retired\n";
  return 0;
}
}

int main(const int count, char **arguments) {
  try {
    const auto probe = ::open("/dev/snd/seq", O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (probe < 0) {
      if (errno == ENOENT || errno == ENXIO || errno == EACCES || errno == EPERM) {
        std::cout << "SKIP: local ALSA sequencer unavailable, errno=" << errno << '\n';
        return skipped;
      }
      throw blofeld::Fault("Unexpected ALSA sequencer availability failure");
    }
    ::close(probe);
    if (count == 2 && std::string_view(arguments[1]) == "--discovery") { return discovery(); }
    require(count == 1, "Unexpected private test argument");
    const auto before = descriptors();
    std::cout << "admission\n" << std::flush;
    test_admission();
    std::cout << "exchange\n" << std::flush;
    test_native_exchange();
    std::cout << "sysex\n" << std::flush;
    test_sysex_exchange();
    std::cout << "failure sessions\n" << std::flush;
    test_failure_sessions();
    std::cout << "overrun\n" << std::flush;
    test_overrun();
    require(descriptors() == before,
            "Every local port/client and rejected session closes its descriptor");
    std::cout << "Actual local ALSA MIDI-1 port, fragmented SysEx, failure/overrun and retirement "
                 "passed; no physical traffic\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
