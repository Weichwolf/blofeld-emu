// SPDX-License-Identifier: GPL-3.0-only
#include "midi/input.hpp"
#include "midi/ump.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <bits/time.h>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <fcntl.h>
#include <limits>
#include <memory>
#include <optional>
#include <sound/asequencer.h>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <string_view>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/types.h>
#include <system_error>
#include <thread>
#include <unistd.h>
#include <vector>

extern "C" {
#include <time.h>
}

namespace blofeld::host::midi {
namespace {
constexpr std::size_t capacity = 1024, batch_size = 64, receive_bytes = 65536;
constexpr std::uint64_t second = 1000000000;
static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
constexpr unsigned source_capabilities = SNDRV_SEQ_PORT_CAP_READ | SNDRV_SEQ_PORT_CAP_SUBS_READ;
constexpr unsigned source_types = SNDRV_SEQ_PORT_TYPE_MIDI_GENERIC | SNDRV_SEQ_PORT_TYPE_MIDI_UMP;
constexpr unsigned minimum_protocol = (1U << 16) | 3;

enum Midi : std::uint8_t {
  data_mask = 127,
  byte_mask = 255,
  channel_mask = 15,
  note_off = 0x80,
  note_on = 0x90,
  key_pressure = 0xa0,
  controller = 0xb0,
  program_change = 0xc0,
  channel_pressure = 0xd0,
  pitch_bend = 0xe0,
  sysex_begin = 0xf0,
  time_code = 0xf1,
  song_position = 0xf2,
  song_select = 0xf3,
  tune_request = 0xf6,
  sysex_end = 0xf7,
  realtime_clock = 0xf8,
  midi_start = 0xfa,
  midi_continue = 0xfb,
  midi_stop = 0xfc,
  active_sensing = 0xfe,
  midi_reset = 0xff
};

void control(const int descriptor, const unsigned long request, void *value, const char *name) {
  int result = 0;
  do { result = ::ioctl(descriptor, request, value); } while (result < 0 && errno == EINTR);
  if (result < 0) { throw std::system_error(errno, std::generic_category(), name); }
}

class Sequencer {
public:
  Sequencer() : descriptor_(::open("/dev/snd/seq", O_RDWR | O_CLOEXEC | O_NONBLOCK)) {
    if (descriptor_ < 0) {
      throw std::system_error(errno, std::generic_category(), "Open native ALSA sequencer");
    }
  }

  ~Sequencer() { ::close(descriptor_); }

  Sequencer(const Sequencer &) = delete;
  Sequencer &operator=(const Sequencer &) = delete;

  [[nodiscard]] int descriptor() const noexcept { return descriptor_; }

  void query(const unsigned long request, void *value, const char *name) const {
    control(descriptor_, request, value, name);
  }

private:
  int descriptor_;
};

class Wakeup {
public:
  Wakeup() : descriptor_(::eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK)) {
    if (descriptor_ < 0) {
      throw std::system_error(errno, std::generic_category(), "Create MIDI owner wakeup");
    }
  }

  ~Wakeup() { ::close(descriptor_); }

  Wakeup(const Wakeup &) = delete;
  Wakeup &operator=(const Wakeup &) = delete;

  [[nodiscard]] int descriptor() const noexcept { return descriptor_; }

  void signal() const noexcept {
    const std::uint64_t value = 1;
    ssize_t result = 0;
    do {
      result = ::write(descriptor_, &value, sizeof(value));
    } while (result < 0 && errno == EINTR);
  }

private:
  int descriptor_;
};

std::uint64_t nanoseconds(const std::uint64_t seconds, const std::uint64_t fraction) {
  if (fraction >= second ||
      seconds > (std::numeric_limits<std::uint64_t>::max() - fraction) / second) {
    throw std::overflow_error("Native MIDI clock exceeds nanosecond range");
  }
  return (seconds * second) + fraction;
}

bool selectable(const snd_seq_port_info &port) {
  return (port.capability & source_capabilities) == source_capabilities &&
         (port.capability & (SNDRV_SEQ_PORT_CAP_NO_EXPORT | SNDRV_SEQ_PORT_CAP_INACTIVE)) == 0 &&
         (port.type & source_types) != 0;
}

snd_seq_addr address(const std::int32_t source) {
  constexpr unsigned address_mask = 255;
  constexpr std::int32_t maximum = 65535;
  if (source < 0 || source > maximum) { throw std::invalid_argument("Invalid ALSA MIDI source"); }
  return {.client = static_cast<unsigned char>(static_cast<unsigned>(source) >> 8),
          .port = static_cast<unsigned char>(static_cast<unsigned>(source) & address_mask)};
}

bool same(const snd_seq_addr a, const snd_seq_addr b) {
  return a.client == b.client && a.port == b.port;
}
}

struct Input::Impl {
  const Route route;
  Sequencer seq;
  Ump decoder;
  snd_seq_addr source{}, port{};
  int client = -1;
  Wakeup wakeup;
  std::array<Packet, capacity> packets{};
  std::array<std::uint8_t, receive_bytes> buffer{};
  bool ump = false;
  std::atomic<std::uint64_t> head{0}, tail{0};
  std::atomic<bool> started{false}, failed{false};
  std::uint64_t previous = 0;
  std::exception_ptr failure;
  std::jthread worker;

  explicit Impl(const Route selected)
      : route(selected), decoder(selected.group), source(address(selected.source)) {
    configure_client();
    snd_seq_port_info peer{};
    peer.addr = source;
    seq.query(SNDRV_SEQ_IOCTL_GET_PORT_INFO, &peer, "Find selected ALSA MIDI source");
    if (!selectable(peer)) {
      throw std::runtime_error("Selected ALSA source cannot be subscribed");
    }
    configure_port();
    subscribe({.client = SNDRV_SEQ_CLIENT_SYSTEM, .port = SNDRV_SEQ_PORT_SYSTEM_ANNOUNCE});
    subscribe(source);
    connection();
    worker = std::jthread([this](const std::stop_token &stop) {
      started.store(true, std::memory_order_release);
      started.notify_one();
      try {
        run(stop);
      } catch (...) {
        failure = std::current_exception();
        failed.store(true, std::memory_order_release);
      }
    });
    started.wait(false, std::memory_order_acquire);
  }

  ~Impl() {
    worker.request_stop();
    wakeup.signal();
  }

  void configure_client() {
    int protocol = 0;
    seq.query(SNDRV_SEQ_IOCTL_PVERSION, &protocol, "Query ALSA sequencer protocol");
    seq.query(SNDRV_SEQ_IOCTL_CLIENT_ID, &client, "Query ALSA MIDI client ID");
    if (client < 0 || client > std::numeric_limits<unsigned char>::max()) {
      throw std::runtime_error("Invalid native MIDI client ID grant");
    }
    snd_seq_client_info info{};
    info.client = client;
    seq.query(SNDRV_SEQ_IOCTL_GET_CLIENT_INFO, &info, "Query native MIDI client");
    constexpr std::string_view name = "Virtual instrument MIDI";
    std::ranges::copy(name, info.name);
    info.name[name.size()] = '\0';
    info.midi_version = protocol >= static_cast<int>(minimum_protocol)
                            ? SNDRV_SEQ_CLIENT_UMP_MIDI_1_0
                            : SNDRV_SEQ_CLIENT_LEGACY_MIDI;
    seq.query(SNDRV_SEQ_IOCTL_SET_CLIENT_INFO, &info, "Select native MIDI 1.0 UMP input");
    seq.query(SNDRV_SEQ_IOCTL_GET_CLIENT_INFO, &info, "Read granted MIDI protocol");
    ump = info.midi_version == SNDRV_SEQ_CLIENT_UMP_MIDI_1_0;
    if (!ump && (info.midi_version != SNDRV_SEQ_CLIENT_LEGACY_MIDI || route.group != 0)) {
      throw std::runtime_error("Legacy native ALSA input exposes MIDI 1.0 group 0 only");
    }
    snd_seq_client_pool pool{};
    pool.client = client;
    pool.input_pool = static_cast<int>(capacity);
    seq.query(SNDRV_SEQ_IOCTL_SET_CLIENT_POOL, &pool, "Set bounded native MIDI input pool");
    seq.query(SNDRV_SEQ_IOCTL_GET_CLIENT_POOL, &pool, "Read granted native MIDI input pool");
    if (pool.input_pool <= 0 || pool.input_pool > static_cast<int>(capacity)) {
      throw std::runtime_error("Native MIDI input pool exceeds its requested bound");
    }
  }

  void configure_port() {
    snd_seq_port_info info{};
    info.addr.client = static_cast<unsigned char>(client);
    constexpr std::string_view name = "MIDI input";
    std::ranges::copy(name, info.name);
    info.name[name.size()] = '\0';
    info.capability =
        SNDRV_SEQ_PORT_CAP_WRITE | SNDRV_SEQ_PORT_CAP_SUBS_WRITE | SNDRV_SEQ_PORT_CAP_NO_EXPORT;
    info.type = SNDRV_SEQ_PORT_TYPE_APPLICATION | SNDRV_SEQ_PORT_TYPE_MIDI_GENERIC;
    seq.query(SNDRV_SEQ_IOCTL_CREATE_PORT, &info, "Create timestamped native MIDI input port");
    port = info.addr;
  }

  void subscribe(const snd_seq_addr sender) const {
    snd_seq_port_subscribe subscription{};
    subscription.sender = sender;
    subscription.dest = port;
    seq.query(SNDRV_SEQ_IOCTL_SUBSCRIBE_PORT, &subscription, "Subscribe native MIDI input");
  }

  void connection() const {
    snd_seq_port_subscribe subscription{};
    subscription.sender = source;
    subscription.dest = port;
    seq.query(SNDRV_SEQ_IOCTL_GET_SUBSCRIPTION, &subscription, "Selected MIDI source disconnected");
  }

  void publish(const std::span<const std::uint32_t> words, const std::uint64_t time) {
    const auto packet = decoder.decode(words, time);
    if (decoder.error() != nullptr) { throw std::runtime_error(decoder.error()); }
    if (!packet || packet->size == 0) { return; }
    if (packet->timestamp == 0 || packet->timestamp < previous) {
      throw std::runtime_error("Native MIDI timestamp is missing or moved backwards");
    }
    const auto at = head.load(std::memory_order_relaxed);
    if (at == std::numeric_limits<std::uint64_t>::max() ||
        at - tail.load(std::memory_order_acquire) == capacity) {
      throw std::runtime_error("Native MIDI input queue overflow");
    }
    packets[at % capacity] = *packet;
    head.store(at + 1, std::memory_order_release);
    previous = packet->timestamp;
  }

  void
  short_message(const unsigned status, const std::array<int, 2> data, const std::uint64_t time) {
    constexpr unsigned maximum = data_mask;
    constexpr unsigned type_shift = 28;
    const auto a = data[0];
    const auto b = data[1];
    if (status > byte_mask || a < 0 || a > static_cast<int>(maximum) || b < 0 ||
        b > static_cast<int>(maximum)) {
      throw std::runtime_error("Invalid native ALSA MIDI status/data range");
    }
    const unsigned type = status < sysex_begin ? 2U : 1U;
    const std::array words{(type << type_shift) | (status << 16) | (static_cast<unsigned>(a) << 8) |
                           static_cast<unsigned>(b)};
    publish(words, time);
  }

  void control_change(const snd_seq_ev_ctrl &control,
                      const unsigned parameter,
                      const unsigned value,
                      const std::uint64_t time) {
    if (control.channel > 15 || parameter > data_mask || value > data_mask) {
      throw std::runtime_error("Invalid native ALSA MIDI controller range");
    }
    short_message(
        controller | control.channel, {static_cast<int>(parameter), static_cast<int>(value)}, time);
  }

  void
  paired_control(const unsigned type, const snd_seq_event_data &data, const std::uint64_t time) {
    constexpr unsigned wide_maximum = 16383;
    constexpr unsigned msb_shift = 7;
    constexpr unsigned low_mask = data_mask;
    constexpr unsigned wide_controller_maximum = 31;
    const auto &control = data.control;
    if (control.param > wide_maximum || control.value < 0 ||
        control.value > static_cast<int>(wide_maximum)) {
      throw std::runtime_error("Invalid native ALSA MIDI paired-controller range");
    }
    unsigned parameter = control.param;
    if (type == SNDRV_SEQ_EVENT_CONTROL14) {
      if (parameter > wide_controller_maximum) {
        throw std::runtime_error("Invalid native ALSA 14-bit controller number");
      }
    } else {
      const unsigned selector = type == SNDRV_SEQ_EVENT_REGPARAM ? 101 : 99;
      control_change(control, selector, parameter >> msb_shift, time);
      control_change(control, selector - 1, parameter & low_mask, time);
      parameter = 6;
    }
    const auto value = static_cast<unsigned>(control.value);
    control_change(control, parameter, value >> msb_shift, time);
    control_change(control, parameter + 32, value & low_mask, time);
  }

  void
  legacy_message(const snd_seq_event_data &data, const unsigned type, const std::uint64_t time) {
    if (type >= SNDRV_SEQ_EVENT_NOTEON && type <= SNDRV_SEQ_EVENT_REGPARAM &&
        data.note.channel > 15) {
      throw std::runtime_error("Invalid native ALSA MIDI channel");
    }
    const auto channel = data.note.channel;
    switch (type) {
      case SNDRV_SEQ_EVENT_NOTEON:
        short_message(note_on | channel, {data.note.note, data.note.velocity}, time);
        break;
      case SNDRV_SEQ_EVENT_NOTEOFF:
        short_message(note_off | channel, {data.note.note, data.note.velocity}, time);
        break;
      case SNDRV_SEQ_EVENT_KEYPRESS:
        short_message(key_pressure | channel, {data.note.note, data.note.velocity}, time);
        break;
      case SNDRV_SEQ_EVENT_CONTROLLER:
        control_change(
            data.control, data.control.param, static_cast<unsigned>(data.control.value), time);
        break;
      case SNDRV_SEQ_EVENT_PGMCHANGE:
        short_message(program_change | channel, {data.control.value, 0}, time);
        break;
      case SNDRV_SEQ_EVENT_CHANPRESS:
        short_message(channel_pressure | channel, {data.control.value, 0}, time);
        break;
      case SNDRV_SEQ_EVENT_PITCHBEND:
      case SNDRV_SEQ_EVENT_SONGPOS: {
        constexpr int center = 8192;
        constexpr int maximum = 16383;
        const auto value =
            std::int64_t{data.control.value} + (type == SNDRV_SEQ_EVENT_PITCHBEND ? center : 0);
        if (value < 0 || value > maximum) {
          throw std::runtime_error("Invalid native ALSA MIDI 14-bit value");
        }
        const unsigned status =
            type == SNDRV_SEQ_EVENT_PITCHBEND ? pitch_bend | channel : song_position;
        short_message(
            status, {static_cast<int>(value & data_mask), static_cast<int>(value >> 7)}, time);
        break;
      }
      case SNDRV_SEQ_EVENT_SONGSEL:
        short_message(song_select, {data.control.value, 0}, time);
        break;
      case SNDRV_SEQ_EVENT_QFRAME: short_message(time_code, {data.control.value, 0}, time); break;
      case SNDRV_SEQ_EVENT_START: short_message(midi_start, {0, 0}, time); break;
      case SNDRV_SEQ_EVENT_CONTINUE: short_message(midi_continue, {0, 0}, time); break;
      case SNDRV_SEQ_EVENT_STOP: short_message(midi_stop, {0, 0}, time); break;
      case SNDRV_SEQ_EVENT_CLOCK: short_message(realtime_clock, {0, 0}, time); break;
      case SNDRV_SEQ_EVENT_TUNE_REQUEST: short_message(tune_request, {0, 0}, time); break;
      case SNDRV_SEQ_EVENT_RESET: short_message(midi_reset, {0, 0}, time); break;
      case SNDRV_SEQ_EVENT_SENSING: short_message(active_sensing, {0, 0}, time); break;
      case SNDRV_SEQ_EVENT_CONTROL14:
      case SNDRV_SEQ_EVENT_NONREGPARAM:
      case SNDRV_SEQ_EVENT_REGPARAM: paired_control(type, data, time); break;
      default:
        throw std::runtime_error("Unsupported native ALSA MIDI event type=" + std::to_string(type));
    }
  }

  void legacy_sysex(const std::span<const std::uint8_t> data, const std::uint64_t time) {
    if (data.empty()) { throw std::runtime_error("Empty native ALSA SysEx event"); }
    std::array<std::uint8_t, 6> pending{};
    unsigned used = 0;
    const auto flush = [&](const unsigned kind) {
      constexpr unsigned type_shift = 28;
      constexpr unsigned status_shift = 20;
      std::array<std::uint32_t, 2> words{(3U << type_shift) | (kind << status_shift) | (used << 16),
                                         0};
      for (unsigned index = 0; index < used; ++index) {
        const auto word = index < 2 ? 0U : 1U;
        const auto shift = (index < 2 ? 1 - index : 5 - index) * 8;
        words[word] |= static_cast<std::uint32_t>(pending[index]) << shift;
      }
      publish(words, time);
      used = 0;
    };
    for (const auto byte : data) {
      if (byte < note_off) {
        pending[used++] = byte;
        if (used == pending.size()) { flush(2); }
        continue;
      }
      if (byte != sysex_end && used != 0) { flush(2); }
      switch (byte) {
        case sysex_begin: flush(1); break;
        case sysex_end: flush(3); break;
        default:
          if (byte < realtime_clock) {
            throw std::runtime_error("Invalid native ALSA SysEx status");
          }
          short_message(byte, {0, 0}, time);
      }
    }
    if (used != 0) { flush(2); }
  }

  void receive(const snd_seq_ump_event &event,
               const std::span<const std::uint8_t> payload,
               const std::uint64_t timestamp) {
    if (!same(event.source, source)) {
      if (event.source.client == SNDRV_SEQ_CLIENT_SYSTEM) { return; }
      throw std::runtime_error("Unexpected source on selected native MIDI route");
    }
    if ((event.flags & SNDRV_SEQ_EVENT_UMP) != 0) {
      const auto words = event.ump[0] >> 28U == 3 ? 2U : 1U;
      publish(std::span(event.ump).first(words), timestamp);
    } else if (event.type == SNDRV_SEQ_EVENT_SYSEX) {
      legacy_sysex(payload, timestamp);
    } else {
      legacy_message(event.data, event.type, timestamp);
    }
  }

  void receive_buffer(const std::span<const std::uint8_t> bytes, const std::uint64_t timestamp) {
    const auto size = ump ? sizeof(snd_seq_ump_event) : sizeof(snd_seq_event);
    std::size_t offset = 0;
    while (offset < bytes.size()) {
      if (bytes.size() - offset < size) {
        throw std::runtime_error("Truncated native MIDI event header");
      }
      snd_seq_ump_event event{};
      std::memcpy(&event, bytes.data() + offset, size);
      offset += size;
      std::span<const std::uint8_t> payload;
      if ((event.flags & SNDRV_SEQ_EVENT_LENGTH_MASK) == SNDRV_SEQ_EVENT_LENGTH_VARIABLE) {
        const auto length = event.data.ext.len;
        if (length > bytes.size() - offset) {
          throw std::runtime_error("Truncated native MIDI SysEx payload");
        }
        payload = bytes.subspan(offset, length);
        const auto padded = ((static_cast<std::size_t>(length) + size - 1) / size) * size;
        if (padded > bytes.size() - offset) {
          throw std::runtime_error("Truncated native MIDI SysEx alignment");
        }
        offset += padded;
      } else if ((event.flags & SNDRV_SEQ_EVENT_LENGTH_MASK) != SNDRV_SEQ_EVENT_LENGTH_FIXED) {
        throw std::runtime_error("Unexpected native MIDI event length encoding");
      }
      receive(event, payload, timestamp);
    }
  }

  void drain(const std::stop_token &stop) {
    for (std::size_t batch = 0; batch < capacity / batch_size && !stop.stop_requested(); ++batch) {
      const auto bytes = ::read(seq.descriptor(), buffer.data(), buffer.size());
      const auto error = errno;
      const auto received = Input::now();
      if (bytes < 0 && (error == EAGAIN || error == EINTR)) { return; }
      if (bytes < 0 && error == ENOSPC) {
        throw std::runtime_error("Native MIDI kernel input overflow");
      }
      if (bytes < 0) {
        throw std::system_error(error, std::generic_category(), "Read native MIDI packets");
      }
      if (bytes == 0) { throw std::runtime_error("Native MIDI packet stream disconnected"); }
      // Legacy ALSA queue clocks can jump when their shared timer is restarted.
      // This is the native receiver's monotonic receipt point, never a claim
      // about kernel/hardware arrival or the later application dequeue time.
      receive_buffer(std::span(buffer).first(static_cast<std::size_t>(bytes)), received);
    }
  }

  void run(const std::stop_token &stop) {
    constexpr int identity_poll_ms = 50;
    while (!stop.stop_requested()) {
      std::array descriptors{pollfd{.fd = seq.descriptor(), .events = POLLIN, .revents = 0},
                             pollfd{.fd = wakeup.descriptor(), .events = POLLIN, .revents = 0}};
      const auto result = ::poll(descriptors.data(), descriptors.size(), identity_poll_ms);
      if (result < 0 && errno == EINTR) { continue; }
      if (result < 0) {
        throw std::system_error(errno, std::generic_category(), "Poll native MIDI input");
      }
      if (stop.stop_requested()) { return; }
      for (const auto &descriptor : descriptors) {
        if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
          throw std::runtime_error("Native MIDI transport disconnected");
        }
      }
      connection();
      // An oversized event can overflow an empty native FIFO without making it
      // readable. A bounded timeout read must also observe that latched loss.
      drain(stop);
    }
  }
};

std::vector<Source> Input::sources() {
  const Sequencer seq;
  std::vector<Source> result;
  snd_seq_client_info client{};
  client.client = -1;
  while (::ioctl(seq.descriptor(), SNDRV_SEQ_IOCTL_QUERY_NEXT_CLIENT, &client) == 0) {
    snd_seq_port_info port{};
    port.addr.client = static_cast<unsigned char>(client.client);
    port.addr.port = std::numeric_limits<unsigned char>::max();
    while (::ioctl(seq.descriptor(), SNDRV_SEQ_IOCTL_QUERY_NEXT_PORT, &port) == 0) {
      if (!selectable(port)) { continue; }
      result.push_back({.id = (client.client << 8) | port.addr.port,
                        .name = std::string(client.name) + ": " + port.name});
    }
    if (errno != ENOENT) {
      throw std::system_error(errno, std::generic_category(), "Discover MIDI ports");
    }
  }
  if (errno != ENOENT) {
    throw std::system_error(errno, std::generic_category(), "Discover MIDI clients");
  }
  return result;
}

std::uint64_t Input::now() {
  timespec value{};
  if (::clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
    throw std::system_error(errno, std::generic_category(), "Read monotonic MIDI clock");
  }
  if (value.tv_sec < 0 || value.tv_nsec < 0) {
    throw std::runtime_error("Negative native MIDI time");
  }
  return nanoseconds(static_cast<std::uint64_t>(value.tv_sec),
                     static_cast<std::uint64_t>(value.tv_nsec));
}

Input::Input(const Route route) : impl_(std::make_unique<Impl>(route)) {}

Input::~Input() = default;

void Input::check() const {
  try {
    if (impl_->failed.load(std::memory_order_acquire)) { std::rethrow_exception(impl_->failure); }
  } catch (const std::exception &error) {
    throw std::runtime_error("MIDI source=" + std::to_string(impl_->route.source) +
                             " group=" + std::to_string(impl_->route.group) + ": " + error.what());
  }
}

std::optional<Packet> Input::read() {
  check();
  const auto at = impl_->tail.load(std::memory_order_relaxed);
  if (at == impl_->head.load(std::memory_order_acquire)) { return {}; }
  auto packet = impl_->packets[at % capacity];
  impl_->tail.store(at + 1, std::memory_order_release);
  return packet;
}
}
