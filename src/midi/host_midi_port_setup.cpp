// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "midi/detail/host_midi_port.hpp"
#include "midi/host_midi_port.hpp"
#include "midi/sequence_codec.hpp"
#include "midi/sequence_pool.hpp"

#include <algorithm>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <fcntl.h>
#include <iterator>
#include <sound/asequencer.h>
#include <string>
#include <string_view>
#include <sys/ioctl.h>
#include <system_error>
#include <unistd.h>

namespace blofeld {
namespace {
constexpr unsigned protocol_major_shift = 16, channels = 16, maximum_address = 252;
constexpr unsigned capabilities = SNDRV_SEQ_PORT_CAP_READ | SNDRV_SEQ_PORT_CAP_WRITE |
                                  SNDRV_SEQ_PORT_CAP_SUBS_READ | SNDRV_SEQ_PORT_CAP_SUBS_WRITE;
}

HostMidiPort::Impl::Impl(const std::string_view name) {
  const snd_seq_client_info info{};
  if (name.empty() || name.size() >= std::size(info.name) || name.contains('\0')) {
    throw Fault("Host MIDI port name must contain 1..63 non-NUL bytes");
  }
  descriptor = ::open("/dev/snd/seq", O_RDWR | O_NONBLOCK | O_CLOEXEC | O_NOCTTY);
  if (descriptor < 0) { fail("open /dev/snd/seq", errno); }
  try {
    configure(name);
  } catch (...) {
    close();
    throw;
  }
}

HostMidiPort::Impl::~Impl() {
  close();
}

void HostMidiPort::Impl::close() noexcept {
  if (descriptor >= 0) { ::close(descriptor); }
  descriptor = -1;
}

void HostMidiPort::Impl::require_active() const {
  if (descriptor < 0 || failed) { throw Fault("Host MIDI port requires a new active session"); }
}

void HostMidiPort::Impl::fail(const char *operation, const int error) const {
  throw Fault("Host MIDI sequencer " + std::string(operation) +
              " client=" + std::to_string(endpoint.client) + ":" + std::to_string(endpoint.port) +
              ": " + std::error_code(error, std::generic_category()).message());
}

void HostMidiPort::Impl::check_status() const {
  snd_seq_client_info info{};
  info.client = static_cast<int>(endpoint.client);
  if (::ioctl(descriptor, SNDRV_SEQ_IOCTL_GET_CLIENT_INFO, &info) < 0) {
    fail("client status", errno);
  }
  if (info.type != USER_CLIENT || info.midi_version != SNDRV_SEQ_CLIENT_LEGACY_MIDI ||
      info.event_lost != 0) {
    throw Fault("Host MIDI sequencer changed protocol or lost delivery reports");
  }
}

void HostMidiPort::Impl::configure(const std::string_view name) {
  int protocol = 0;
  int client = -1;
  if (::ioctl(descriptor, SNDRV_SEQ_IOCTL_PVERSION, &protocol) < 0) { fail("version", errno); }
  if ((static_cast<unsigned>(protocol) >> protocol_major_shift) != 1) {
    throw Fault("Host MIDI requires Linux sequencer protocol version 1");
  }
  if (::ioctl(descriptor, SNDRV_SEQ_IOCTL_CLIENT_ID, &client) < 0) { fail("client ID", errno); }
  if (client < 0 || client > static_cast<int>(maximum_address)) {
    throw Fault("Host MIDI client address is outside the usable range");
  }
  endpoint.client = static_cast<unsigned>(client);
  granted.protocol_version = static_cast<unsigned>(protocol);
  snd_seq_client_info info{};
  info.client = client;
  if (::ioctl(descriptor, SNDRV_SEQ_IOCTL_GET_CLIENT_INFO, &info) < 0) {
    fail("client info", errno);
  }
  std::ranges::fill(info.name, '\0');
  std::ranges::copy(name, std::begin(info.name));
  info.filter = SNDRV_SEQ_FILTER_BOUNCE;
  info.midi_version = SNDRV_SEQ_CLIENT_LEGACY_MIDI;
  if (::ioctl(descriptor, SNDRV_SEQ_IOCTL_SET_CLIENT_INFO, &info) < 0) {
    fail("client setup", errno);
  }
  configure_pools();
  snd_seq_port_info port{};
  port.addr.client = static_cast<std::uint8_t>(client);
  std::ranges::copy(name, std::begin(port.name));
  port.capability = capabilities;
  port.type = SNDRV_SEQ_PORT_TYPE_MIDI_GENERIC | SNDRV_SEQ_PORT_TYPE_APPLICATION |
              SNDRV_SEQ_PORT_TYPE_SOFTWARE;
  port.midi_channels = channels;
  if (::ioctl(descriptor, SNDRV_SEQ_IOCTL_CREATE_PORT, &port) < 0) { fail("create port", errno); }
  endpoint.port = port.addr.port;
  if (port.addr.client != endpoint.client || port.addr.port > maximum_address ||
      (port.capability & capabilities) != capabilities) {
    throw Fault("Host MIDI kernel returned incompatible port capabilities");
  }
  check_status();
}

snd_seq_client_pool HostMidiPort::Impl::query_pool() const {
  snd_seq_client_pool actual{};
  actual.client = static_cast<int>(endpoint.client);
  if (::ioctl(descriptor, SNDRV_SEQ_IOCTL_GET_CLIENT_POOL, &actual) < 0) {
    fail("pool readback", errno);
  }
  if (actual.input_pool < 1 || actual.output_pool < 1) {
    throw Fault("Host MIDI kernel returned invalid pool dimensions");
  }
  return actual;
}

int HostMidiPort::Impl::request_pool(const bool input_pool, const int preferred) const {
  auto request = query_pool();
  if (input_pool) {
    request.input_pool = preferred;
  } else {
    request.output_pool = preferred;
  }
  request.output_room = 1;
  if (::ioctl(descriptor, SNDRV_SEQ_IOCTL_SET_CLIENT_POOL, &request) < 0) {
    fail("pool request", errno);
  }
  const auto actual = query_pool();
  return input_pool ? actual.input_pool : actual.output_pool;
}

void HostMidiPort::Impl::configure_pools() {
  constexpr int preferred_input = 4096;
  constexpr int preferred_output = 256;
  const auto initial = query_pool();
  const auto input_grant =
      detail::negotiate_sequence_pool(preferred_input, initial.input_pool, [this](const int value) {
        return request_pool(true, value);
      });
  const auto output_grant = detail::negotiate_sequence_pool(
      preferred_output, initial.output_pool, [this](const int value) {
        return request_pool(false, value);
      });
  const auto final = query_pool();
  if (final.input_pool != input_grant || final.output_pool != output_grant ||
      final.output_room != 1) {
    throw Fault("Host MIDI negotiated pools changed before port publication");
  }
  granted.input_events = static_cast<std::size_t>(input_grant);
  granted.output_events = static_cast<std::size_t>(output_grant);
  // Reserve an event cell and conservatively allow at most one payload byte
  // per remaining cell; no kernel-internal bytes-per-cell constant is assumed.
  const auto fragment = std::min(std::size_t{256}, granted.output_events - 1);
  encoder = detail::SequenceEncoder(fragment);
}

}
