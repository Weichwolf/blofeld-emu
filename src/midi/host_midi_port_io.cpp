// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "midi/detail/host_midi_port.hpp"
#include "midi/host_midi_port.hpp"
#include "midi/sequence_codec.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <sound/asequencer.h>
#include <span>
#include <string>
#include <unistd.h>

namespace blofeld {
bool HostMidiPort::Impl::fill() {
  const auto result = ::read(descriptor, input.data(), input.size());
  if (result < 0) {
    if (errno == EAGAIN || errno == EINTR) { return false; }
    fail(errno == ENOSPC ? "input FIFO overflow" : "read events", errno);
  }
  if (result == 0) { throw Fault("Host MIDI sequencer reached EOF"); }
  count = static_cast<std::size_t>(result);
  position = 0;
  return true;
}

void HostMidiPort::Impl::next() {
  if (count - position < wire_event) { throw Fault("Truncated host MIDI sequencer record"); }
  snd_seq_event event{};
  std::memcpy(&event, input.data() + position, wire_event);
  position += wire_event;
  if ((event.flags & SNDRV_SEQ_EVENT_UMP) != 0) {
    throw Fault("Unexpected UMP record in a legacy MIDI-1 host session");
  }
  const auto length = event.flags & SNDRV_SEQ_EVENT_LENGTH_MASK;
  if (length == SNDRV_SEQ_EVENT_LENGTH_VARIABLE) {
    const auto bytes = static_cast<std::size_t>(event.data.ext.len);
    if (event.type != SNDRV_SEQ_EVENT_SYSEX || bytes == 0 || bytes > count - position) {
      throw Fault("Unsupported or incomplete variable host MIDI record type=" +
                  std::to_string(event.type));
    }
    const auto padded = ((bytes + wire_event - 1) / wire_event) * wire_event;
    if (padded > count - position) { throw Fault("Truncated host MIDI variable record padding"); }
    payload = position;
    remaining = bytes;
    position += padded;
    return;
  }
  if (length != SNDRV_SEQ_EVENT_LENGTH_FIXED) {
    throw Fault("Unexpected external pointer in a host MIDI input record");
  }
  switch (event.type) {
    case SNDRV_SEQ_EVENT_PORT_SUBSCRIBED:
    case SNDRV_SEQ_EVENT_PORT_UNSUBSCRIBED:
    case SNDRV_SEQ_EVENT_NONE: return;
    default:
      decoded = detail::sequence_bytes(event);
      decoded_position = 0;
      break;
  }
}

std::size_t HostMidiPort::Impl::receive(const std::span<std::uint8_t> destination) {
  std::size_t written = 0;
  unsigned events = 0;
  while (written < destination.size()) {
    if (remaining != 0) {
      const auto amount = std::min(remaining, destination.size() - written);
      std::ranges::copy(std::span{input}.subspan(payload, amount),
                        destination.subspan(written).begin());
      remaining -= amount;
      payload += amount;
      written += amount;
    } else if (decoded_position < decoded.size) {
      const auto amount = std::min(decoded.size - decoded_position, destination.size() - written);
      std::ranges::copy(std::span{decoded.data}.subspan(decoded_position, amount),
                        destination.subspan(written).begin());
      decoded_position += amount;
      written += amount;
    } else {
      if (events == event_budget) { break; }
      if (position == count && !fill()) { break; }
      next();
      ++events;
    }
  }
  return written;
}

void HostMidiPort::Impl::deliver() {
  const auto *packet = encoder.pending();
  if (packet == nullptr) { return; }
  auto event = packet->event;
  event.source = {.client = static_cast<std::uint8_t>(endpoint.client),
                  .port = static_cast<std::uint8_t>(endpoint.port)};
  event.dest = {.client = SNDRV_SEQ_ADDRESS_SUBSCRIBERS, .port = 0};
  event.queue = SNDRV_SEQ_QUEUE_DIRECT;
  std::array<std::uint8_t, wire_event + 256> buffer{};
  std::memcpy(buffer.data(), &event, wire_event);
  std::ranges::copy(std::span{packet->data}.first(packet->size),
                    std::span{buffer}.subspan(wire_event).begin());
  const auto bytes = wire_event + packet->size;
  const auto result = ::write(descriptor, buffer.data(), bytes);
  // Direct multicast delivery can already have affected another subscriber.
  // A syscall failure retires this session; retry must not duplicate that prefix.
  if (result < 0) { fail("write event; delivery may be partial", errno); }
  if (static_cast<std::size_t>(result) != bytes) {
    throw Fault("Host MIDI sequencer accepted an incomplete output record");
  }
  encoder.consume();
}

void HostMidiPort::Impl::send(const std::span<const std::uint8_t> bytes) {
  std::size_t index = 0;
  while (index < bytes.size()) {
    const bool consumed = encoder.feed(bytes[index]);
    deliver();
    if (consumed) { ++index; }
  }
}
}
