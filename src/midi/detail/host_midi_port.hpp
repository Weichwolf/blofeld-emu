// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "midi/host_midi_port.hpp"
#include "midi/sequence_codec.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <sound/asequencer.h>
#include <span>
#include <string_view>

namespace blofeld {
struct HostMidiPort::Impl {
  explicit Impl(std::string_view name);
  ~Impl();
  Impl(const Impl &) = delete;
  Impl &operator=(const Impl &) = delete;
  void require_active() const;
  void check_status() const;
  void configure(std::string_view name);
  void configure_pools();
  [[nodiscard]] snd_seq_client_pool query_pool() const;
  [[nodiscard]] int request_pool(bool input_pool, int preferred) const;
  void close() noexcept;
  [[noreturn]] void fail(const char *operation, int error) const;
  std::size_t receive(std::span<std::uint8_t> destination);
  void send(std::span<const std::uint8_t> bytes);
  bool fill();
  void next();
  void deliver();
  static constexpr std::size_t wire_event = sizeof(snd_seq_event);
  static constexpr std::size_t receive_bytes = wire_event * 32768;
  static constexpr unsigned event_budget = 16;
  int descriptor = -1;
  HostMidiAddress endpoint;
  HostMidiLimits granted;
  std::array<std::uint8_t, receive_bytes> input{};
  std::size_t position = 0, count = 0;
  std::size_t payload = 0, remaining = 0;
  detail::SequenceBytes decoded;
  std::size_t decoded_position = 0;
  detail::SequenceEncoder encoder;
  bool failed = false;
};
}
