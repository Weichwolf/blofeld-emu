// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string_view>

namespace blofeld {
struct HostMidiAddress {
  unsigned client = 0;
  unsigned port = 0;
  bool operator==(const HostMidiAddress &) const = default;
};

struct HostMidiLimits {
  unsigned protocol_version = 0;
  std::size_t input_events = 0;
  std::size_t output_events = 0;
  bool operator==(const HostMidiLimits &) const = default;
};

// Non-real-time Linux MIDI-1 transport. Never connects to a physical port by
// itself. Handles and pending host bytes remain outside modeled checkpoints.
class HostMidiPort {
public:
  static constexpr std::size_t maximum_transfer = 4096;
  explicit HostMidiPort(std::string_view name);
  ~HostMidiPort();
  HostMidiPort(const HostMidiPort &) = delete;
  HostMidiPort &operator=(const HostMidiPort &) = delete;
  HostMidiPort(HostMidiPort &&) = delete;
  HostMidiPort &operator=(HostMidiPort &&) = delete;
  [[nodiscard]] std::optional<HostMidiAddress> address() const;
  [[nodiscard]] std::optional<HostMidiLimits> limits() const;
  [[nodiscard]] std::size_t read(std::span<std::uint8_t> destination);
  // A failure can follow an already delivered prefix. It retires the complete
  // session; never retry that prefix or replay host output after model restore.
  void write(std::span<const std::uint8_t> bytes);
  void retire() noexcept;
  [[nodiscard]] bool faulted() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
