// SPDX-License-Identifier: GPL-3.0-only
#include "midi/host_midi_port.hpp"

#include "common/bus.hpp"
#include "midi/detail/host_midi_port.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string_view>

namespace blofeld {
HostMidiPort::HostMidiPort(const std::string_view name) : impl_(std::make_unique<Impl>(name)) {}

HostMidiPort::~HostMidiPort() = default;

std::optional<HostMidiAddress> HostMidiPort::address() const {
  return impl_->descriptor < 0 ? std::nullopt : std::optional{impl_->endpoint};
}

std::optional<HostMidiLimits> HostMidiPort::limits() const {
  return impl_->descriptor < 0 ? std::nullopt : std::optional{impl_->granted};
}

void HostMidiPort::retire() noexcept {
  impl_->close();
}

bool HostMidiPort::faulted() const {
  return impl_->failed;
}

std::size_t HostMidiPort::read(const std::span<std::uint8_t> destination) {
  impl_->require_active();
  if (destination.size() > maximum_transfer) {
    throw Fault("Host MIDI input exceeds bounded span");
  }
  try {
    impl_->check_status();
    const auto received = impl_->receive(destination);
    impl_->check_status();
    return received;
  } catch (...) {
    impl_->failed = true;
    impl_->close();
    throw;
  }
}

void HostMidiPort::write(const std::span<const std::uint8_t> bytes) {
  impl_->require_active();
  if (bytes.size() > maximum_transfer) { throw Fault("Host MIDI output exceeds bounded span"); }
  try {
    impl_->check_status();
    impl_->send(bytes);
  } catch (...) {
    impl_->failed = true;
    impl_->close();
    throw;
  }
}
}
