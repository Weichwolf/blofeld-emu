// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/midi.hpp"

#include "io/midi_port.hpp"

#include <cstdint>
#include <optional>

namespace blofeld::vi::dsp {
namespace {
constexpr unsigned data_bits = 8, character_bits = data_bits + 2;
constexpr auto bit_ticks = MidiPort::byte_ticks / character_bits;
static_assert(bit_ticks * character_bits == MidiPort::byte_ticks);
constexpr auto receiver = MidiPort::Receiver::dsp;
}

void MidiInput::sample() noexcept {
  level_ = true;
  if (port_ == nullptr) { return; }
  const auto event = port_->peek_input(receiver);
  if (!event || event->ticks > time_) { return; }
  const auto bit = (time_ - event->ticks) / bit_ticks;
  if (bit == 0) {
    level_ = false;
  } else if (bit <= data_bits) {
    level_ = ((unsigned{event->byte} >> (bit - 1)) & 1U) != 0;
  }
}

void MidiInput::reset(const std::uint64_t now) noexcept {
  time_ = now;
  error_ = nullptr;
  if (port_ != nullptr) {
    while (const auto event = port_->peek_input(receiver)) {
      if (now < event->ticks + MidiPort::byte_ticks) { break; }
      port_->pop_input(receiver);
    }
  }
  sample();
}

bool MidiInput::advance(const std::uint64_t now) noexcept {
  if (error() != nullptr) { return false; }
  if (now < time_) {
    error_ = "DSP DIN-MIDI time moved backwards";
    return false;
  }
  if (port_ != nullptr) {
    while (const auto event = port_->peek_input(receiver)) {
      if (now < event->ticks + MidiPort::byte_ticks) { break; }
      if (time_ < event->ticks) {
        error_ = "DSP execution missed a complete DIN-MIDI character";
        return false;
      }
      port_->pop_input(receiver);
    }
  }
  time_ = now;
  sample();
  return error() == nullptr;
}

std::optional<std::uint64_t> MidiInput::deadline() const noexcept {
  if (port_ == nullptr) { return {}; }
  const auto event = port_->peek_input(receiver);
  if (!event) { return {}; }
  if (event->ticks > time_) { return event->ticks; }
  const auto bit = (time_ - event->ticks) / bit_ticks;
  return event->ticks + ((bit + 1) * bit_ticks);
}

const char *MidiInput::error() const noexcept {
  if (error_ != nullptr) { return error_; }
  return port_ != nullptr ? port_->error() : nullptr;
}
}
