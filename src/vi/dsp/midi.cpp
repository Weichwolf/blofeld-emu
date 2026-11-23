// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/midi.hpp"

#include "io/midi_port.hpp"

#include <bit>
#include <cstdint>
#include <limits>
#include <optional>
#include <stop_token>

namespace blofeld::vi::dsp {
namespace {
constexpr unsigned data_bits = 8, character_bits = data_bits + 2;
constexpr auto bit_ticks = MidiPort::byte_ticks / character_bits;
static_assert(bit_ticks * character_bits == MidiPort::byte_ticks);
constexpr auto receiver = MidiPort::Receiver::dsp;
}

std::uint64_t MidiInput::horizon() const noexcept {
  return port_ == nullptr ? std::numeric_limits<std::uint64_t>::max() : port_->input_knowledge();
}

bool MidiInput::known() const noexcept {
  if (port_ == nullptr) { return true; }
  const auto limit = port_->input_limit();
  return !limit || time_ < *limit;
}

bool MidiInput::await_current(const std::stop_token &stop) const noexcept {
  return port_ == nullptr || port_->await_input(time_, stop);
}

void MidiInput::sample(const std::optional<MidiPort::Event> &event) noexcept {
  level_ = true;
  deadline_.reset();
  if (!event) { return; }
  if (event->ticks > time_) {
    deadline_ = event->ticks;
    return;
  }
  const auto bit = static_cast<unsigned>((time_ - event->ticks) / bit_ticks);
  constexpr unsigned stop_bit = 1U << (data_bits + 1);
  constexpr unsigned end = 1U << character_bits;
  const auto levels = stop_bit | (unsigned{event->byte} << 1U);
  level_ = ((levels >> bit) & 1U) != 0;
  // The current level and next transition use the same published character.
  const auto changes = ((levels ^ (levels << 1U)) & (end - 1U)) | end;
  const auto first = bit + 1;
  const auto next = first + static_cast<unsigned>(std::countr_zero(changes >> first));
  deadline_ = event->ticks + (next * bit_ticks);
}

void MidiInput::reset(const std::uint64_t now) noexcept {
  time_ = now;
  error_ = nullptr;
  auto event = port_ == nullptr ? std::nullopt : port_->peek_input(receiver);
  while (event && now >= event->ticks + MidiPort::byte_ticks) {
    port_->pop_input(receiver);
    event = port_->peek_input(receiver);
  }
  sample(event);
}

bool MidiInput::advance(const std::uint64_t now) noexcept {
  if (error() != nullptr) { return false; }
  if (now < time_) {
    error_ = "DSP DIN-MIDI time moved backwards";
    return false;
  }
  auto event = port_ == nullptr ? std::nullopt : port_->peek_input(receiver);
  while (event && now >= event->ticks + MidiPort::byte_ticks) {
    if (time_ < event->ticks) {
      error_ = "DSP execution missed a complete DIN-MIDI character";
      return false;
    }
    port_->pop_input(receiver);
    event = port_->peek_input(receiver);
  }
  time_ = now;
  sample(event);
  return error() == nullptr;
}

const char *MidiInput::error() const noexcept {
  if (error_ != nullptr) { return error_; }
  return port_ != nullptr ? port_->error() : nullptr;
}
}
