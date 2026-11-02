// SPDX-License-Identifier: GPL-3.0-only
#include "midi/midi_serial_receiver.hpp"

#include "common/bus.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <string_view>

namespace blofeld {
namespace {
constexpr std::uint64_t baud = 31250, half_bit_rate = 2 * baud;
constexpr unsigned data_bits = 8, stop_slot = data_bits + 1;
constexpr unsigned last_sample_numerator = (2 * stop_slot) + 1;
}

MidiSerialReceiver::MidiSerialReceiver(const std::uint64_t timebase_hz) : timebase_(timebase_hz) {
  if (timebase_ < half_bit_rate ||
      timebase_ > std::numeric_limits<std::uint64_t>::max() / last_sample_numerator) {
    throw Fault("MIDI serial receiver timebase cannot represent bounded half-bit samples");
  }
}

void MidiSerialReceiver::fail(const std::uint64_t ticks, const std::string_view reason) {
  faulted_ = true;
  throw Fault("MIDI serial receiver operation=observe ticks=" + std::to_string(ticks) +
              " start=" + std::to_string(start_) + " slot=" + std::to_string(slot_) + ": " +
              std::string(reason));
}

void MidiSerialReceiver::schedule() {
  const auto numerator = timebase_ * ((2 * slot_) + 1);
  const auto offset = (numerator / half_bit_rate) + (numerator % half_bit_rate != 0 ? 1U : 0U);
  if (start_ > std::numeric_limits<std::uint64_t>::max() - offset) {
    fail(start_, "sample deadline overflow");
  }
  next_ = start_ + offset;
}

std::optional<MidiSerialByte> MidiSerialReceiver::sample(const bool level) {
  if (!next_) { fail(last_.value_or(0), "missing pending sample"); }
  const auto at = *next_;
  if (slot_ == 0 && level) { fail(at, "invalid start bit"); }
  if (slot_ > 0 && slot_ <= data_bits) { word_ |= static_cast<unsigned>(level) << (slot_ - 1); }
  if (slot_ == stop_slot) {
    if (!level) { fail(at, "invalid stop bit"); }
    if (received_ == std::numeric_limits<std::uint64_t>::max()) {
      fail(at, "received-byte counter overflow");
    }
    const MidiSerialByte result{
        .value = static_cast<std::uint8_t>(word_), .start_ticks = start_, .stop_sample_ticks = at};
    ++received_;
    next_.reset();
    return result;
  }
  ++slot_;
  schedule();
  return std::nullopt;
}

std::optional<MidiSerialByte> MidiSerialReceiver::observe(const std::uint64_t ticks,
                                                          const std::optional<bool> level) {
  if (faulted_) { fail(ticks, "requires coordinated recovery after a rejected observation"); }
  if (last_ && ticks < *last_) { fail(ticks, "nonmonotonic observation"); }
  if (last_ == ticks && level != previous_) { fail(ticks, "contradictory same-tick levels"); }
  std::optional<MidiSerialByte> result;
  while (next_ && *next_ < ticks) {
    if (!previous_) { fail(ticks, "unknown active level"); }
    result = sample(*previous_);
  }
  if (!next_ && previous_ == true && level == false) {
    start_ = ticks;
    slot_ = 0;
    word_ = 0;
    schedule();
  }
  previous_ = level;
  last_ = ticks;
  if (next_) {
    if (!level) { fail(ticks, "unknown active level"); }
    if (*next_ == ticks) { result = sample(*level); }
  }
  return result;
}
}
