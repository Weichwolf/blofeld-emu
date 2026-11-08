// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/audio.hpp"

#include "dsp/clock.hpp"
#include "dsp/dax.hpp"
#include "dsp/esai.hpp"
#include "io/pcm_output.hpp"

#include <cstdint>
#include <dsp56300/memory.hpp>
#include <limits>
#include <optional>
#include <span>

namespace blofeld::vi::dsp {
namespace {
constexpr unsigned reference_hz = 11289600;
constexpr dsp56300::Word sign_bit = 0x800000;

std::int32_t sample(const dsp56300::Word word) noexcept {
  return static_cast<std::int32_t>(word ^ sign_bit) - static_cast<std::int32_t>(sign_bit);
}
}

std::optional<std::uint64_t> Audio::Pulse::deadline() const noexcept {
  if (numerator == 0) { return {}; }
  return floor + (remainder != 0 ? 1U : 0U);
}

bool Audio::Pulse::configure(const std::uint64_t now,
                             const std::uint32_t next_numerator,
                             const std::uint32_t next_denominator) noexcept {
  if (numerator == next_numerator && denominator == next_denominator) { return true; }
  if (next_numerator == 0) {
    *this = {};
    return true;
  }
  if (next_denominator == 0 || next_numerator < next_denominator ||
      now == std::numeric_limits<std::uint64_t>::max()) {
    return false;
  }
  numerator = next_numerator;
  denominator = next_denominator;
  // Divide before multiplying to retain phase without overflowing now * denominator.
  const auto index =
      ((now / numerator) * denominator) + (((now % numerator) * denominator) / numerator) + 1;
  const auto whole = index / denominator;
  const auto fraction = (index % denominator) * numerator;
  if (whole > (std::numeric_limits<std::uint64_t>::max() - numerator) / numerator) { return false; }
  floor = whole * numerator + fraction / denominator;
  remainder = static_cast<std::uint32_t>(fraction % denominator);
  return true;
}

bool Audio::Pulse::advance() noexcept {
  const auto fraction = remainder + (numerator % denominator);
  const auto delta = (numerator / denominator) + (fraction / denominator);
  if (floor > std::numeric_limits<std::uint64_t>::max() - delta - 1) { return false; }
  floor += delta;
  remainder = fraction % denominator;
  return true;
}

bool Audio::fail(const char *message) noexcept {
  error_ = message;
  return false;
}

void Audio::reset(const std::uint64_t epoch) noexcept {
  transmit_ = digital_ = {};
  epoch_ = epoch;
  next_frame_ = 0;
  used_ = frame_ticks_ = 0;
  underruns_ = 0;
  have_left_ = underrun_ = false;
  error_ = nullptr;
}

bool Audio::configure(const std::uint64_t cycles, const Clock::Ratio ratio) {
  std::uint32_t ticks = 0;
  if (esai_.active()) {
    const auto format = esai_.format();
    if (!format || format->slots != 2) {
      return fail("Unsupported stereo ESAI_1 format or clock source");
    }
    ticks = format->slot_ticks;
  }
  if (frame_ticks_ != ticks * 2) {
    if (!submit(true)) { return false; }
    have_left_ = false;
    frame_ticks_ = ticks * 2;
  }
  if (!transmit_.configure(cycles, ticks * ratio.numerator, ratio.denominator)) {
    return fail("ESAI_1 clock deadline is not representable");
  }
  const auto clock = dax_.clock();
  const auto numerator = dax_.connected() ? clock.frame_ticks / 2 : 0;
  if (!digital_.configure(cycles,
                          numerator * (clock.external ? ratio.numerator : 1),
                          clock.external ? ratio.denominator : 1)) {
    return fail("DAX clock deadline is not representable");
  }
  return true;
}

std::optional<std::uint64_t> Audio::deadline() const noexcept {
  const auto transmit = transmit_.deadline();
  const auto digital = digital_.deadline();
  if (!digital || (transmit && *transmit < *digital)) { return transmit; }
  return digital;
}

bool Audio::capture(const Esai::Slot &slot) noexcept {
  if ((slot.enabled & slot.known & 1) == 0) {
    return fail("ESAI_1 output word is disabled or unknown");
  }
  if (slot.index == 0) {
    frame_[0] = sample(slot.samples[0]);
    have_left_ = true;
    underrun_ = slot.repeated;
    return true;
  }
  if (!have_left_ || used_ == frames_.size()) {
    return fail("ESAI_1 stereo output ownership is invalid");
  }
  frame_[1] = sample(slot.samples[0]);
  frames_[used_] = frame_;
  if (underrun_ || slot.repeated) { underruns_ |= static_cast<std::uint16_t>(1U << used_); }
  ++used_;
  ++next_frame_;
  have_left_ = false;
  return true;
}

bool Audio::clock() {
  const auto next = deadline();
  if (!next || error_ != nullptr) { return false; }
  if (transmit_.deadline() == next) {
    const auto slot = esai_.clock_slot();
    if (!slot || !capture(*slot)) { return fail("ESAI_1 clock produced no valid stereo word"); }
    if (!transmit_.advance()) { return fail("ESAI_1 clock overflow"); }
  }
  if (digital_.deadline() == next) {
    static_cast<void>(dax_.clock_subframe());
    if (!digital_.advance()) { return fail("DAX clock overflow"); }
  }
  return true;
}

bool Audio::submit(const bool partial) {
  if (used_ == 0 || (!partial && used_ != frames_.size())) { return true; }
  if (output_.write != nullptr && !output_.write(output_.owner,
                                                 {.frames = std::span(frames_).first(used_),
                                                  .epoch = epoch_,
                                                  .first_frame = next_frame_ - used_,
                                                  .reference_hz = reference_hz,
                                                  .frame_ticks = frame_ticks_,
                                                  .underruns = underruns_},
                                                 stop_)) {
    return fail("PCM consumer rejected an output block");
  }
  used_ = 0;
  underruns_ = 0;
  return true;
}
}
