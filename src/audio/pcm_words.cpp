// SPDX-License-Identifier: GPL-3.0-only
#include "audio/pcm_words.hpp"

#include "audio/serial_pcm.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_esai.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <string_view>

namespace blofeld {
PcmWords::PcmWords(const SerialPcmFormat format, const std::uint64_t after_ticks)
    : format_(format), after_ticks_(after_ticks) {
  if (format.word_bits == 0 || format.word_bits > SerialPcmFormat::dsp_word_bits ||
      format.slot_bits < format.word_bits || format.slot_bits > 32 || format.channels < 2 ||
      format.channels > SerialPcmFrame::maximum_channels) {
    throw Fault("Invalid DSP PCM word format");
  }
}

void PcmWords::reset() {
  *this = PcmWords(format_, after_ticks_);
}

void PcmWords::fail(const std::uint64_t ticks, const std::string_view reason) {
  faulted_ = true;
  throw Fault("DSP PCM words ticks=" + std::to_string(ticks) +
              " sequence=" + std::to_string(sequence_) + " channel=" + std::to_string(next_) +
              ": " + std::string(reason));
}

std::uint64_t
PcmWords::timing(const DspEsaiSlot &slot, const unsigned lane, const PcmWordClock clock) {
  const auto [ticks, high_half_period] = clock;
  if (slot.channels != format_.channels || slot.slot_bits != format_.slot_bits ||
      slot.word_bits != format_.word_bits || !slot.msb_first ||
      slot.shift > SerialPcmFormat::dsp_word_bits - slot.word_bits) {
    fail(ticks, "Transmitter format differs from the output contract");
  }
  if (lane >= slot.words.size() || ((slot.driven & slot.known) & (1U << lane)) == 0) {
    fail(ticks, "Selected transmitter word is disabled or unknown");
  }
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  if (slot.high_edges_per_bit == 0 || high_half_period == 0 ||
      high_half_period > maximum / slot.high_edges_per_bit) {
    fail(ticks, "Transmitter has no representable periodic clock");
  }
  const auto bit_ticks = high_half_period * slot.high_edges_per_bit;
  const auto bits = std::uint64_t{format_.slot_bits} * format_.channels;
  if (bit_ticks > (maximum - ticks) / bits) { fail(ticks, "Transmitter frame time overflows"); }
  return bit_ticks;
}

std::optional<SerialPcmFrame>
PcmWords::observe(const DspEsaiSlot &slot, const unsigned lane, const PcmWordClock clock) {
  const auto ticks = clock.ticks;
  if (faulted_) { fail(ticks, "Faulted receiver requires reset"); }
  if (slot.sequence == sequence_) { return std::nullopt; }
  if (locked_ && (sequence_ == UINT64_MAX || slot.sequence != sequence_ + 1)) {
    fail(ticks, "Transmitter slots were lost or reset without the receiver");
  }
  sequence_ = slot.sequence;
  if (ticks < after_ticks_ || (!locked_ && slot.index != 0)) { return std::nullopt; }
  const auto bit_ticks = timing(slot, lane, clock);
  if (slot.index != next_) { fail(ticks, "Unexpected transmitter channel order"); }
  if (slot.index == 0) {
    frame_ = {.channels = format_.channels, .first_tick = ticks, .bit_ticks = bit_ticks};
    frame_.last_tick =
        ticks + ((std::uint64_t{format_.slot_bits} * format_.channels - 1) * bit_ticks);
    locked_ = true;
  } else if (ticks < frame_.first_tick || frame_.bit_ticks != bit_ticks ||
             ticks - frame_.first_tick !=
                 std::uint64_t{slot.index} * format_.slot_bits * bit_ticks) {
    fail(ticks, "Transmitter word timing changed within a frame");
  }
  const auto sign = std::uint32_t{1} << (format_.word_bits - 1);
  const auto raw = (slot.words[lane] >> slot.shift) & ((sign * 2) - 1);
  frame_.samples.at(next_) =
      static_cast<std::int32_t>(raw ^ sign) - static_cast<std::int32_t>(sign);
  ++next_;
  if (next_ != format_.channels) { return std::nullopt; }
  if (frames_ == UINT64_MAX) { fail(ticks, "Completed frame count overflow"); }
  ++frames_;
  next_ = 0;
  return frame_;
}
}
