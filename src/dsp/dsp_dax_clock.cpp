// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_dax.hpp"
#include "dsp/dsp_dax_registers.hpp"

#include <array>
#include <bit>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

namespace blofeld {
bool DspDax::external_clock() const {
  return (control_ & dax::clock_select) != 0;
}

std::optional<bool> DspDax::clock(const DspDaxInputs &inputs) const {
  return external_clock() ? inputs.aci : inputs.core_clock;
}

unsigned DspDax::divider() const {
  // DSP56371UM figure10-5: selected input clocks per128*Fs biphase unit.
  constexpr std::array ratios{8U, 2U, 3U, 4U};
  return ratios.at((control_ & dax::clock_select) >> 3U);
}

void DspDax::require_clock(const DspDaxInputs &inputs) const {
  constexpr std::uint64_t maximum_hz = 1000000000;
  const auto source = external_clock() ? inputs.rates.aci_hz : inputs.rates.core_hz;
  constexpr std::uint64_t minimum_core_ratio = 5;
  constexpr unsigned shift_units = 64;
  const auto frame_ratio = divider() * dax::frame_units;
  if (inputs.timing != DspDaxTiming::reference_rising_edges || !clock(inputs) || source == 0 ||
      source > maximum_hz || inputs.rates.core_hz == 0 || inputs.rates.core_hz > maximum_hz ||
      inputs.rates.core_hz * frame_ratio < source * (minimum_core_ratio * shift_units)) {
    throw Fault("Unknown/unqualified DAX clock/rate at X:0xffffd0");
  }
}

void DspDax::begin_frame() {
  if (!non_audio_) { throw Fault("Unknown DAX XNADR at frame upload X:0xffffd1"); }
  promote();
  const bool a = buffers_[0].has_value();
  const bool b = buffers_[1].has_value();
  if (a != b) { throw Fault("Unqualified DAX partial-frame buffer underrun at X:0xffffd4"); }
  repeating_ = !a;
  if (repeating_ && (!last_words_[0] || !last_words_[1])) {
    throw Fault("Unknown retained DAX frame at X:0xffffd2");
  }
  if (repeating_) { status_ |= dax::underrun; }
  if (block_frame_ == dax::frames_per_block || (control_ & dax::start_block) != 0) {
    block_frame_ = 0;
    control_ &= ~dax::start_block;
  }
  if (block_frame_ + 1 == dax::frames_per_block) { status_ |= dax::last_block_frame; }
  frame_non_audio_ = *non_audio_;
}

void DspDax::begin_subframe() {
  const unsigned channel = position_ == 0 ? 0U : 1U;
  if (channel == 0) { begin_frame(); }
  std::uint32_t audio = 0;
  if (repeating_) {
    const auto retained = last_words_.at(channel);
    if (!retained) { throw Fault("Unknown retained DAX channel"); }
    audio = *retained;
  } else {
    const auto word = buffers_.at(channel);
    if (!word) { throw Fault("Unknown DAX channel at upload X:0xffffd2"); }
    audio = *word;
    buffers_.at(channel).reset();
    last_words_.at(channel) = audio;
    promote();
  }
  if (channel == 0 && !buffers_[0] && !pending_) { status_ |= dax::empty; }
  constexpr unsigned first_non_audio = 10;
  const auto flags = (frame_non_audio_ >> (first_non_audio + channel * 3)) & 7U;
  payload_ = audio | (flags << dax::audio_bits);
  payload_ |= (static_cast<unsigned>(std::popcount(payload_)) & 1U) << dax::payload_bits;
  preamble_ = block_frame_ == 0 ? dax::preamble_z : dax::preamble_x;
  if (channel == 1) { preamble_ = dax::preamble_y; }
}

void DspDax::emit() {
  if (half_bits_ == std::numeric_limits<std::uint64_t>::max()) {
    throw Fault("DAX biphase observation counter exhausted");
  }
  const auto offset = position_ % dax::subframe_units;
  if (offset == 0) { begin_subframe(); }
  if (offset < dax::preamble_units) {
    level_ = ((preamble_ >> (dax::preamble_units - 1 - offset)) & 1U) != 0;
  } else {
    const auto data = offset - dax::preamble_units;
    if ((data & 1U) == 0 || (payload_ & (1U << (data / 2))) != 0) { level_ = !level_; }
  }
  ++half_bits_;
  ++position_;
  if (position_ == dax::frame_units) {
    position_ = 0;
    ++block_frame_;
  }
}

void DspDax::observe_checked(const DspDaxInputs inputs) {
  if (inputs.timing != DspDaxTiming::unqualified &&
      inputs.timing != DspDaxTiming::reference_rising_edges) {
    throw Fault("Invalid DAX timing contract");
  }
  if (!active_) {
    observed_ = inputs;
    previous_ = clock(inputs);
    return;
  }
  require_clock(inputs);
  if (rates_frozen_ && inputs.rates != observed_.rates) {
    throw Fault("DAX clock rates changed while enabled");
  }
  const auto current = clock(inputs);
  if (!current) { throw Fault("Unknown DAX selected clock at X:0xffffd0"); }
  const auto previous = previous_;
  previous_ = current;
  observed_ = inputs;
  rates_frozen_ = true;
  if (!previous || previous == current || !*current) { return; }
  ++phase_;
  if (phase_ != divider()) { return; }
  phase_ = 0;
  emit();
}

void DspDax::observe(const DspDaxInputs inputs) {
  auto next = *this;
  try {
    next.observe_checked(inputs);
  } catch (const Fault &error) {
    throw Fault("DSP DAX clock observation at X:0xffffd0: " + std::string(error.what()));
  }
  *this = next;
}
}
