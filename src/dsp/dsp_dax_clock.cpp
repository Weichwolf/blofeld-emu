// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_dax.hpp"
#include "dsp/dsp_dax_registers.hpp"

#include <algorithm>
#include <array>
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
}

void DspDax::begin_subframe() {
  const unsigned channel = position_ == 0 ? 0U : 1U;
  if (channel == 0) { begin_frame(); }
  if (repeating_) {
    const auto retained = last_words_.at(channel);
    if (!retained) { throw Fault("Unknown retained DAX channel"); }
  } else {
    const auto word = buffers_.at(channel);
    if (!word) { throw Fault("Unknown DAX channel at upload X:0xffffd2"); }
    buffers_.at(channel).reset();
    last_words_.at(channel) = word;
    promote();
  }
  if (channel == 0 && !buffers_[0] && !pending_) { status_ |= dax::empty; }
}

void DspDax::advance_units(std::uint64_t units) {
  while (units != 0) {
    if (position_ % dax::subframe_units == 0) { begin_subframe(); }
    const auto span =
        std::min<std::uint64_t>(units, dax::subframe_units - (position_ % dax::subframe_units));
    position_ += static_cast<unsigned>(span);
    units -= span;
    if (position_ == dax::frame_units) {
      position_ = 0;
      ++block_frame_;
    }
  }
}

std::optional<std::uint64_t> DspDax::aci_edges_until_event() const {
  if (!active_ || !external_clock()) { return std::nullopt; }
  require_clock(observed_);
  if (!observed_.aci) { throw Fault("DAX ACI event lacks its current clock state"); }
  const auto offset = position_ % dax::subframe_units;
  const auto units = offset == 0 ? 1U : dax::subframe_units - offset + 1U;
  const auto rises = (units * divider()) - phase_;
  return (std::uint64_t{rises} * 2U) - (*observed_.aci ? 0U : 1U);
}

void DspDax::advance_aci(const std::uint64_t transitions, const DspDaxRates rates) {
  if (!observed_.aci || transitions == 0 || (active_ && !external_clock())) {
    throw Fault("DAX span requires its external clock baseline and positive transitions");
  }
  auto input = observed_;
  input.aci = *observed_.aci != ((transitions & 1U) != 0);
  input.rates = rates;
  if (!active_) {
    observe(input);
    return;
  }
  require_clock(input);
  if (rates_frozen_ && input.rates != observed_.rates) {
    throw Fault("DAX clock rates changed while enabled");
  }
  const auto rises = (transitions / 2U) + (!*observed_.aci && (transitions & 1U) != 0 ? 1U : 0U);
  if (rises > std::numeric_limits<std::uint64_t>::max() - phase_) {
    throw Fault("DAX span exceeds its clock range");
  }
  const auto units = rises + phase_;
  advance_units(units / divider());
  phase_ = static_cast<unsigned>(units % divider());
  observed_ = input;
  previous_ = input.aci;
  rates_frozen_ = true;
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
  advance_units(1);
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
