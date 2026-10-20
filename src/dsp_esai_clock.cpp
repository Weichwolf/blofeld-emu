// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_esai.hpp"
#include "dsp_esai_registers.hpp"

#include <optional>

namespace blofeld {
bool DspEsai::clock_generator() const {
  return (clock_control_ & esai::clock_direction) != 0;
}

bool DspEsai::frame_generator() const {
  return (clock_control_ & esai::frame_direction) != 0;
}

bool DspEsai::generator_running() const {
  return active_ && (control_ & esai::personal_reset) == 0 &&
         (clock_generator() || frame_generator());
}

unsigned DspEsai::high_clock_divisor() const {
  constexpr unsigned modulus_mask = 0xff;
  constexpr unsigned prescale_bypass = 0x100;
  constexpr unsigned high_divider_shift = 14;
  constexpr unsigned high_divider_mask = 15;
  const auto prescale = (clock_control_ & prescale_bypass) != 0 ? 1U : 8U;
  const auto modulus = (clock_control_ & modulus_mask) + 1;
  const auto divider = ((clock_control_ >> high_divider_shift) & high_divider_mask) + 1;
  // The fixed divide-by-two is represented by toggling SCKT on each expiry.
  return prescale * modulus * divider;
}

unsigned DspEsai::required_clock_pads() const {
  if (!generator_running() && !needs_transmit_clock()) { return 0; }
  constexpr unsigned high_clock = 0x20;
  constexpr unsigned bit_clock = 8;
  constexpr unsigned sync = 16;
  return (clock_generator() ? high_clock : bit_clock) | (frame_generator() ? 0U : sync);
}

DspEsaiClockOutputs DspEsai::clock_outputs() const {
  if (!active_) { return {}; }
  validate();
  const unsigned driven = (clock_generator() ? 1U : 0U) | (frame_generator() ? 2U : 0U);
  if (driven == 0 || (control_ & esai::personal_reset) != 0) {
    return {.driven = driven, .known = driven};
  }
  const auto source = clock_generator() ? observed_.high_clock : observed_.clock;
  if (observed_.timing != DspEsaiClockTiming::reference_reset_edges || !source) {
    throw Fault("Unknown/unqualified DSP ESAI generated clock input at " + address(esai::tccr));
  }
  const bool sync = generated_sync_ != ((clock_control_ & esai::frame_polarity) != 0);
  return {.driven = driven,
          .known = driven,
          .levels = ((generated_clock_ ? 1U : 0U) | (sync ? 2U : 0U)) & driven};
}

void DspEsai::rebaseline() {
  baseline_ = false;
  previous_sync_ = false;
  previous_clock_ = clock_generator() ? std::optional{generated_clock_} : observed_.clock;
  if (!active_ || (control_ & esai::personal_reset) != 0 || !previous_clock_) { return; }
  if ((clock_generator() || frame_generator()) &&
      observed_.timing != DspEsaiClockTiming::reference_reset_edges) {
    return;
  }
  if (clock_generator() && !observed_.high_clock) { return; }
  const bool inverted = (clock_control_ & esai::frame_polarity) != 0;
  baseline_ = frame_generator() || (observed_.frame_sync && *observed_.frame_sync == inverted);
}

void DspEsai::generate_frame() {
  const bool bit_sync = (control_ & esai::bit_sync) != 0;
  generated_sync_ = bit_sync ? frame_position_ == 0 : frame_position_ < format().slot;
  if (!bit_sync && frame_position_ == 0 && needs_transmit_clock()) { pending_frame_ = true; }
  frame_clocked_ = true;
  ++frame_position_;
  if (frame_position_ == format().slot * frame_slots()) { frame_position_ = 0; }
  update_frame_flag();
}

void DspEsai::update_frame_flag() {
  if (!frame_generator() || !frame_clocked_ || enabled() == 0) { return; }
  const auto period = format().slot * frame_slots();
  const auto last = (frame_position_ + period - 1) % period;
  const auto data = (control_ & esai::bit_sync) != 0 ? (last + period - 1) % period : last;
  status_ = (status_ & ~esai::frame_flag) | (data < format().slot ? esai::frame_flag : 0U);
}

void DspEsai::clock_edge(const bool clock, const std::optional<bool> sync) {
  if (previous_clock_ == clock) { return; }
  previous_clock_ = clock;
  const bool launch_edge = clock != ((clock_control_ & esai::clock_polarity) != 0);
  if (launch_edge) {
    if (frame_generator()) { generate_frame(); }
    if (needs_transmit_clock()) { launch(); }
    return;
  }
  if (!needs_transmit_clock() || (control_ & esai::bit_sync) == 0) { return; }
  bool active = generated_sync_;
  if (!frame_generator()) {
    if (!sync) { throw Fault("Unknown DSP ESAI FST at " + address(esai::tccr)); }
    active = *sync != ((clock_control_ & esai::frame_polarity) != 0);
  }
  if (active && previous_sync_) {
    throw Fault("DSP ESAI bit frame sync spans multiple bits at " + address(esai::tccr));
  }
  if (active) { pending_frame_ = true; }
  previous_sync_ = active;
}

void DspEsai::observe_high_clock(const DspEsaiInputs inputs) {
  if (!inputs.high_clock) { throw Fault("Unknown DSP ESAI HCKT at " + address(esai::tccr)); }
  const auto previous = observed_.high_clock;
  observed_ = inputs;
  if (!baseline_) { rebaseline(); }
  require_observed_clock();
  const bool polarity = (clock_control_ & esai::high_clock_polarity) != 0;
  if (!previous || previous == inputs.high_clock || *inputs.high_clock == polarity) { return; }
  ++high_phase_;
  if (high_phase_ != high_clock_divisor()) { return; }
  high_phase_ = 0;
  generated_clock_ = !generated_clock_;
  clock_edge(generated_clock_, inputs.frame_sync);
}

void DspEsai::observe_bit_clock(const DspEsaiInputs inputs) {
  if (!inputs.clock) { throw Fault("Unknown DSP ESAI SCKT at " + address(esai::tccr)); }
  observed_ = inputs;
  if (!baseline_) {
    rebaseline();
    if (needs_transmit_clock() && !baseline_) {
      throw Fault("DSP ESAI baseline requires known inactive FST at " + address(esai::tccr));
    }
    return;
  }
  clock_edge(*inputs.clock, inputs.frame_sync);
}
}
