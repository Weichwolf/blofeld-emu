// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_esai.hpp"
#include "dsp_esai_registers.hpp"

#include <cstdint>
#include <limits>

namespace blofeld {
void DspEsai::start_frame() {
  if (frames_ == std::numeric_limits<std::uint64_t>::max()) {
    throw Fault("DSP ESAI frame counter exhausted at " + address(esai::saisr));
  }
  if (frame_ && (position_ != format().slot || slot_ + 1 != frame_slots())) {
    throw Fault("Early DSP ESAI frame sync at " + address(esai::saisr));
  }
  ++frames_;
  frame_ = true;
  pending_frame_ = false;
  latched_mask_ = transmit_mask_;
  running_ = enabled();
  stop_after_slot_ = 0;
  slot_ = 0;
  start_slot();
}

void DspEsai::start_slot() {
  if (slots_ == std::numeric_limits<std::uint64_t>::max()) {
    throw Fault("DSP ESAI slot counter exhausted at " + address(esai::saisr));
  }
  ++slots_;
  position_ = 0;
  const bool slot_enabled = (latched_mask_ & (1U << slot_)) != 0;
  const bool network = (control_ & esai::network) != 0;
  const bool transfer = running_ != 0 && slot_enabled && (slot_ == 0 || network);
  status_ = (status_ & ~esai::frame_flag) | (slot_ == 0 ? esai::frame_flag : 0U);
  slot_driven_ = transfer && !skip_;
  if (!transfer) { return; }
  if (!skip_) {
    if ((written_ & running_) != running_) { status_ |= esai::underrun; }
    shift_ = data_;
    shift_known_ = known_;
  }
  written_ &= ~running_;
  skip_ = false;
  status_ |= esai::empty | ((slot_ & 1U) == 0 ? esai::even_empty : esai::odd_empty);
}

unsigned DspEsai::data_bit(const unsigned position) const {
  const auto layout = format();
  const bool right = (control_ & esai::right_align) != 0;
  const unsigned before = right ? layout.slot - layout.word : 0;
  unsigned in_word = 0;
  if (position >= before + layout.word) {
    in_word = layout.word - 1;
  } else if (position >= before) {
    in_word = position - before;
  }
  const unsigned top = (common_control_ & esai::alignment) != 0 ? 15U : 23U;
  return (control_ & esai::shift_lsb) == 0 ? top - in_word : top + 1 - layout.word + in_word;
}

void DspEsai::emit_bit() {
  output_ = {};
  if (!slot_driven_) { return; }
  const auto layout = format();
  const bool right = (control_ & esai::right_align) != 0;
  const unsigned before = right ? layout.slot - layout.word : 0;
  const bool padding = position_ < before || position_ >= before + layout.word;
  const bool zero = padding && (control_ & esai::zero_pad) != 0;
  const auto bit = data_bit(position_);
  output_.driven = running_;
  output_.known = zero ? running_ : shift_known_ & running_;
  for (unsigned channel = 0; channel < 6; ++channel) {
    const auto mask = 1U << channel;
    if (!zero && (output_.known & mask) != 0 && (shift_.at(channel) & (1U << bit)) != 0) {
      output_.levels |= mask;
    }
  }
}

void DspEsai::launch() {
  if (pending_frame_) {
    start_frame();
  } else if (frame_ && position_ == format().slot) {
    running_ &= ~stop_after_slot_;
    ++slot_;
    if (slot_ == frame_slots()) {
      frame_ = false;
      running_ = 0;
      output_ = {};
      status_ &= ~esai::frame_flag;
      return;
    }
    start_slot();
  }
  if (!frame_) {
    output_ = {};
    return;
  }
  emit_bit();
  ++position_;
}

void DspEsai::observe_checked(const DspEsaiInputs inputs) {
  if (!active_ || (control_ & esai::personal_reset) != 0 || (enabled() == 0 && running_ == 0)) {
    baseline_ = false;
    previous_clock_ = inputs.clock;
    output_ = {};
    return;
  }
  if (!inputs.clock) { throw Fault("Unknown DSP ESAI SCKT at " + address(esai::tccr)); }
  const bool inverted = (clock_control_ & esai::frame_polarity) != 0;
  if (!baseline_) {
    if (!inputs.frame_sync || *inputs.frame_sync != inverted) {
      throw Fault("DSP ESAI baseline requires known inactive FST at " + address(esai::tccr));
    }
    previous_clock_ = inputs.clock;
    previous_sync_ = false;
    baseline_ = true;
    return;
  }
  if (previous_clock_ == inputs.clock) { return; }
  previous_clock_ = inputs.clock;
  const bool launch_edge = *inputs.clock != ((clock_control_ & esai::clock_polarity) != 0);
  if (launch_edge) {
    launch();
    return;
  }
  if (!inputs.frame_sync) { throw Fault("Unknown DSP ESAI FST at " + address(esai::tccr)); }
  const bool sync = *inputs.frame_sync != inverted;
  if (sync && previous_sync_) {
    throw Fault("DSP ESAI bit frame sync spans multiple bits at " + address(esai::tccr));
  }
  if (sync) { pending_frame_ = true; }
  previous_sync_ = sync;
}

void DspEsai::observe(const DspEsaiInputs inputs) {
  auto next = *this;
  next.observe_checked(inputs);
  *this = next;
}
}
