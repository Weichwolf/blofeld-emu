// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_shi.hpp"
#include "dsp_shi_registers.hpp"

namespace blofeld {

void DspShi::load_transmit() {
  if (!active_ || !transmit_ || transmit_fresh_ || word_started_ || !pins_) { return; }
  if (!phase() && pins_->selected) { return; }
  shift_ = *transmit_;
  shift_known_ = shi::word_mask;
  transmit_.reset();
  transmit_fresh_ = true;
}

void DspShi::set_miso() {
  miso_.reset();
  if ((shift_known_ & shi::top_bit) != 0) { miso_ = (shift_ & shi::top_bit) != 0; }
}

void DspShi::begin_word() {
  load_transmit();
  word_started_ = true;
  captured_ = 0;
  first_clock_ = false;
  complete_ = false;
  if (!transmit_fresh_) { errors_ |= shi::underrun; }
  if (!phase()) { set_miso(); }
}

void DspShi::capture_bit(const bool value) {
  const auto low = shi::word_bits - width();
  const auto mask = (shi::word_mask << low) & shi::word_mask;
  const auto incoming = (value ? 1U : 0U) << low;
  // Only the upper 8/16/24 bits form the active IOSR (DSP56371UM 7.4.1).
  // Inactive HTX padding must not shift into the first received bit.
  shift_ = (((shift_ & mask) << 1U) | incoming) & mask;
  shift_known_ = (((shift_known_ & mask) << 1U) | (1U << low)) & mask;
  ++captured_;
  if (captured_ == width()) { finish_word(); }
}

void DspShi::finish_word() {
  const auto capacity = (control_ & shi::fifo) != 0 ? receive_.size() : 1;
  if (receive_count_ == capacity) {
    errors_ |= shi::overrun;
  } else {
    receive_[(receive_head_ + receive_count_) % receive_.size()] = shift_;
    ++receive_count_;
  }
  complete_ = true;
  if (phase()) {
    word_started_ = false;
    first_clock_ = false;
    captured_ = 0;
    load_transmit();
  }
}

void DspShi::validate_pins(const SpiInputs &pins) const {
  if (!pins_) {
    if (pins.selected) { throw Fault("DSP SHI SPI baseline requires deasserted SS"); }
    return;
  }
  const auto old = *pins_;
  if (old.selected != pins.selected && old.clock != pins.clock) {
    throw Fault("Ambiguous simultaneous DSP SHI SS and SCK transitions");
  }
  if (!old.selected && pins.selected && pins.clock != polarity()) {
    throw Fault("DSP SHI selection requires the configured idle SCK level");
  }
  if (!old.selected || !pins.selected || old.clock == pins.clock) { return; }
  const bool leading = pins.clock != polarity();
  const bool capture = phase() ? !leading : leading;
  if (!phase() && complete_ && capture) {
    throw Fault("DSP SHI CPHA=0 requires deasserted SS between words");
  }
  if (capture && !pins.mosi) { throw Fault("Unknown DSP SHI MOSI at a data-capture edge"); }
}

void DspShi::clock_edge(const SpiInputs &pins) {
  const bool leading = pins.clock != polarity();
  if (phase() && leading && !word_started_) { begin_word(); }
  first_clock_ = true;
  transmit_fresh_ = false;
  const bool capture = phase() ? !leading : leading;
  if (capture) {
    if (!pins.mosi) { throw Fault("Unknown DSP SHI MOSI at a data-capture edge"); }
    capture_bit(*pins.mosi);
  } else if (!complete_ || phase()) {
    set_miso();
  }
}

void DspShi::observe_spi(const SpiInputs pins) {
  if (reset_configuration_changed_) {
    if (pins.selected) {
      throw Fault("Unqualified DSP SHI SPI selection during reset reconfiguration at X:0xffff91");
    }
    pins_ = pins;
    return;
  }
  if (!active_) {
    pins_ = pins;
    return;
  }
  validate_pins(pins);
  const auto old = pins_;
  pins_ = pins;
  if (!old) {
    load_transmit();
    return;
  }
  if (old->selected && !pins.selected) {
    // Aborted words retain the partially shifted IOSR but never enter HRX.
    word_started_ = false;
    first_clock_ = false;
    complete_ = false;
    captured_ = 0;
    miso_.reset();
    load_transmit();
    return;
  }
  if (!old->selected && pins.selected) {
    if (!phase()) { begin_word(); }
    return;
  }
  if (pins.selected && old->clock != pins.clock) { clock_edge(pins); }
}

ShiOutputs DspShi::outputs() const {
  ShiOutputs result;
  if (!active_) { return result; }
  if (!pins_) { throw Fault("Unknown DSP SHI pin baseline while observing outputs"); }
  if (pins_->selected) { result.miso = {.driven = true, .level = miso_}; }
  const auto request = (control_ >> 7U) & 3U;
  if (request == 0) { return result; }
  if (reset_configuration_changed_) {
    result.request.driven = true;
    return result; // FIFO/word-size transition: ownership known, readiness unknown.
  }
  if (!phase() && complete_ && pins_->selected) {
    // The precise HREQ reassertion point before the required SS release is uncalibrated.
    result.request.driven = true;
    return result;
  }
  const auto capacity = (control_ & shi::fifo) != 0 ? receive_.size() : 1;
  const bool boundary = !first_clock_ && (phase() || !complete_ || !pins_->selected);
  const bool can_receive = boundary && receive_count_ < capacity;
  const bool can_transmit = boundary && transmit_fresh_;
  const bool ready = (request == 1 && can_receive) || (request == 2 && can_transmit) ||
                     (request == 3 && can_receive && can_transmit);
  result.request = {.driven = true, .level = !ready};
  return result;
}

}
