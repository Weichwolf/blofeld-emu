// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_esai.hpp"

#include "blofeld/bus.hpp"
#include "dsp_esai_registers.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace blofeld {
DspEsai::DspEsai(const DspEsaiInstance instance) : instance_(instance) {
  if (instance != DspEsaiInstance::primary && instance != DspEsaiInstance::secondary) {
    throw Fault("Invalid DSP ESAI instance");
  }
  reset();
}

std::string DspEsai::address(const unsigned offset) const {
  if (offset > esai::rsmb) { return "invalid-offset=" + std::to_string(offset); }
  constexpr unsigned primary_base = 0xffffa0;
  constexpr unsigned secondary_base = 0xffff80;
  const bool primary = instance_ == DspEsaiInstance::primary;
  return std::string(primary ? "X:" : "Y:") +
         hex((primary ? primary_base : secondary_base) + offset, 6);
}

void DspEsai::reset_paths() {
  status_ = 0;
  known_ = 0;
  shift_known_ = 0;
  written_ = 0;
  clear_written_ = 0;
  running_ = 0;
  stop_after_slot_ = 0;
  position_ = 0;
  slot_ = 0;
  previous_clock_.reset();
  previous_sync_ = false;
  baseline_ = false;
  frame_ = false;
  pending_frame_ = false;
  slot_driven_ = false;
  skip_ = false;
  clear_armed_ = false;
  output_ = {};
  high_phase_ = 0;
  frame_position_ = 0;
  generated_clock_ = false;
  generated_sync_ = false;
  frame_clocked_ = false;
}

void DspEsai::reset() {
  control_ = 0;
  clock_control_ = 0;
  common_control_ = 0;
  receive_control_ = 0;
  transmit_mask_ = UINT32_MAX;
  latched_mask_ = UINT32_MAX;
  receive_mask_ = UINT32_MAX;
  frames_ = 0;
  slots_ = 0;
  active_ = false;
  reset_paths();
}

void DspEsai::set_active(const bool active) {
  if (active_ && !active) { reset_paths(); }
  const bool activate = active && !active_;
  active_ = active;
  if (activate) { rebaseline(); }
}

bool DspEsai::handles(const unsigned offset) {
  return offset <= esai::tsr || (offset >= esai::rx_first && offset <= esai::rx_last) ||
         (offset >= esai::saisr && offset <= esai::rsmb);
}

DspEsai::Format DspEsai::format() const {
  // DSP56371UM tables8-5/9-5, including the non-linear encodings for24-bit words.
  constexpr std::array<Format, 32> formats{{{.slot = 8, .word = 8},
                                            {.slot = 12, .word = 12},
                                            {.slot = 16, .word = 16},
                                            {.slot = 20, .word = 20},
                                            {.slot = 12, .word = 8},
                                            {.slot = 16, .word = 12},
                                            {.slot = 20, .word = 16},
                                            {.slot = 24, .word = 20},
                                            {.slot = 16, .word = 8},
                                            {.slot = 20, .word = 12},
                                            {.slot = 24, .word = 16},
                                            {},
                                            {.slot = 20, .word = 8},
                                            {.slot = 24, .word = 12},
                                            {},
                                            {.slot = 32, .word = 20},
                                            {.slot = 24, .word = 8},
                                            {},
                                            {.slot = 32, .word = 16},
                                            {},
                                            {},
                                            {.slot = 32, .word = 12},
                                            {},
                                            {},
                                            {.slot = 32, .word = 8},
                                            {},
                                            {},
                                            {},
                                            {},
                                            {},
                                            {.slot = 24, .word = 24},
                                            {.slot = 32, .word = 24}}};
  constexpr unsigned format_shift = 10;
  constexpr unsigned format_mask = 31;
  return formats.at((control_ >> format_shift) & format_mask);
}

unsigned DspEsai::enabled() const {
  return (control_ & esai::personal_reset) != 0 ? 0U : control_ & esai::channels;
}

bool DspEsai::needs_transmit_clock() const {
  return active_ && (control_ & esai::personal_reset) == 0 && (enabled() != 0 || running_ != 0);
}

bool DspEsai::transmit_request() const {
  require_observed_clock();
  return (status_ & esai::empty) != 0;
}

void DspEsai::require_observed_clock() const {
  if (needs_transmit_clock() && (clock_generator() || frame_generator()) &&
      observed_.timing != DspEsaiClockTiming::reference_reset_edges) {
    throw Fault("DSP ESAI generator requires explicit reference reset-edge timing at " +
                address(esai::tccr));
  }
  if (needs_transmit_clock() && !baseline_) {
    throw Fault("Unknown DSP ESAI clock/frame observation at " + address(esai::saisr));
  }
}

DspEsaiOutputs DspEsai::outputs() const {
  require_observed_clock();
  return output_;
}

unsigned DspEsai::frame_slots() const {
  constexpr unsigned rate_shift = 9;
  constexpr unsigned rate_mask = 31;
  return ((clock_control_ >> rate_shift) & rate_mask) + 1;
}

void DspEsai::validate() const {
  const auto layout = format();
  if (layout.slot == 0 || ((common_control_ & esai::alignment) != 0 && layout.word > 16)) {
    throw Fault("Reserved/unpredictable DSP ESAI word format at " + address(esai::tcr));
  }
  constexpr unsigned unsynchronized_divider = 0x1ff;
  if (clock_generator() && (clock_control_ & unsynchronized_divider) == 0x100) {
    throw Fault("Unpredictable DSP ESAI TPM0/TPSR1 clock divider at " + address(esai::tccr));
  }
  if (enabled() != 0 && (control_ & esai::bit_sync) == 0 && !frame_generator()) {
    throw Fault("Unqualified DSP ESAI word-length frame sync at " + address(esai::tcr));
  }
  if ((enabled() != 0 || generator_running()) && (control_ & esai::bit_sync) == 0 &&
      frame_slots() == 1) {
    throw Fault("DSP ESAI continuous normal frames require bit sync at " + address(esai::tcr));
  }
  if (enabled() != 0 && (control_ & esai::network) != 0 && frame_slots() == 1) {
    throw Fault("Unimplemented DSP ESAI on-demand mode at " + address(esai::tcr));
  }
  if (skip_ && (((control_ & esai::network) != 0 && transmit_mask_ != UINT32_MAX) ||
                ((control_ & esai::network) == 0 && frame_slots() != 1))) {
    throw Fault("Unqualified DSP ESAI TSR/masked or spaced slots at " + address(esai::tsr));
  }
}

std::uint32_t DspEsai::read(const unsigned offset) {
  switch (offset) {
    case esai::saisr:
      require_observed_clock();
      if ((status_ & esai::underrun) != 0 && !clear_armed_) {
        clear_armed_ = true;
        clear_written_ = 0;
      }
      return status_;
    case esai::saicr: return common_control_;
    case esai::tcr: return control_;
    case esai::tccr: return clock_control_;
    case esai::rcr: return receive_control_;
    case esai::rccr: return 0;
    case esai::tsma: return transmit_mask_ & esai::half_mask;
    case esai::tsmb: return transmit_mask_ >> 16U;
    case esai::rsma: return receive_mask_ & esai::half_mask;
    case esai::rsmb: return receive_mask_ >> 16U;
    default: throw Fault("Undefined/unimplemented DSP ESAI read at " + address(offset));
  }
}

void DspEsai::clear_error(const unsigned written) {
  if (!clear_armed_) { return; }
  clear_written_ |= written;
  const auto needed = enabled();
  if (needed != 0 && (clear_written_ & needed) == needed) {
    status_ &= ~esai::underrun;
    clear_armed_ = false;
    clear_written_ = 0;
  }
}

void DspEsai::write_data(const unsigned channel, const std::uint32_t value) {
  if (skip_) { throw Fault("Unqualified DSP ESAI TX/TSR mixture at " + address(channel)); }
  data_.at(channel) = value;
  const auto bit = 1U << channel;
  known_ |= bit;
  written_ |= bit;
  const auto needed = enabled();
  if (needed != 0 && (written_ & needed) == needed) { status_ &= ~esai::empty_flags; }
  clear_error(bit);
}

void DspEsai::write_control(const std::uint32_t value) {
  constexpr unsigned format_fields = 0x7c00;
  constexpr unsigned supported = esai::channels | esai::shift_lsb | esai::right_align |
                                 esai::network | format_fields | esai::bit_sync | esai::zero_pad |
                                 esai::personal_reset;
  if ((value & ~supported) != 0) {
    throw Fault("Unimplemented/reserved DSP ESAI TCR fields at " + address(esai::tcr));
  }
  const auto old = enabled();
  if (active_ && (control_ & esai::personal_reset) == 0 &&
      ((value ^ control_) & ~(esai::channels | esai::personal_reset)) != 0) {
    throw Fault("DSP ESAI configuration requires individual reset at " + address(esai::tcr));
  }
  control_ = value;
  validate();
  if ((control_ & esai::personal_reset) != 0) {
    reset_paths();
  } else {
    stop_after_slot_ |= old & ~enabled();
    if (old == 0 && enabled() != 0 && running_ == 0) {
      baseline_ = false;
      frame_ = false;
      pending_frame_ = false;
      slot_driven_ = false;
      output_ = {};
      rebaseline();
    }
    if ((value & esai::personal_reset) == 0 && old == 0 && enabled() == 0) { rebaseline(); }
    update_frame_flag();
  }
}

void DspEsai::write_configuration(const Configuration kind, const std::uint32_t value) {
  constexpr unsigned common_mask = 0x140;
  constexpr unsigned clock_mask = 0x7fffff;
  const bool common = kind == Configuration::common;
  const auto offset = common ? esai::saicr : esai::tccr;
  const auto allowed = common ? common_mask : clock_mask;
  auto &target = common ? common_control_ : clock_control_;
  const bool changed = value != target;
  if ((value & ~allowed) != 0 || (active_ && (control_ & esai::personal_reset) == 0 && changed)) {
    throw Fault("Unsupported/active DSP ESAI configuration at " + address(offset));
  }
  target = value;
  validate();
  if (changed || !active_) { rebaseline(); }
}

void DspEsai::write_checked(const unsigned offset, const std::uint32_t value) {
  if (value > esai::word_mask) {
    throw Fault("DSP ESAI write exceeds24bits at " + address(offset));
  }
  if (offset < 6) {
    write_data(offset, value);
    return;
  }
  switch (offset) {
    case esai::tsr:
      if ((written_ & enabled()) != 0) {
        throw Fault("Unqualified DSP ESAI TSR/TX mixture at " + address(offset));
      }
      skip_ = true;
      validate();
      status_ &= ~esai::empty_flags;
      clear_error(esai::channels);
      return;
    case esai::tcr: write_control(value); return;
    case esai::saicr: write_configuration(Configuration::common, value); return;
    case esai::tccr: write_configuration(Configuration::transmit_clock, value); return;
    case esai::rcr:
      if (value != 0 && value != esai::personal_reset) {
        throw Fault("Unimplemented DSP ESAI receiver at " + address(offset));
      }
      receive_control_ = value;
      return;
    case esai::rccr:
      if (value != 0) { throw Fault("Unimplemented DSP ESAI RX clock at " + address(offset)); }
      return;
    case esai::tsma:
    case esai::tsmb:
    case esai::rsma:
    case esai::rsmb: {
      if (value > esai::half_mask) {
        throw Fault("Reserved DSP ESAI mask bits at " + address(offset));
      }
      auto &mask = offset <= esai::tsmb ? transmit_mask_ : receive_mask_;
      const auto shift = (offset == esai::tsmb || offset == esai::rsmb) ? 16U : 0U;
      mask = (mask & ~(esai::half_mask << shift)) | (value << shift);
      validate();
      return;
    }
    default: throw Fault("Undefined/unimplemented DSP ESAI write at " + address(offset));
  }
}

void DspEsai::write(const unsigned offset, const std::uint32_t value) {
  auto next = *this;
  next.write_checked(offset, value);
  *this = next;
}
}
