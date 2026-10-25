// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_dax.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_dax_registers.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
void DspDax::reset_paths() {
  buffers_ = {};
  last_words_ = {};
  pending_.reset();
  previous_.reset();
  status_ = 0;
  next_write_ = 0;
  clear_writes_ = 0;
  phase_ = 0;
  position_ = 0;
  block_frame_ = 0;
  half_bits_ = 0;
  level_ = true;
  repeating_ = false;
  clear_armed_ = false;
  rates_frozen_ = false;
}

void DspDax::reset() {
  control_ = 0;
  active_ = false;
  reset_paths();
  // XNADR is unaffected by every documented reset; cold contents stay unknown.
}

bool DspDax::handles(const unsigned address) {
  return address >= dax::control && address <= dax::status;
}

DspDax::Register DspDax::reg(const unsigned address) {
  if (!handles(address)) { throw Fault("Unmapped DAX register"); }
  return static_cast<Register>(address - dax::control);
}

void DspDax::set_active(const bool active) {
  if (active_ && !active) { reset_paths(); }
  if (!active_ && active) { previous_ = clock(observed_); }
  active_ = active;
}

void DspDax::promote() {
  if (pending_ && !buffers_.at(pending_->channel)) {
    buffers_.at(pending_->channel) = pending_->value;
    pending_.reset();
  }
}

void DspDax::write_audio(const std::uint32_t value) {
  if (pending_) { throw Fault("Unqualified DAX write while XADR is full"); }
  const auto channel = next_write_;
  pending_ = Pending{.channel = channel, .value = value};
  next_write_ ^= 1U;
  promote();
  if (clear_armed_) { clear_writes_ |= 1U << channel; }
  if (channel == 1) { status_ &= ~(dax::empty | dax::last_block_frame); }
  if (clear_armed_ && clear_writes_ == 3) {
    status_ &= ~dax::underrun;
    clear_armed_ = false;
    clear_writes_ = 0;
  }
}

void DspDax::write_checked(const Register target, const std::uint32_t value) {
  if (value > word_mask) { throw Fault("DAX value exceeds24 bits"); }
  if (target == Register::audio_a || target == Register::audio_b) {
    write_audio(value); // Both aliases advance the same alternating A/B input.
    return;
  }
  if (target == Register::non_audio) {
    if ((value & ~dax::non_audio_mask) != 0) { throw Fault("Reserved DAX XNADR fields"); }
    non_audio_ = value;
    return;
  }
  if (target != Register::control) { throw Fault("Undefined/read-only DAX register"); }
  if ((value & ~(dax::start_block | dax::clock_select)) != 0) {
    throw Fault("Unimplemented/reserved DAX interrupt/control fields");
  }
  if (active_ && ((value ^ control_) & dax::clock_select) != 0) {
    throw Fault("DAX clock source changed while enabled");
  }
  control_ = value;
  if (!active_) { previous_ = clock(observed_); }
}

void DspDax::write(const unsigned address, const std::uint32_t value) {
  auto next = *this;
  try {
    next.write_checked(reg(address), value);
  } catch (const Fault &error) {
    throw Fault("DSP DAX register write at X:" + hex(address, 6) + ": " + error.what());
  }
  *this = next;
}

std::uint32_t DspDax::read_checked(const Register target, const std::uint32_t mask) {
  if (mask > word_mask) { throw Fault("DAX read mask exceeds24 bits"); }
  if (target == Register::control) { return control_ & mask; }
  if (target != Register::status) { throw Fault("Undefined/write-only DAX register"); }
  if (active_) {
    require_clock(observed_);
    if (external_clock() && (mask & dax::underrun) != 0) {
      throw Fault("Unqualified DAX slave XAUR flag: DSP56371E ED55");
    }
  }
  if ((mask & status_ & dax::underrun) != 0 && !clear_armed_) {
    clear_armed_ = true;
    clear_writes_ = 0;
  }
  return status_ & mask;
}

std::uint32_t DspDax::read(const unsigned address, const std::uint32_t mask) {
  auto next = *this;
  try {
    const auto value = next.read_checked(reg(address), mask);
    *this = next;
    return value;
  } catch (const Fault &error) {
    throw Fault("DSP DAX register read at X:" + hex(address, 6) + ": " + error.what());
  }
}

DspDaxOutput DspDax::output() const {
  if (!active_) { return {}; }
  require_clock(observed_);
  return {.driven = true, .level = level_};
}

bool DspDax::request() const {
  if (active_) { require_clock(observed_); }
  return active_ && (status_ & dax::empty) != 0;
}
}
