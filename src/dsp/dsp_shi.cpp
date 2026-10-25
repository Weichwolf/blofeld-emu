// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_shi.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_shi_registers.hpp"

#include <cstdint>
#include <vector>

namespace blofeld {

DspShi::DspShi() {
  reset();
}

void DspShi::reset() {
  clock_control_ = 1; // DSP56371UM 7.4.5: CPHA=1, CPOL=0 on hardware/software reset.
  control_ = 0;
  constexpr unsigned initial_slave_address = 0xb00000;
  slave_address_ = initial_slave_address;
  active_ = false;
  reset_pending_ = false;
  clear_paths();
  // Externally supplied wire levels survive an internal peripheral reset.
}

void DspShi::clear_paths() {
  reset_configuration_changed_ = false;
  errors_ = 0;
  clear_armed_ = 0;
  interrupt_enable_ = 0;
  interrupt_delay_ = false;
  transmit_.reset();
  shift_ = 0;
  shift_known_ = 0;
  receive_head_ = 0;
  receive_count_ = 0;
  captured_ = 0;
  word_started_ = false;
  first_clock_ = false;
  complete_ = false;
  transmit_fresh_ = false;
  miso_.reset();
}

unsigned DspShi::width() const {
  return (((control_ >> 2U) & 3U) + 1U) * 8U;
}

bool DspShi::phase() const {
  return (clock_control_ & 1U) != 0;
}

bool DspShi::polarity() const {
  return (clock_control_ & 2U) != 0;
}

std::uint32_t DspShi::status() const {
  if (reset_configuration_changed_) {
    throw Fault("Unqualified DSP SHI status during reset reconfiguration at X:0xffff91");
  }
  auto result = (control_ & ~shi::idle) | shi::idle | errors_;
  if (!transmit_) { result |= shi::transmit_empty; }
  if (receive_count_ != 0) { result |= shi::receive_not_empty; }
  const auto capacity = (control_ & shi::fifo) != 0 ? receive_.size() : 1;
  if (receive_count_ == capacity) { result |= shi::receive_full; }
  if (active_) {
    if (!pins_) { throw Fault("Unknown DSP SHI SS/SCK baseline at X:0xffff91"); }
    if (pins_->selected) { result |= shi::busy; }
  }
  return result;
}

std::uint32_t DspShi::read(const std::uint32_t address) {
  switch (address) {
    case shi::hckr: return clock_control_;
    case shi::hcsr: {
      const auto result = status();
      clear_armed_ |= errors_ & (shi::underrun | shi::overrun);
      return result;
    }
    case shi::hsar: return slave_address_;
    case shi::hrx: {
      if (reset_configuration_changed_) {
        throw Fault("Unqualified DSP SHI HRX read during reset reconfiguration at X:0xffff94");
      }
      if (receive_count_ == 0) { throw Fault("Undefined empty DSP SHI FIFO read at X:0xffff94"); }
      const auto result = receive_[receive_head_];
      receive_head_ = (receive_head_ + 1) % receive_.size();
      --receive_count_;
      errors_ &= ~(clear_armed_ & shi::overrun);
      clear_armed_ &= ~shi::overrun;
      return result;
    }
    case shi::htx: throw Fault("Undefined write-only DSP SHI HTX read at X:0xffff93");
    default: throw Fault("Unimplemented DSP SHI read at X:" + hex(address, 6));
  }
}

void DspShi::write(const DspWordWrite access) {
  const auto address = access.address;
  const auto value = access.value;
  if (value > shi::word_mask) {
    throw Fault("DSP SHI write exceeds 24 bits at X:" + hex(address, 6));
  }
  switch (address) {
    case shi::hckr: {
      constexpr unsigned writable = 0x7ff;
      constexpr unsigned prescaler_and_divider = 0x7fc;
      if ((value & ~writable) != 0 || (value & prescaler_and_divider) == 4) {
        throw Fault("Reserved/illegal DSP SHI HCKR write at X:0xffff90");
      }
      if ((active_ || reset_pending_) && value != clock_control_) {
        throw Fault("DSP SHI HCKR change requires completed individual reset at X:0xffff90");
      }
      clock_control_ = value;
      return;
    }
    case shi::hcsr: write_control(value & shi::control_mask); return;
    case shi::hsar: {
      constexpr unsigned writable = 0xf40000;
      if ((value & ~writable) != 0) { throw Fault("Reserved DSP SHI HSAR write at X:0xffff92"); }
      slave_address_ = value;
      return;
    }
    case shi::htx:
      if (reset_configuration_changed_) {
        throw Fault("Unqualified DSP SHI HTX write during reset reconfiguration at X:0xffff93");
      }
      if (!active_) { throw Fault("Unverified DSP SHI HTX write during reset at X:0xffff93"); }
      // DSP56371UM 7.4.2: writing before HTDE overwrites pending HTX data.
      // The separately prepared/in-flight IOSR word is retained.
      transmit_ = value;
      errors_ &= ~(clear_armed_ & shi::underrun);
      clear_armed_ &= ~shi::underrun;
      load_transmit();
      return;
    case shi::hrx: throw Fault("Undefined read-only DSP SHI HRX write at X:0xffff94");
    default: throw Fault("Unimplemented DSP SHI write at X:" + hex(address, 6));
  }
}

void DspShi::write_control(const std::uint32_t value) {
  constexpr unsigned mode_mask = 0x0c;
  constexpr unsigned receive_interrupt_mask = 0x3000;
  constexpr unsigned reserved_receive_interrupt = 0x2000;
  constexpr unsigned configuration_mask = 0x7e;
  if ((value & mode_mask) == mode_mask ||
      (value & receive_interrupt_mask) == reserved_receive_interrupt) {
    throw Fault("Reserved DSP SHI word size/receive interrupt at X:0xffff91");
  }
  if ((value & (shi::i2c | shi::master)) != 0) {
    throw Fault("Unimplemented DSP SHI I2C/master mode at X:0xffff91");
  }
  if (reset_pending_) {
    throw Fault("DSP SHI individual-reset cycle has not elapsed at X:0xffff91");
  }
  const bool enabled = (value & shi::hen) != 0;
  const bool changed = ((value ^ control_) & configuration_mask) != 0;
  if (active_ && enabled && changed) {
    throw Fault("DSP SHI mode change requires individual reset at X:0xffff91");
  }
  if (active_ && changed && (!pins_ || pins_->selected)) {
    throw Fault("DSP SHI reset reconfiguration requires known HBUSY=0 at X:0xffff91");
  }
  constexpr unsigned request_mask = 0x180;
  if (active_ && pins_ && pins_->selected && ((value ^ control_) & request_mask) != 0) {
    throw Fault("DSP SHI HREQ configuration change requires HBUSY=0 at X:0xffff91");
  }
  if (!active_ && enabled && pins_ && pins_->selected) {
    throw Fault("Unverified DSP SHI enable with SS already asserted at X:0xffff91");
  }
  if (!active_ && enabled) { active_ = true; }
  if (active_ && !enabled) {
    // HCSR fields are writable; HEN clear takes effect after one instruction cycle.
    // Idle HM changes are permitted (7.4.6.3); data-path transients remain unqualified.
    reset_pending_ = true;
    reset_configuration_changed_ = changed;
  }
  const auto requested = value & shi::interrupt_mask;
  interrupt_enable_ |= requested;
  interrupt_delay_ = interrupt_enable_ != requested;
  if (!enabled && !active_) { clear_paths(); }
  control_ = value;
}

void DspShi::advance_instruction_cycles(const unsigned cycles) {
  if (cycles == 0) { return; }
  if (interrupt_delay_) {
    interrupt_enable_ = control_ & shi::interrupt_mask;
    interrupt_delay_ = false;
  }
  if (reset_pending_) {
    active_ = false;
    reset_pending_ = false;
    clear_paths();
  }
}

std::vector<std::uint32_t> DspShi::interrupt_vectors() const {
  std::vector<std::uint32_t> result;
  if (!active_) { return result; }
  const auto current = status();
  constexpr unsigned transmit_interrupt = 0x800;
  constexpr unsigned receive_interrupt_shift = 12;
  constexpr unsigned tx_vector = 0x40;
  constexpr unsigned tx_error_vector = 0x42;
  constexpr unsigned rx_vector = 0x44;
  constexpr unsigned rx_full_vector = 0x48;
  constexpr unsigned rx_error_vector = 0x4a;
  const auto receive = (interrupt_enable_ >> receive_interrupt_shift) & 3U;
  const bool tx_pending =
      (interrupt_enable_ & transmit_interrupt) != 0 && (current & shi::transmit_empty) != 0;
  const bool rx_error = (current & shi::overrun) != 0;
  const bool tx_error = (current & shi::underrun) != 0;
  // DSP56371UM table 7-2: receive error, transmit error, full, transmit, nonempty.
  if (receive != 0 && rx_error) { result.push_back(rx_error_vector); }
  if (tx_pending && tx_error) { result.push_back(tx_error_vector); }
  if (receive == 3 && !rx_error && (current & shi::receive_full) != 0) {
    result.push_back(rx_full_vector);
  }
  if (tx_pending && !tx_error) { result.push_back(tx_vector); }
  if (receive == 1 && !rx_error && (current & shi::receive_not_empty) != 0) {
    result.push_back(rx_vector);
  }
  return result;
}

}
