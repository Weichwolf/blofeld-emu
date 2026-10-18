// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/mcu_clock.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/hc12_registers.hpp"

#include <cstdint>
#include <string_view>

namespace blofeld {
namespace {
constexpr unsigned usb_enable = 0x8000;
constexpr unsigned usb_clock_enable = 0x4000;
constexpr unsigned usb_commands = 0x3200;
constexpr unsigned usb_phy_reset = 0x0400;
constexpr unsigned usb_suspend = 0x0800;
constexpr unsigned usb_descriptor = 0x0010;
constexpr unsigned usb_low_configuration = 0x002f;
constexpr unsigned usb_reset = 0x0820;
constexpr unsigned phy_qualified_edges = 15;
constexpr unsigned power_on_flag = 0x40;
constexpr unsigned lock_flag = 0x10;
constexpr unsigned locked_status = 8;
constexpr unsigned clearable_flags = 0xd0;
constexpr unsigned pll_select = 0x80;
constexpr unsigned clock_select_mask = 0xb7;
constexpr unsigned cop_wait = 1;
constexpr unsigned reference_divider_mask = 15;
constexpr unsigned interrupt_enable_mask = 0x90;
constexpr unsigned rti_control_mask = 0x7f;
constexpr unsigned rti_flag = 0x80;
constexpr unsigned rti_modulus_mask = 15;
constexpr unsigned byte_mask = 0xff;
}

void McuClock::reset() {
  usb_control_ = usb_reset;
  reference_divider_ = 0;
  flags_ = static_cast<std::uint8_t>(flags_ & power_on_flag);
  clock_select_ = 0;
  interrupt_enable_ = 0;
  rti_control_ = 0;
  rti_phase_ = 0;
  cop_wait_written_ = false;
  lock_known_ = true;
}

std::string_view McuClock::register_name(const std::uint16_t offset) {
  switch (offset) {
    case hc12::refdv: return "REFDV";
    case hc12::crg_test_flags: return "CTFLG normal mode";
    case hc12::crg_flags: return "CRGFLG";
    case hc12::crg_interrupts: return "CRGINT";
    case hc12::clock_select: return "CLKSEL";
    case hc12::rti_control: return "RTICTL";
    case hc12::usb_control: return "UMCR high";
    case hc12::usb_control + 1: return "UMCR low";
    default: return {};
  }
}

bool McuClock::phy_running() const {
  // SPHY controls the PHY clock. MCE gates the USB controller clock and is a
  // write prerequisite, not a documented substitute for suspending the PHY.
  return (usb_control_ & usb_suspend) == 0;
}

bool McuClock::pll_selected() const {
  if ((clock_select_ & pll_select) == 0) { return false; }
  if (!lock_known_) { throw Fault("Unknown USB PHY clock qualification for PLL selection"); }
  return (flags_ & locked_status) != 0;
}

std::uint8_t McuClock::read(const std::uint16_t offset) const {
  switch (offset) {
    case hc12::refdv: return reference_divider_;
    case hc12::crg_flags:
      if (!lock_known_) {
        throw Fault("Unknown USB PHY startup/clock edges at CRGFLG; clock source is required");
      }
      return flags_;
    case hc12::clock_select: return clock_select_;
    case hc12::crg_interrupts: return interrupt_enable_;
    case hc12::rti_control: return rti_control_;
    case hc12::usb_control: return static_cast<std::uint8_t>(usb_control_ >> 8U);
    case hc12::usb_control + 1: return static_cast<std::uint8_t>(usb_control_ & byte_mask);
    case hc12::crg_test_flags: return 0;
    default: throw Fault("Unknown MCU clock register read at " + hex(offset));
  }
}

void McuClock::update_lock(const bool locked) {
  const bool previous = (flags_ & locked_status) != 0;
  if (previous != locked) { flags_ = static_cast<std::uint8_t>(flags_ | lock_flag); }
  flags_ = static_cast<std::uint8_t>(locked ? flags_ | locked_status : flags_ & ~locked_status);
  lock_known_ = true;
}

void McuClock::observe_phy(const PhyClockWindow window) {
  if (!phy_running() && window.edges != 0) {
    throw Fault("PHY edges supplied while USB PHY is suspended");
  }
  update_lock(phy_running() && window.edges == phy_qualified_edges);
}

void McuClock::write_usb(const ByteWrite write) {
  const unsigned previous = usb_control_;
  const bool high_byte = write.address == hc12::usb_control;
  const unsigned requested =
      high_byte ? (static_cast<unsigned>(write.value) << 8U) | (previous & byte_mask)
                : (previous & ~byte_mask) | write.value;
  unsigned next = previous;
  if ((previous & usb_enable) == 0) {
    next = (next & ~usb_clock_enable) | (requested & usb_clock_enable);
  }
  if ((next & usb_clock_enable) != 0) {
    if ((previous & usb_enable) == 0 && (requested & usb_commands) != 0) {
      throw Fault("Unimplemented USB reset or remote-resume command");
    }
    next = (next & ~usb_descriptor) | (requested & usb_descriptor);
    if ((previous & usb_enable) == 0) {
      next = (next & ~usb_low_configuration) | (requested & usb_low_configuration);
    }
    if ((clock_select_ & pll_select) == 0) {
      if ((previous & usb_enable) == 0 && (requested & usb_phy_reset) != 0) {
        throw Fault("Unimplemented USB PHY reset command");
      }
      next = (next & ~usb_suspend) | (requested & usb_suspend);
    }
  }
  next = (next & ~usb_enable) | (requested & usb_enable);
  usb_control_ = static_cast<std::uint16_t>(next);
  if ((previous & usb_suspend) != (next & usb_suspend)) {
    if (phy_running()) {
      lock_known_ = false;
    } else {
      update_lock(false);
    }
  }
}

void McuClock::write_clock_select(const std::uint8_t value) {
  auto next = static_cast<unsigned>(value) & clock_select_mask;
  if (cop_wait_written_) { next = (next & ~cop_wait) | (clock_select_ & cop_wait); }
  clock_select_ = static_cast<std::uint8_t>(next);
  cop_wait_written_ = true;
}

void McuClock::write(const ByteWrite write) {
  switch (write.address) {
    case hc12::refdv:
      if ((clock_select_ & pll_select) == 0) {
        reference_divider_ = static_cast<std::uint8_t>(write.value & reference_divider_mask);
      }
      return;
    case hc12::crg_flags:
      flags_ = static_cast<std::uint8_t>(flags_ & ~(write.value & clearable_flags));
      return;
    case hc12::clock_select: write_clock_select(write.value); return;
    case hc12::usb_control:
    case hc12::usb_control + 1: write_usb(write); return;
    case hc12::crg_test_flags: return; // Documented unimplemented writes in normal mode.
    case hc12::crg_interrupts:
      interrupt_enable_ = static_cast<std::uint8_t>(write.value & interrupt_enable_mask);
      return;
    case hc12::rti_control:
      rti_control_ = static_cast<std::uint8_t>(write.value & rti_control_mask);
      rti_phase_ = 0;
      return;
    default: throw Fault("Unknown MCU clock register write at " + hex(write.address));
  }
}

bool McuClock::lock_interrupt_pending() const {
  if ((interrupt_enable_ & lock_flag) == 0) { return false; }
  if ((flags_ & lock_flag) != 0) { return true; }
  if (!lock_known_) { throw Fault("Unknown PHY qualification with PLL interrupt enabled"); }
  return false;
}

bool McuClock::realtime_interrupt_pending() const {
  return (flags_ & interrupt_enable_ & rti_flag) != 0;
}

unsigned McuClock::rti_period() const {
  const unsigned prescale = rti_control_ >> 4U;
  if (prescale == 0) { return 0; }
  // S12CRGUV1 table 3-2: OSCCLK / (2^(9+RTR[6:4]) * (1+RTR[3:0])).
  return (1U << (9U + prescale)) * ((rti_control_ & rti_modulus_mask) + 1U);
}

unsigned McuClock::oscillator_cycles_until_rti() const {
  const auto period = rti_period();
  return period == 0 ? 0 : period - rti_phase_;
}

void McuClock::advance_oscillator(const unsigned cycles) {
  const auto period = rti_period();
  if (period == 0 || cycles == 0) { return; }
  const auto total = static_cast<std::uint64_t>(rti_phase_) + cycles;
  if (total >= period) { flags_ = static_cast<std::uint8_t>(flags_ | rti_flag); }
  rti_phase_ = static_cast<unsigned>(total % period);
}

}
