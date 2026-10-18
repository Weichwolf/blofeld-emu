// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"

#include <cstdint>
#include <string_view>

namespace blofeld {

// One completed detector window of exactly three oscillator periods. The board
// clock source supplies the observed PHY edges, never a fabricated LOCK flag.
struct PhyClockWindow {
  unsigned edges = 0;
};

// Normal-mode USB PHY control and CRG_U lock detection. This does not generate
// analog PHY startup, CPU cycle timing, USB transactions or interrupt delivery.
class McuClock {
public:
  void reset(); // External reset; PORF is retained until software clears it.
  [[nodiscard]] static std::string_view register_name(std::uint16_t offset);
  [[nodiscard]] std::uint8_t read(std::uint16_t offset) const;
  void write(ByteWrite write);
  void observe_phy(PhyClockWindow window);
  [[nodiscard]] bool phy_running() const;
  [[nodiscard]] bool pll_selected() const;
  [[nodiscard]] bool lock_interrupt_pending() const;
  [[nodiscard]] bool realtime_interrupt_pending() const;

private:
  static constexpr std::uint16_t usb_reset_control = 0x0820;
  void write_usb(ByteWrite write);
  void write_clock_select(std::uint8_t value);
  void update_lock(bool locked);
  std::uint16_t usb_control_ = usb_reset_control;
  std::uint8_t reference_divider_ = 0;
  std::uint8_t flags_ = 0x40;
  std::uint8_t clock_select_ = 0;
  std::uint8_t interrupt_enable_ = 0;
  bool cop_wait_written_ = false;
  bool lock_known_ = true;
};

}
