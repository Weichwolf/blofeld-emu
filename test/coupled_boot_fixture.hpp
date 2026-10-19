// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_shi.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_machine.hpp"

#include <cstdint>
#include <iosfwd>
#include <string>

namespace blofeld::test::coupled {
inline constexpr unsigned phy_poll = 0x8049;
inline constexpr unsigned ready_poll = 0xb98d;
inline constexpr unsigned sender_end = 0xb9aa;
inline constexpr unsigned port_b = 1;
inline constexpr unsigned select_n = 2;
inline constexpr unsigned clock_pin = 4;
inline constexpr unsigned mosi_pin = 16;
inline constexpr unsigned serial_pins = select_n | clock_pin | mosi_pin;
inline constexpr unsigned other_b_inputs = 0xe8;
inline constexpr unsigned port_u = 0x278;
inline constexpr unsigned bootstrap_base = 0xff0000;
inline constexpr unsigned frame_offset = 0x7875;
inline constexpr unsigned stage_words = 94;
inline constexpr unsigned stage_base = 0x6a;
inline constexpr unsigned clear_pc = 0x72;
inline constexpr unsigned clear_count = 0x20000;
inline constexpr unsigned long_ram_end = 0x2000;
inline constexpr unsigned hcsr = 0xffff91;
inline constexpr unsigned hrx = 0xffff94;
inline constexpr unsigned expected_port_writes = 7275;

enum class PadOrder : std::uint8_t { unresolved, clock_before_select, select_before_clock };

struct Experiment {
  PadOrder order = PadOrder::unresolved;
  unsigned dsp_steps_per_mcu_instruction = 1;
  bool break_mosi = false;
};

void require(bool condition, const std::string &message);

// Controlled, one-way laboratory wiring. This is not a timed Blofeld profile.
// External pin order is explicit; raw GPIO observations remain unchanged.
class Link {
public:
  Link(McuMachine &mcu, DspBus &bus, DspCore &dsp, std::ostream &trace, Experiment experiment);
  void start();
  void observe(const BusEvent &event);
  void after_mcu_instruction();
  void step_dsp();
  void verify_transmission() const;

  [[nodiscard]] unsigned dsp_steps() const { return dsp_steps_; }

private:
  [[nodiscard]] SpiInputs sample() const;
  void deliver(SpiInputs pins);
  void ordered_transition(SpiInputs pins);
  McuMachine &mcu_;
  DspBus &bus_;
  DspCore &dsp_;
  std::ostream &trace_;
  Experiment experiment_;
  SpiInputs previous_;
  unsigned dsp_steps_ = 0;
  unsigned writes_ = 0;
  unsigned ordered_pairs_ = 0;
  unsigned completed_frames_ = 0;
  bool injected_ = false;
};
}
