// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "mcu/mcu_gpio.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace blofeld {

// Post-synchronizer input levels at the current timer time. The clock source
// must split advances at transitions; raw-pad synchronizer timing is unverified.
struct TimerInputs {
  std::uint8_t known = 0;
  std::uint8_t levels = 0;
};

class McuTimer {
public:
  void reset();
  [[nodiscard]] static std::string_view register_name(std::uint16_t offset);
  [[nodiscard]] std::uint8_t read(std::uint16_t offset);
  void write(ByteWrite write);
  void advance(unsigned bus_cycles);
  void observe_inputs(TimerInputs inputs);
  [[nodiscard]] unsigned cycles_until_tick() const;
  [[nodiscard]] bool runs_in_wait() const;
  [[nodiscard]] PinFunction pins() const; // Logical channels, before package routing.
  [[nodiscard]] std::uint8_t interrupt_channels() const;
  [[nodiscard]] bool overflow_interrupt() const;

private:
  static constexpr std::size_t channel_count = 8;
  [[nodiscard]] bool enabled() const;
  [[nodiscard]] unsigned edge_mode(unsigned channel) const;
  [[nodiscard]] unsigned output_mode(unsigned channel) const;
  [[nodiscard]] unsigned capture_mask() const;
  [[nodiscard]] unsigned output_mask() const;
  [[nodiscard]] std::uint8_t read_channel(std::uint16_t offset);
  void write_channel(ByteWrite write);
  void output_actions(unsigned matches);
  void apply_output(unsigned channel);
  void count_tick();
  void check_activation();
  std::array<std::uint8_t, 16> controls_{};
  std::array<std::uint16_t, channel_count> channels_{};
  std::uint16_t counter_ = 0;
  std::uint8_t outputs_ = 0;
  std::uint8_t outputs_known_ = 0;
  std::uint8_t inputs_ = 0;
  std::uint8_t inputs_known_ = 0;
  unsigned prescaler_phase_ = 0;
  unsigned active_prescale_ = 0;
  unsigned activation_pending_ = 0;
  bool reset_pending_ = false;
  bool clocked_since_observation_ = true;
};

}
