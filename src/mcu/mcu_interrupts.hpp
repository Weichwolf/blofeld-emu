// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "mcu/mcu_gpio.hpp"

#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

namespace blofeld {

// Normal-mode INT/MEBI priority and external pin requests. Observation is of
// resolved GPIO pads; asynchronous synchronizer delay is not yet scheduled.
class McuInterrupts {
public:
  void reset();
  void observe_pins(PinSample sample);
  [[nodiscard]] static std::string_view register_name(std::uint16_t offset);
  [[nodiscard]] std::uint8_t read(std::uint16_t offset) const;
  void write(ByteWrite write, std::uint8_t ccr);
  [[nodiscard]] std::optional<InterruptRequest> select(std::span<const std::uint16_t> pending,
                                                       std::uint8_t ccr) const;
  void acknowledge(std::uint16_t vector);

private:
  static constexpr std::uint8_t initial_priority = 0xf2;
  [[nodiscard]] static bool valid_maskable(std::uint16_t vector);
  [[nodiscard]] std::optional<std::uint16_t> choose(std::span<const std::uint16_t> pending) const;
  [[nodiscard]] bool external_irq() const;
  std::uint8_t priority_ = initial_priority;
  std::uint8_t control_ = 0x40;
  std::uint8_t test_select_ = 15;
  PinSample pins_{};
  bool edge_written_ = false;
  bool edge_pending_ = false;
  bool edge_known_ = true;
};

}
