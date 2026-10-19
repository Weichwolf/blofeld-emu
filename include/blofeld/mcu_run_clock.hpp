// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/mcu_bus.hpp"

#include <cstdint>

namespace blofeld {

enum class McuRunClockSwitch : std::uint8_t { unqualified, reference_cycle_boundary };

struct McuRunClockInputs {
  std::uint32_t oscillator_hz = 0;
  std::uint64_t initial_oscillator_phase_ticks = 0;
  McuRunClockSwitch switch_timing = McuRunClockSwitch::unqualified;
  bool operator==(const McuRunClockInputs &) const = default;
};

// Exact normal-run reference time. The oscillator rate and its initial phase
// are explicit inputs, not a measured Blofeld board profile. PHY LOCK remains
// independently observed. No WAI/STOP/BDM gates or analog switching are modeled.
// Copies retain phase; a clock/CPU/bus failure requires coordinated recovery.
class McuRunClock {
public:
  static constexpr std::uint32_t phy_hz = 60000000;
  static constexpr std::uint32_t nominal_oscillator_hz = 12000000;
  explicit McuRunClock(McuRunClockInputs inputs);
  // One completed HCS12 reference bus cycle clocks SCI/TIM once and advances
  // RTI by the oscillator edges actually elapsed under the native clock route.
  // Pad observations/transport events must be supplied separately by the caller.
  void advance_run_cycle(McuBus &bus, ExecutionContext context);

  [[nodiscard]] std::uint64_t timebase_hz() const { return timebase_hz_; }

  [[nodiscard]] std::uint64_t elapsed_ticks() const { return elapsed_ticks_; }

  [[nodiscard]] std::uint64_t bus_cycles() const { return bus_cycles_; }

  [[nodiscard]] std::uint64_t oscillator_cycles() const { return oscillator_cycles_; }

  [[nodiscard]] bool faulted() const { return faulted_; }

private:
  std::uint32_t oscillator_hz_;
  std::uint64_t timebase_hz_;
  std::uint64_t oscillator_period_;
  std::uint64_t oscillator_phase_ = 0;
  std::uint64_t elapsed_ticks_ = 0;
  std::uint64_t oscillator_cycles_ = 0;
  std::uint64_t bus_cycles_ = 0;
  bool faulted_ = false;
};

}
