// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "mcu/mcu_bus.hpp"

#include <cstdint>

namespace blofeld {

enum class McuRunClockSwitch : std::uint8_t { unqualified, reference_cycle_boundary };

struct McuRunClockInputs {
  std::uint32_t oscillator_hz = 0;
  std::uint64_t initial_oscillator_phase_ticks = 0;
  McuRunClockSwitch switch_timing = McuRunClockSwitch::unqualified;
  bool operator==(const McuRunClockInputs &) const = default;
};

struct McuRunPeriod {
  std::uint64_t duration_ticks = 0;
  bool cop_deadline = false;
};

// Exact normal-run reference time. The oscillator rate and its initial phase
// are explicit inputs, not a measured Blofeld board profile. PHY LOCK remains
// independently observed. WAI honors SCI/TIM gates with continuing system clocks;
// CRG wait shutdown, STOP/BDM gates and analog switching remain unqualified.
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
  // One enabled IPBus period during native WAI, honoring SCI/TIM wait gates.
  // CRG clock/PLL shutdown modes remain explicitly unqualified.
  void advance_wait_cycle(McuBus &bus, ExecutionContext context);
  // Pure duration offer for a board scheduler; no clock, input or peripheral advance.
  [[nodiscard]] std::uint64_t next_cycle_ticks(const McuBus &bus) const;
  // A COP deadline preempts the IPBus period, including a timeout at its end.
  [[nodiscard]] McuRunPeriod next_period(const McuBus &bus, bool waiting = false) const;
  [[nodiscard]] std::uint64_t next_reset_edge_ticks() const;

  [[nodiscard]] std::uint64_t timebase_hz() const { return timebase_hz_; }

  [[nodiscard]] std::uint32_t oscillator_hz() const { return oscillator_hz_; }

  [[nodiscard]] std::uint64_t elapsed_ticks() const { return elapsed_ticks_; }

  // Completed IPBus periods, including explicitly advanced native WAI time.
  [[nodiscard]] std::uint64_t bus_cycles() const { return bus_cycles_; }

  [[nodiscard]] std::uint64_t oscillator_cycles() const { return oscillator_cycles_; }

  [[nodiscard]] std::uint64_t native_epoch() const { return epoch_; }

  [[nodiscard]] std::uint64_t reset_sysclk_edges() const { return reset_edges_; }

  [[nodiscard]] bool faulted() const { return faulted_; }

private:
  friend class McuMachine;
  void begin_reset_epoch();
  void advance_reset_edge();
  void finish_reset();
  void advance_to_cop_deadline(McuBus &bus, unsigned edges);
  void advance_cycle(McuBus &bus, ExecutionContext context, bool waiting);
  std::uint32_t oscillator_hz_;
  std::uint64_t timebase_hz_;
  std::uint64_t oscillator_period_;
  std::uint64_t oscillator_phase_ = 0;
  std::uint64_t elapsed_ticks_ = 0;
  std::uint64_t oscillator_cycles_ = 0;
  std::uint64_t bus_cycles_ = 0;
  std::uint64_t epoch_ = 1;
  std::uint64_t reset_edges_ = 0;
  bool resetting_ = false;
  bool faulted_ = false;
};

}
