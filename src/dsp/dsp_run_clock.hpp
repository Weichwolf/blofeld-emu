// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_bus.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {

enum class DspClockSwitch : std::uint8_t {
  unqualified,
  reference_instruction_boundary,
  reference_cycle_boundary
};

struct DspRunClockInputs {
  std::uint32_t extal_hz = 0;
  std::uint64_t timebase_hz = 0;
  DspClockSwitch switch_timing = DspClockSwitch::unqualified;
  bool operator==(const DspRunClockInputs &) const = default;
};

struct DspRunContext {
  std::uint32_t pc = 0;
  std::uint32_t omr = 0;
};

enum class DspHardwareResetPhase : std::uint8_t { asserted, recovering, complete };

struct DspHardwareResetState {
  DspHardwareResetPhase phase = DspHardwareResetPhase::asserted;
  std::uint64_t asserted_extal_cycles = 0;
  std::uint64_t recovery_extal_cycles = 0;
  std::uint32_t required_recovery_cycles = 0;
  std::uint64_t phase_ticks = 0;
  bool operator==(const DspHardwareResetState &) const = default;
};

struct DspClockUnits {
  unsigned native = 0;
  unsigned interlocks = 0;
};

// Explicit instruction-boundary experiment. SDK cycle estimates are supplied by
// the owner; they are not calibrated DSP56371 instruction/pipeline durations.
// PLL command changes affect the next explicitly selected instruction or
// single-cycle boundary, with no analog lock inference.
class DspRunClock {
public:
  explicit DspRunClock(DspRunClockInputs inputs);
  void begin_step(const DspBus &bus, DspRunContext context);
  void finish_step(DspBus &bus, unsigned native_cycles);
  void finish_step(DspBus &bus, DspClockUnits units);
  // Nominal zero-duration reset boundary; preserve completed lifetime time/units.
  void restart_native_epoch();
  [[nodiscard]] std::uint64_t next_step_ticks(const DspBus &bus, unsigned native_cycles) const;
  [[nodiscard]] std::uint64_t next_step_ticks(const DspBus &bus, DspClockUnits units) const;
  // Fixed mode5, stable PLL-disabled EXTAL reference experiment; no analog lock.
  void begin_hardware_reset(std::uint32_t recovery_cycles);
  std::uint64_t advance_hardware_reset(std::uint64_t maximum_extal_cycles);
  // Exact supplied time; phase_ticks counts a partial current hold/recovery period.
  // Recovery starts at release, without claiming a physical oscillator/pipeline phase.
  std::uint64_t advance_hardware_reset_ticks(std::uint64_t maximum_ticks);
  void release_hardware_reset();
  [[nodiscard]] bool hardware_reset_active() const;
  [[nodiscard]] std::optional<std::uint64_t> hardware_reset_remaining_ticks() const;

  [[nodiscard]] std::uint64_t extal_period_ticks() const {
    return inputs_.timebase_hz / inputs_.extal_hz;
  }

  [[nodiscard]] std::optional<DspHardwareResetState> hardware_reset_state() const { return reset_; }

  [[nodiscard]] std::uint64_t hardware_reset_ticks() const { return reset_ticks_; }

  [[nodiscard]] std::uint64_t
  next_step_ticks(const DspBus &bus, unsigned native_cycles, unsigned interlock_cycles) const;
  void finish_step(DspBus &bus, unsigned native_cycles, unsigned interlock_cycles);
  void fail() noexcept;

  [[nodiscard]] std::uint64_t timebase_hz() const { return inputs_.timebase_hz; }

  [[nodiscard]] std::uint64_t elapsed_ticks() const { return ticks_; }

  [[nodiscard]] std::uint64_t native_cycle_units() const { return cycles_; }

  [[nodiscard]] std::uint64_t elapsed_cycle_units() const { return cycles_ + interlocks_; }

  [[nodiscard]] std::uint64_t interlock_cycle_units() const { return interlocks_; }

  [[nodiscard]] std::uint64_t native_epoch() const { return epoch_; }

  [[nodiscard]] std::uint64_t epoch_native_cycle_units() const { return cycles_ - epoch_start_; }

  [[nodiscard]] std::uint64_t epoch_interlock_cycle_units() const {
    return interlocks_ - epoch_interlock_start_;
  }

  [[nodiscard]] std::uint64_t timer_clk2_edges() const { return clk2_edges_; }

  [[nodiscard]] bool faulted() const { return faulted_; }

  [[nodiscard]] bool active() const { return active_; }

private:
  [[nodiscard]] std::uint64_t period_ticks(const DspBus &bus) const;
  [[nodiscard]] std::uint64_t duration_ticks(DspClockUnits units, std::uint64_t period) const;
  DspRunClockInputs inputs_;
  DspRunContext context_;
  std::uint64_t ticks_ = 0;
  std::uint64_t cycles_ = 0;
  std::uint64_t interlocks_ = 0;
  std::uint64_t epoch_start_ = 0;
  std::uint64_t epoch_interlock_start_ = 0;
  std::uint64_t epoch_ = 0;
  std::uint64_t clk2_edges_ = 0;
  std::uint64_t period_ticks_ = 0;
  std::optional<DspHardwareResetState> reset_;
  std::uint64_t reset_ticks_ = 0;
  bool active_ = false;
  bool faulted_ = false;
};

}
