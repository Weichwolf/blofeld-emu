// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_bus.hpp"

#include <cstdint>

namespace blofeld {

enum class DspClockSwitch : std::uint8_t { unqualified, reference_instruction_boundary };

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

// Explicit instruction-boundary experiment. SDK cycle estimates are supplied by
// the owner; they are not calibrated DSP56371 instruction/pipeline durations.
// PLL command changes affect the next step, with no analog lock inference.
class DspRunClock {
public:
  explicit DspRunClock(DspRunClockInputs inputs);
  void begin_step(const DspBus &bus, DspRunContext context);
  void finish_step(DspBus &bus, unsigned native_cycles);
  void fail() noexcept;

  [[nodiscard]] std::uint64_t timebase_hz() const { return inputs_.timebase_hz; }

  [[nodiscard]] std::uint64_t elapsed_ticks() const { return ticks_; }

  [[nodiscard]] std::uint64_t native_cycle_units() const { return cycles_; }

  [[nodiscard]] std::uint64_t timer_clk2_edges() const { return cycles_ / 2; }

  [[nodiscard]] bool faulted() const { return faulted_; }

  [[nodiscard]] bool active() const { return active_; }

private:
  DspRunClockInputs inputs_;
  DspRunContext context_;
  std::uint64_t ticks_ = 0;
  std::uint64_t cycles_ = 0;
  std::uint64_t period_ticks_ = 0;
  bool active_ = false;
  bool faulted_ = false;
};

}
