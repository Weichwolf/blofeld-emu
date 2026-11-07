// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_interrupt_admission.hpp"
#include "dsp/dsp_io.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp56300/core.hpp"

#include <cstdint>
#include <limits>
#include <memory>
#include <optional>

namespace blofeld {
struct DspResetInputs {
  std::uint8_t mode_known = 0;
  std::uint8_t mode_levels = 0;
  std::optional<bool> pinit;
  std::uint32_t recovery_cycles = 0;
};

// Instrument memory and peripheral ownership around the independent native Core.
// Instructions execute atomically; board events and audio pacing stay outside it.
class DspMachine {
public:
  explicit DspMachine(DspRunClockInputs clock);
  DspMachine(const DspMachine &) = delete;
  DspMachine &operator=(const DspMachine &) = delete;

  [[nodiscard]] DspBus &bus() noexcept { return bus_; }

  [[nodiscard]] const dsp56300::Core &cpu() const noexcept { return cpu_; }

  [[nodiscard]] const DspRunClock &clock() const noexcept { return clock_; }

  [[nodiscard]] const DspIo &io() const noexcept { return io_; }

  [[nodiscard]] bool interrupt_pending() const noexcept { return interrupts_.pending(); }

  void begin_hardware_reset(DspResetInputs inputs);
  std::uint64_t advance_hardware_reset_ticks(std::uint64_t maximum_ticks);
  void release_hardware_reset();
  [[nodiscard]] dsp56300::RunResult
  run(std::uint32_t instructions = std::numeric_limits<std::uint32_t>::max());
  [[nodiscard]] dsp56300::RunResult run_unclocked();
  [[nodiscard]] bool admit_interrupt(dsp56300::InstructionWords foreground);

private:
  friend class DspExecution;
  void require_running() const;
  void require_unclocked(const char *operation) const;
  void perform_hardware_reset(DspResetInputs inputs);
  void check_hardware_reset(DspResetInputs inputs) const;
  void check_reset_release() const;
  [[nodiscard]] dsp56300::RunResult execute(std::uint32_t instructions,
                                            DspPeripheralObserver observer = {});
  [[nodiscard]] dsp56300::RunResult execute_until(std::uint64_t base_cycle_deadline,
                                                  DspPeripheralObserver observer);
  DspBus bus_;
  DspIo io_;
  dsp56300::Core cpu_;
  DspInterruptAdmission interrupts_;
  DspRunClock clock_;
  const DspRunClockInputs clock_inputs_;
  std::weak_ptr<const bool> clocked_owner_;
  bool active_ = false;
};
}
