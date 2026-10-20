// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_scheduled_run.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_cycles.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_machine.hpp"

#include <cstdint>
#include <functional>
#include <optional>

namespace blofeld {
enum class McuDspTieOrder : std::uint8_t { unqualified, dsp_before_mcu, mcu_before_dsp };
enum class McuDspWaitOrder : std::uint8_t { unqualified, dsp_before_mcu };

struct McuDspScheduleInputs {
  DspScheduleInputs dsp;
  McuDspTieOrder shared_write_order = McuDspTieOrder::unqualified;
  unsigned maximum_dsp_steps_per_cycle = 0;
  McuDspWaitOrder wait_order = McuDspWaitOrder::unqualified;
  bool operator==(const McuDspScheduleInputs &) const = default;
};

class McuDspScheduleCheckpoint {
public:
  McuDspScheduleCheckpoint(const McuDspScheduleCheckpoint &) = default;
  McuDspScheduleCheckpoint &operator=(const McuDspScheduleCheckpoint &) = default;

private:
  friend class McuDspSchedule;
  explicit McuDspScheduleCheckpoint(const DspScheduleCheckpoint &dsp);
  DspScheduleCheckpoint dsp_;
  McuDspScheduleInputs inputs_;
  CpuState mcu_;
  std::uint64_t timebase_hz_ = 0;
  std::uint64_t end_ticks_ = 0;
  std::uint64_t mcu_cycles_ = 0;
  std::uint64_t mcu_idle_cycles_ = 0;
  std::uint64_t ties_ = 0;
};

// Caller-owned board observations supply shared writes. This class owns nominal
// clock ordering only, without GPIO routes, reset policy, observers or MCU steps.
// Native owners and external wiring must be restored before the schedule.
class McuDspSchedule {
public:
  using DspObserver = std::function<void(const DspScheduledStep &)>;
  McuDspSchedule(McuDspScheduleInputs inputs, McuMachine &mcu, DspMachine &dsp);
  McuDspSchedule(const McuDspSchedule &) = delete;
  McuDspSchedule &operator=(const McuDspSchedule &) = delete;
  void begin_cycle(Hc12BusCycle cycle,
                   bool shared_write,
                   const DspObserver &before_dsp = {},
                   const DspCompletionObserver &after_dsp = {},
                   const DspExternalEvents &external = {});
  void observe_write(const BusEvent &event,
                     const std::function<void()> &observe_signals,
                     const DspObserver &before_dsp = {},
                     const DspCompletionObserver &after_dsp = {},
                     const DspExternalEvents &external = {});
  void end_instruction();
  void begin_wait_period(McuWaitPeriod period,
                         const DspObserver &before_dsp = {},
                         const DspCompletionObserver &after_dsp = {},
                         const DspExternalEvents &external = {});
  void end_wait();
  [[nodiscard]] McuDspScheduleCheckpoint checkpoint() const;
  void restore(const McuDspScheduleCheckpoint &checkpoint);
  [[nodiscard]] bool faulted() const;

  [[nodiscard]] std::uint64_t end_ticks() const { return end_ticks_; }

  [[nodiscard]] std::uint64_t ties() const { return ties_; }

  [[nodiscard]] std::uint64_t completed_steps() const { return dsp_schedule_.completed_steps(); }

private:
  class Guard;
  void healthy() const;
  void consumed() const;
  void offer_period();
  void finish_boundary();
  [[nodiscard]] std::uint64_t mcu_ticks() const;
  void advance(DspScheduleBoundary boundary,
               const DspObserver &before_dsp,
               const DspCompletionObserver &after_dsp,
               const DspExternalEvents &external);
  McuDspScheduleInputs inputs_;
  McuMachine &mcu_;
  DspMachine &dsp_;
  DspScheduledRun dsp_schedule_;
  std::uint64_t timebase_hz_;
  std::uint64_t end_ticks_;
  std::uint64_t expected_mcu_cycles_;
  std::uint64_t expected_idle_cycles_;
  std::uint64_t ties_ = 0;
  Hc12BusCycle cycle_;
  std::optional<ExecutionContext> wait_context_;
  bool offered_ = false;
  bool shared_write_ = false;
  bool waiting_ = false;
  bool active_ = false;
  bool faulted_ = false;
};
}
