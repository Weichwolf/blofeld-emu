// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/mcu_machine.hpp"

#include <cstdint>
#include <functional>
#include <optional>

namespace blofeld {
struct McuDspFetchInputs {
  DspFetchRunInputs dsp;
  std::uint64_t epoch_ticks = 0;
  McuDspTieOrder shared_write_order = McuDspTieOrder::unqualified;
  unsigned maximum_dsp_cycles_per_period = 0;
  McuDspWaitOrder wait_order = McuDspWaitOrder::unqualified;
};

// Owns nominal MCU-period/DSP-pipeline ordering, with no board wiring or host
// instruction stepping. COP deadlines and reset require further contracts.
class McuDspFetchSchedule {
public:
  using CycleInputs = std::function<DspFetchCycleInputs()>;
  using Observer = std::function<void(const DspFetchCycleResult &)>;
  McuDspFetchSchedule(McuDspFetchInputs inputs, McuMachine &mcu, DspMachine &dsp);
  McuDspFetchSchedule(const McuDspFetchSchedule &) = delete;
  McuDspFetchSchedule &operator=(const McuDspFetchSchedule &) = delete;
  void begin_cycle(Hc12BusCycle cycle,
                   bool shared_write,
                   const CycleInputs &inputs,
                   const Observer &after_dsp = {});
  void observe_write(const BusEvent &event,
                     const std::function<void()> &observe_signals,
                     const CycleInputs &inputs,
                     const Observer &after_dsp = {});
  void end_instruction();
  void begin_wait_period(McuWaitPeriod period,
                         const CycleInputs &inputs,
                         const Observer &after_dsp = {});
  void end_wait();
  [[nodiscard]] bool faulted() const;

  [[nodiscard]] std::uint64_t end_ticks() const { return end_ticks_; }

  [[nodiscard]] std::uint64_t ties() const { return ties_; }

  [[nodiscard]] const DspFetchRun &run() const { return run_; }

private:
  class Guard;
  void healthy() const;
  void consumed() const;
  void offer_period(bool waiting = false);
  void finish_boundary();
  [[nodiscard]] std::uint64_t mcu_ticks() const;
  [[nodiscard]] std::uint64_t deadline() const;
  void advance(bool through, const CycleInputs &inputs, const Observer &after_dsp);
  McuDspFetchInputs inputs_;
  McuMachine &mcu_;
  DspMachine &dsp_;
  DspFetchRun run_;
  std::uint64_t end_ticks_;
  std::uint64_t expected_cycles_;
  std::uint64_t expected_epoch_;
  std::uint64_t expected_idle_;
  std::uint64_t ties_ = 0;
  unsigned period_cycles_ = 0;
  Hc12BusCycle cycle_;
  std::optional<ExecutionContext> wait_context_;
  bool offered_ = false;
  bool shared_write_ = false;
  bool waiting_ = false;
  bool active_ = false;
  bool faulted_ = false;
};
}
