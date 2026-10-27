// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_fetch_schedule.hpp"
#include "dsp/dsp_machine.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/mcu_machine.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>

namespace blofeld {
struct McuDspFetchInitialReset {
  std::uint8_t mode_known = 0;
  std::uint8_t mode_levels = 0;
  std::optional<bool> pinit;
  std::uint32_t recovery_cycles = 0;
  bool operator==(const McuDspFetchInitialReset &) const = default;
};

enum class McuExternalWriteOrder : std::uint8_t {
  unqualified,
  external_before_mcu,
  mcu_before_external
};

struct McuDspFetchInputs {
  DspFetchRunInputs dsp;
  std::uint64_t epoch_ticks = 0;
  McuDspTieOrder shared_write_order = McuDspTieOrder::unqualified;
  unsigned maximum_dsp_cycles_per_period = 0;
  McuExternalWriteOrder external_write_order = McuExternalWriteOrder::unqualified;
  McuDspWaitOrder wait_order = McuDspWaitOrder::unqualified;
  McuDspResetOrder reset_order = McuDspResetOrder::unqualified;
  std::optional<McuDspFetchInitialReset> initial_reset = std::nullopt;
  bool operator==(const McuDspFetchInputs &) const = default;
};

// Trusted in-process native snapshot;not a file format or external-I/O rewind.
class McuDspFetchCheckpoint {
public:
  McuDspFetchCheckpoint(const McuDspFetchCheckpoint &) = default;
  McuDspFetchCheckpoint &operator=(const McuDspFetchCheckpoint &) = default;
  ~McuDspFetchCheckpoint();

private:
  friend class McuDspFetchSchedule;
  struct Storage;
  explicit McuDspFetchCheckpoint(std::shared_ptr<const Storage> storage);
  std::shared_ptr<const Storage> storage_;
};

// Owns nominal MCU-period/DSP-pipeline ordering, with no board wiring or host
// instruction stepping. Reset/COP order is explicit; DSP reset wiring is external.
class McuDspFetchSchedule {
public:
  using CycleInputs = std::function<DspFetchCycleInputs()>;
  using Observer = std::function<void(const DspFetchCycleResult &)>;
  McuDspFetchSchedule(McuDspFetchInputs inputs,
                      McuMachine &mcu,
                      DspMachine &dsp,
                      DspFetchExternalEvents external = {});
  // Stage fresh native targets from trusted state; no callbacks are invoked.
  // Targets may change on failure, so publish them only after successful construction.
  McuDspFetchSchedule(const McuDspFetchCheckpoint &checkpoint,
                      McuMachine &mcu,
                      DspMachine &dsp,
                      const DspFetchExternalEvents &external = {});
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
  // Owns MCU reset time only; no DSP reset pad or board route is inferred.
  void begin_system_reset(McuResetSource source, unsigned synchronization_cycles);
  void begin_reset_period(McuResetPeriod period,
                          const CycleInputs &inputs,
                          const Observer &after_dsp = {});
  void end_system_reset();
  // Exact consumed DSP-cycle or held boundaries;no asynchronous phase is inferred.
  void begin_dsp_reset(DspHardwareResetInputs inputs);
  void release_dsp_reset();
  // Capture only completed native offers;restore drops MCU observers.
  [[nodiscard]] McuDspFetchCheckpoint checkpoint() const;
  void restore(const McuDspFetchCheckpoint &checkpoint);
  [[nodiscard]] bool faulted() const;

  [[nodiscard]] std::uint64_t end_ticks() const { return end_ticks_; }

  [[nodiscard]] std::uint64_t ties() const { return ties_; }

  [[nodiscard]] DspFetchScheduleState timeline_state() const { return dsp_schedule_.state(); }

  [[nodiscard]] const DspFetchRun &run() const { return dsp_schedule_.run(); }

private:
  class Guard;
  static const McuDspFetchCheckpoint::Storage &clone_source(const McuDspFetchCheckpoint &checkpoint,
                                                            McuMachine &mcu,
                                                            const DspMachine &dsp,
                                                            const DspFetchExternalEvents &external);
  static McuMachine &clone_mcu(const McuDspFetchCheckpoint::Storage &source, McuMachine &target);
  McuDspFetchSchedule(const McuDspFetchCheckpoint::Storage &source,
                      McuMachine &mcu,
                      DspMachine &dsp,
                      DspFetchExternalEvents external);
  void healthy() const;
  void consumed() const;
  void validate_dsp_reset_observation() const;
  void prepare_shared_write(const CycleInputs &inputs, const Observer &after_dsp);
  [[nodiscard]] DspFetchExternalEvents checked_events() const;
  void verify_external_boundary() const;
  void
  advance_horizon(DspFetchHorizon horizon, const CycleInputs &inputs, const Observer &after_dsp);
  void offer_period(bool waiting = false);
  void finish_boundary();
  [[nodiscard]] std::uint64_t mcu_ticks() const;
  [[nodiscard]] std::uint64_t deadline() const;
  void advance(bool through, const CycleInputs &inputs, const Observer &after_dsp);
  const std::shared_ptr<const bool> lifetime_ = std::make_shared<const bool>(true);
  McuDspFetchInputs inputs_;
  McuMachine &mcu_;
  DspMachine &dsp_;
  DspFetchScheduledRun dsp_schedule_;
  const DspFetchExternalEvents external_;
  const std::uint64_t initial_mcu_ticks_;
  std::uint64_t end_ticks_;
  std::uint64_t expected_cycles_;
  std::uint64_t expected_epoch_;
  std::uint64_t expected_idle_;
  std::uint64_t expected_reset_edges_;
  std::uint64_t ties_ = 0;
  unsigned period_cycles_ = 0;
  Hc12BusCycle cycle_;
  std::optional<ExecutionContext> wait_context_;
  std::optional<ExecutionContext> reset_context_;
  bool offered_ = false;
  bool shared_write_ = false;
  bool shared_write_observed_ = false;
  bool waiting_ = false;
  bool resetting_ = false;
  bool cop_deadline_ = false;
  bool active_ = false;
  bool faulted_ = false;
  bool observing_signals_ = false;
};
}
