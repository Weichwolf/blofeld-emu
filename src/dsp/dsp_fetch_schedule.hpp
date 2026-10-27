// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_scheduled_run.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>

namespace blofeld {
struct DspFetchEventIdentity final {};

struct DspFetchExternalEvents {
  std::shared_ptr<const DspFetchEventIdentity> identity;
  std::function<std::optional<std::uint64_t>()> next_ticks;
  std::function<void(std::uint64_t)> observe;
  DspExternalOrder order = DspExternalOrder::unqualified;
  unsigned maximum_events = 0;
};

struct DspFetchScheduleInputs {
  DspFetchRunInputs dsp;
  std::uint64_t epoch_ticks = 0;
  std::optional<std::uint64_t> initial_horizon_ticks = std::nullopt;
  bool operator==(const DspFetchScheduleInputs &) const = default;
};

struct DspFetchHorizon {
  std::uint64_t ticks = 0;
  DspScheduleBoundary native = DspScheduleBoundary::through;
  DspScheduleBoundary external = DspScheduleBoundary::through;
  bool operator==(const DspFetchHorizon &) const = default;
};

struct DspFetchAdvance {
  DspFetchHorizon horizon;
  unsigned maximum_cycles = 0;
};

struct DspFetchScheduleState {
  DspFetchHorizon horizon;
  std::shared_ptr<const DspFetchEventIdentity> external_identity;
  DspExternalOrder order = DspExternalOrder::unqualified;
  std::optional<std::uint64_t> next_external, last_external, last_native;
  std::uint64_t external_events = 0;
  bool advanced = false;
  bool operator==(const DspFetchScheduleState &) const = default;
};

class DspFetchScheduleCheckpoint {
public:
  DspFetchScheduleCheckpoint(const DspFetchScheduleCheckpoint &) = default;
  DspFetchScheduleCheckpoint &operator=(const DspFetchScheduleCheckpoint &) = default;
  ~DspFetchScheduleCheckpoint();

private:
  friend class DspFetchScheduledRun;
  struct Storage;
  explicit DspFetchScheduleCheckpoint(std::shared_ptr<const Storage> storage);
  std::shared_ptr<const Storage> storage_;
};

// Explicit reference ordering of real pipeline periods and peripheral pin events.
// The caller owns/restores its event cursor; no callback enters a checkpoint.
class DspFetchScheduledRun {
public:
  using CycleInputs = std::function<DspFetchCycleInputs()>;
  using Observer = std::function<void(const DspFetchCycleResult &)>;
  DspFetchScheduledRun(DspFetchScheduleInputs inputs, DspMachine &machine);
  DspFetchScheduledRun(DspFetchScheduleInputs inputs,
                       DspMachine &machine,
                       std::optional<DspHardwareResetInputs> initial_reset);
  DspFetchScheduledRun(const DspFetchScheduledRun &) = delete;
  DspFetchScheduledRun &operator=(const DspFetchScheduledRun &) = delete;
  unsigned advance_to(DspFetchAdvance advance,
                      const CycleInputs &inputs,
                      const Observer &after_cycle = {},
                      const DspFetchExternalEvents &external = {});
  void begin_hardware_reset(DspHardwareResetInputs inputs);
  void release_hardware_reset();
  [[nodiscard]] DspFetchScheduleCheckpoint checkpoint() const;
  void restore(const DspFetchScheduleCheckpoint &checkpoint);
  [[nodiscard]] DspFetchScheduleState state() const;
  [[nodiscard]] const DspFetchRun &run() const;
  [[nodiscard]] bool faulted() const;

private:
  class Guard;
  enum class Step : std::uint8_t { none, native, external };
  void healthy() const;
  void check_horizon(DspFetchHorizon horizon) const;
  void bind_external(const DspFetchExternalEvents &external);
  [[nodiscard]] std::optional<std::uint64_t>
  query_external(const DspFetchExternalEvents &external) const;
  [[nodiscard]] std::uint64_t absolute(std::uint64_t relative) const;
  [[nodiscard]] std::optional<std::uint64_t> native_deadline() const;
  [[nodiscard]] bool external_first(std::optional<std::uint64_t> native) const;
  [[nodiscard]] static bool
  eligible(std::uint64_t ticks, DspScheduleBoundary boundary, DspFetchHorizon horizon);
  [[nodiscard]] Step next_step(DspFetchHorizon horizon) const;
  void complete_native(const CycleInputs &inputs,
                       const Observer &after_cycle,
                       unsigned maximum_cycles,
                       unsigned &cycles);
  void reset_until(std::uint64_t ticks, bool include_recovery);
  void complete_external(const DspFetchExternalEvents &external);
  void require_reset_boundary() const;
  DspFetchScheduleInputs inputs_;
  DspMachine &machine_;
  DspFetchRun run_;
  DspFetchScheduleState state_;
  bool active_ = false;
  bool faulted_ = false;
};
} // namespace blofeld
