// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_peripheral_calendar.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>

namespace blofeld {
enum class DspPeerOrder : std::uint8_t { unqualified, dsp_first, peer_first, reject_tie };

enum class McuWriteOrder : std::uint8_t { unqualified, before_mcu, after_mcu, reject_tie };

struct McuDspRuntimeInputs {
  OwnDspAutomaticTiming timing = OwnDspAutomaticTiming::unqualified;
  std::uint64_t epoch_ticks = 0;
  unsigned maximum_cycles_per_period = 0;
  McuWriteOrder writes = McuWriteOrder::unqualified;
  McuWriteOrder external_writes = McuWriteOrder::unqualified;
  bool wait_dsp_first = false;
  bool reset_dsp_first = false;
  std::optional<OwnDspResetInputs> initial_reset;
};

struct DspRuntimeEvents {
  std::shared_ptr<const bool> identity;
  std::function<std::optional<std::uint64_t>()> next_ticks;
  std::function<void(std::uint64_t)> observe;
  DspPeerOrder order = DspPeerOrder::unqualified;
  unsigned maximum_events_per_period = 0;
};

// Board-owned causal deadlines are separate from the pinned input cursor.
// The supported combined order is inherited input, peripheral, then DSP.
struct DspRuntimePeripheralEvents {
  std::shared_ptr<const PeripheralEventIdentity> identity;
  std::function<std::optional<std::uint64_t>()> next_ticks;
  std::function<void(std::uint64_t)> observe;
  DspPeerOrder order = DspPeerOrder::unqualified;
  unsigned maximum_events_per_period = 0;
};

struct DspRuntimeCycle {
  std::uint64_t start_ticks;
  std::uint64_t completion_ticks;
  std::uint64_t cycle;
  OwnDspFetchCycleResult result;
};

class McuDspRuntimeCheckpoint {
public:
  McuDspRuntimeCheckpoint() = default;
  ~McuDspRuntimeCheckpoint();
  McuDspRuntimeCheckpoint(const McuDspRuntimeCheckpoint &) = default;
  McuDspRuntimeCheckpoint &operator=(const McuDspRuntimeCheckpoint &) = default;

private:
  friend class McuDspRuntime;
  struct Storage;
  explicit McuDspRuntimeCheckpoint(std::shared_ptr<const Storage> storage);
  std::shared_ptr<const Storage> storage_;
};

// Canonical MCU-period coordinator for dsp56300; board wiring and host I/O stay
// with the caller. No legacy DSP executor, instruction estimates or CPU shim.
class McuDspRuntime {
public:
  using CycleInputs = std::function<DspDmaInputs()>;
  using Observer = std::function<void(const DspRuntimeCycle &)>;
  McuDspRuntime(McuDspRuntimeInputs inputs,
                McuMachine &mcu,
                OwnDspMachine &dsp,
                DspRuntimeEvents external = {},
                DspRuntimePeripheralEvents peripheral = {});
  McuDspRuntime(const McuDspRuntimeCheckpoint &checkpoint,
                McuMachine &mcu,
                OwnDspMachine &dsp,
                DspRuntimeEvents external = {},
                DspRuntimePeripheralEvents peripheral = {});
  McuDspRuntime(const McuDspRuntime &) = delete;
  McuDspRuntime &operator=(const McuDspRuntime &) = delete;
  void begin_cycle(Hc12BusCycle cycle,
                   bool shared_write,
                   const CycleInputs &inputs,
                   const Observer &observer = {});
  void observe_write(const BusEvent &event,
                     const std::function<void()> &observe,
                     const CycleInputs &inputs,
                     const Observer &observer = {});
  void end_instruction();
  void
  begin_wait_period(McuWaitPeriod period, const CycleInputs &inputs, const Observer &observer = {});
  // Also closes a zero-period slice cancelled by an IRQ before the peer hook.
  void end_wait();
  void begin_reset_period(McuResetPeriod period,
                          const CycleInputs &inputs,
                          const Observer &observer = {});
  void end_system_reset();
  void begin_system_reset(McuResetSource source, unsigned synchronization_cycles);
  // Caller-owned reset wiring may act at a completed operation or inside the
  // consumed MCU shared-write signal observation. Running DSP time must match.
  void begin_dsp_reset(OwnDspResetInputs inputs);
  void release_dsp_reset();
  [[nodiscard]] McuDspRuntimeCheckpoint checkpoint() const;
  [[nodiscard]] bool faulted() const;

  [[nodiscard]] std::uint64_t end_ticks() const { return state_.end_ticks; }

  [[nodiscard]] std::uint64_t ties() const { return state_.ties; }

  [[nodiscard]] std::uint64_t completed_cycles() const;
  [[nodiscard]] std::optional<PeripheralCalendarState> peripheral_state() const;

private:
  friend class McuDspRuntimeCheckpoint;
  class Guard;

  struct State {
    std::uint64_t end_ticks = 0;
    std::uint64_t cycles = 0;
    std::uint64_t epoch = 0;
    std::uint64_t idle = 0;
    std::uint64_t reset_edges = 0;
    std::uint64_t ties = 0;
    std::optional<std::uint64_t> last_external;
    std::optional<std::uint64_t> pending_external;
    std::optional<std::uint64_t> last_native;
    std::optional<std::uint64_t> closed_external;
    bool resetting = false;
    bool waiting = false;
    bool offered = false;
    bool shared_write = false;
    bool cop_deadline = false;
  };

  struct InputEvent {
    std::optional<std::uint64_t> ticks;
    DspPeerOrder order;
    bool peripheral;
  };

  [[nodiscard]] std::uint64_t mcu_ticks() const;
  [[nodiscard]] std::uint64_t deadline() const;
  [[nodiscard]] std::optional<std::uint64_t> query_external() const;
  [[nodiscard]] std::optional<std::uint64_t> next_external();
  void check_peripheral_contract() const;
  [[nodiscard]] std::optional<std::uint64_t> query_peripheral() const;
  [[nodiscard]] std::optional<std::uint64_t> next_peripheral() const;
  [[nodiscard]] std::optional<std::uint64_t> next_input();
  [[nodiscard]] InputEvent next_event();
  [[nodiscard]] DspPeerOrder input_order(std::uint64_t ticks) const;
  void count_input_event();
  void peripheral_cause(PeripheralCalendarCause cause, std::uint64_t ticks);
  void finish_peripheral(bool native_through, bool events_through);
  void observe_peripheral(std::uint64_t ticks);
  void healthy() const;
  void consumed() const;
  void validate_dsp_reset_observation() const;
  void finish_boundary();
  void offer_period(bool waiting);
  void advance(bool native_through,
               bool external_through,
               const CycleInputs &inputs,
               const Observer &observer);
  void advance_native(std::uint64_t ticks, const CycleInputs &inputs, const Observer &observer);
  void advance_reset(std::uint64_t ticks, bool through);
  void observe_external(std::uint64_t ticks);
  void prepare_write(const CycleInputs &inputs, const Observer &observer);
  [[noreturn]] void reject_native(const dsp56300::Fault &fault) const;
  [[nodiscard]] static const McuDspRuntimeCheckpoint::Storage &
  saved_state(const McuDspRuntimeCheckpoint &checkpoint,
              const DspRuntimeEvents &external,
              const DspRuntimePeripheralEvents &peripheral);
  McuDspRuntimeInputs inputs_;
  McuMachine &mcu_;
  OwnDspMachine &dsp_;
  std::unique_ptr<OwnDspTimeline> timeline_;
  DspRuntimeEvents external_;
  DspRuntimePeripheralEvents peripheral_events_;
  std::unique_ptr<PeripheralCalendar> peripheral_;
  State state_;
  Hc12BusCycle cycle_;
  unsigned period_cycles_ = 0;
  unsigned period_events_ = 0;
  bool active_ = false;
  bool faulted_ = false;
  bool observing_signals_ = false;
};
}
