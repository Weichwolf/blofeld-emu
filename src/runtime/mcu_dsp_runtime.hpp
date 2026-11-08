// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_execution.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_peripheral_calendar.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>

namespace blofeld {
enum class DspPeerOrder : std::uint8_t { unqualified, dsp_first, peer_first, reject_tie };

struct McuDspRuntimeInputs {
  std::uint64_t epoch_ticks = 0;
  bool reset_dsp_first = false;
  std::optional<DspResetInputs> initial_reset;
};

struct DspRuntimeEvents {
  std::shared_ptr<const bool> identity;
  std::function<std::optional<std::uint64_t>()> next_ticks;
  std::function<void(std::uint64_t)> observe;
  DspPeerOrder order = DspPeerOrder::unqualified;
};

// Instrument-owned causal deadlines are separate from the pinned input cursor.
// The supported combined order is inherited input, peripheral, then DSP.
struct DspRuntimePeripheralEvents {
  std::shared_ptr<const PeripheralEventIdentity> identity;
  std::function<std::optional<std::uint64_t>()> next_ticks;
  std::function<void(std::uint64_t)> observe;
  DspPeerOrder order = DspPeerOrder::unqualified;
};

enum class DspRuntimePhase : std::uint8_t { region, preparing, peripheral };

struct DspRuntimeCycle {
  std::uint64_t start_ticks;
  std::uint64_t completion_ticks;
  std::uint64_t cycle;
  std::optional<std::uint32_t> native_pc;
  DspRuntimePhase phase = DspRuntimePhase::region;
  std::optional<dsp56300::RunResult> region = std::nullopt;
  bool cycle_known = true;
};

class McuDspRuntime {
public:
  using CycleInputs = std::function<DspDmaInputs()>;
  using Observer = std::function<void(const DspRuntimeCycle &)>;
  McuDspRuntime(McuDspRuntimeInputs inputs,
                McuMachine &mcu,
                DspMachine &dsp,
                DspRuntimeEvents external = {},
                DspRuntimePeripheralEvents peripheral = {});
  McuDspRuntime(const McuDspRuntime &) = delete;
  McuDspRuntime &operator=(const McuDspRuntime &) = delete;
  // Advance the peer to the MCU's completed native device-time boundary.
  void synchronize(const CycleInputs &inputs, const Observer &observer = {});
  void observe_write(const BusEvent &event,
                     const std::function<void()> &observe,
                     const CycleInputs &inputs,
                     const Observer &observer = {});
  void end_instruction();
  void begin_reset_period(McuResetPeriod period,
                          const CycleInputs &inputs,
                          const Observer &observer = {});
  void end_system_reset();
  void begin_system_reset(McuResetSource source, unsigned synchronization_cycles);
  // Caller-owned reset wiring may act at a completed operation or inside the
  // consumed MCU shared-write signal observation. Running DSP time must match.
  void begin_dsp_reset(DspResetInputs inputs);
  void release_dsp_reset();
  [[nodiscard]] bool faulted() const;

  [[nodiscard]] std::uint64_t end_ticks() const { return state_.end_ticks; }

  [[nodiscard]] std::uint64_t completed_cycles() const;

private:
  class Guard;

  struct State {
    std::uint64_t end_ticks = 0;
    std::uint64_t cycles = 0;
    std::uint64_t epoch = 0;
    std::uint64_t idle = 0;
    std::uint64_t reset_edges = 0;
    std::optional<std::uint64_t> last_external;
    std::optional<std::uint64_t> pending_external;
    std::optional<std::uint64_t> last_native;
    std::optional<std::uint64_t> closed_external;
    bool resetting = false;
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
  void peripheral_cause(PeripheralCalendarCause cause, std::uint64_t ticks);
  void finish_peripheral(bool native_through, bool events_through);
  void observe_peripheral(std::uint64_t ticks);
  void healthy() const;
  void consumed() const;
  void validate_dsp_reset_observation() const;
  void finish_boundary();
  void advance(bool native_through,
               bool external_through,
               const CycleInputs &inputs,
               const Observer &observer);
  void advance_regions(bool native_through,
                       bool external_through,
                       const CycleInputs &inputs,
                       const Observer &observer);
  [[nodiscard]] bool
  advance_region_reset(bool native_through, bool external_through, const InputEvent &event);
  [[nodiscard]] bool advance_native_region(std::uint64_t horizon,
                                           DspBoundary boundary,
                                           const CycleInputs &inputs,
                                           const Observer &observer);
  void prepare_native_effect(unsigned pc, const CycleInputs &inputs, const Observer &observer);
  void observe_native_effect(unsigned pc, const Observer &observer);
  void observe_input_event(const InputEvent &event);
  void advance_reset(std::uint64_t ticks, bool through);
  void observe_external(std::uint64_t ticks);
  [[noreturn]] void reject_native(const dsp56300::Fault &fault) const;
  McuDspRuntimeInputs inputs_;
  McuMachine &mcu_;
  DspMachine &dsp_;
  std::unique_ptr<DspExecution> timeline_;
  DspRuntimeEvents external_;
  DspRuntimePeripheralEvents peripheral_events_;
  std::unique_ptr<PeripheralCalendar> peripheral_;
  State state_;
  bool active_ = false;
  bool faulted_ = false;
  bool observing_signals_ = false;
};
}
