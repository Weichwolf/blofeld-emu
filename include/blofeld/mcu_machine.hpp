// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/flash_image.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_cycles.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_reset.hpp"
#include "blofeld/mcu_run_clock.hpp"
#include "blofeld/mcu_sci.hpp"

#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace blofeld {

enum class McuRunMemory : std::uint8_t { unqualified, unstretched_internal, internal_flash_reads };

struct McuRunContract {
  McuRunClockInputs clock;
  McuRunMemory memory_timing = McuRunMemory::unqualified;
  bool operator==(const McuRunContract &) const = default;
};

// Before an idle IPBus period; this is not an executing CPU bus-cycle slot.
struct McuWaitCycle {
  ExecutionContext context;
};

// After current-boundary IRQ/input checks, before a committed idle period.
// This is not an executing CPU bus slot. Exceptions quarantine the wait owner.
struct McuWaitPeriod {
  ExecutionContext context;
};

// Immutable trusted in-process state. Not a file format or a whole-board snapshot.
class McuCheckpoint {
public:
  McuCheckpoint(const McuCheckpoint &) = default;
  McuCheckpoint &operator=(const McuCheckpoint &) = default;
  McuCheckpoint(McuCheckpoint &&) noexcept = default;
  McuCheckpoint &operator=(McuCheckpoint &&) noexcept = default;
  ~McuCheckpoint();

private:
  friend class McuMachine;
  struct Storage;
  explicit McuCheckpoint(std::shared_ptr<const Storage> storage);
  std::shared_ptr<const Storage> storage_;
};

// Single-threaded MCU owner. Restore replaces bus/CPU objects, invalidating
// references to them. External observers must be explicitly reattached.
class McuMachine {
public:
  McuMachine(const McuFlashImage &image, const std::filesystem::path &specification);
  McuMachine(const McuFlashImage &image,
             const std::filesystem::path &specification,
             std::optional<McuRunContract> run);
  ~McuMachine();
  McuMachine(const McuMachine &) = delete;
  McuMachine &operator=(const McuMachine &) = delete;
  [[nodiscard]] McuBus &bus();
  [[nodiscard]] Hc12 &cpu();
  [[nodiscard]] const McuBus &bus() const;
  [[nodiscard]] const Hc12 &cpu() const;
  [[nodiscard]] bool clocked() const;
  [[nodiscard]] const McuRunClock &run_clock() const;
  [[nodiscard]] bool timing_faulted() const;
  // Explicit board event. Replaces native objects, drops external observers,
  // resets modules and enters a clocked CRG pulse without fetching a vector.
  void begin_system_reset(McuResetSource source, unsigned synchronization_cycles);
  // Caller supplies the observed RESET pad for these SYSCLK edges. Once ready,
  // the native five-cycle CPU reset sequence fetches the selected vector.
  unsigned advance_system_reset(unsigned maximum_sysclk_edges, McuResetPad pad);
  [[nodiscard]] bool system_reset_active() const;
  [[nodiscard]] std::optional<McuResetState> system_reset_state() const;
  // Validate a healthy inactive native clock boundary without copying state.
  void validate_run_boundary() const;
  // Explicit pre-cycle pad observations; external callbacks are never checkpointed.
  void set_cycle_observer(std::function<void(Hc12BusCycle)> observer);
  void set_wait_observer(std::function<void(McuWaitCycle)> observer);
  void set_wait_period_observer(std::function<void(McuWaitPeriod)> observer);
  // Explicit test-scenario hypothesis; never selected by native registers.
  void set_sci_receive_reference(std::optional<SciReceiveReference> reference);
  [[nodiscard]] std::optional<SciReceiveReference> sci_receive_reference() const;
  // Stop before executing the first pending interrupt. Returns elapsed idle
  // IPBus periods; native cpu().step() performs wake/frame reuse separately.
  unsigned advance_wait(unsigned maximum_bus_cycles);
  [[nodiscard]] std::uint64_t idle_bus_cycles() const;
  // Prime selected timer inputs now, then sample them before each running cycle.
  void set_timer_inputs(std::uint8_t mask);
  [[nodiscard]] McuCheckpoint checkpoint() const;
  // Strong exception guarantee; also permits recovery after a failed CPU step.
  void restore(const McuCheckpoint &checkpoint);

private:
  void check_specification() const;
  struct State;
  std::filesystem::path specification_;
  std::vector<std::uint8_t> specification_bytes_;
  std::unique_ptr<State> state_;
};

}
