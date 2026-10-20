// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/dsp_reference_execution.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>

namespace blofeld {

struct DspState {
  std::uint32_t pc = 0;
  std::uint32_t sr = 0;
  std::uint32_t omr = 0;
  std::uint32_t sp = 0;
  std::uint32_t vba = 0;
  std::uint32_t stack_pc = 0;
  std::uint32_t stack_sr = 0;
  std::uint32_t la = 0;
  std::uint32_t lc = 0;
  std::uint64_t x = 0;
  std::uint64_t y = 0;
  std::uint64_t a = 0;
  std::uint64_t b = 0;
  std::array<std::uint32_t, 8> r{};
  std::array<std::uint32_t, 8> n{};
  std::array<std::uint32_t, 8> m{};
  bool operator==(const DspState &) const = default;
};

// Observations without destructive SSH reads. This view omits native REP/IRQ
// continuation; DspMachine provides opaque restorable checkpoints.
struct DspDebugState {
  DspState cpu;
  // SDK diagnostic counters, not calibrated elapsed instruction/bus clocks.
  std::uint64_t native_instructions = 0;
  std::uint64_t native_cycles = 0;
  std::uint32_t ep = 0;
  std::uint32_t sz = 0;
  std::uint32_t sc = 0;
  std::array<std::uint32_t, 16> stack_high{};
  std::array<std::uint32_t, 16> stack_low{};
  bool operator==(const DspDebugState &) const = default;
};

// Opaque native state; only DspMachine can pair it with matching bus contents.
class DspCoreCheckpoint {
public:
  DspCoreCheckpoint(const DspCoreCheckpoint &) = default;
  DspCoreCheckpoint &operator=(const DspCoreCheckpoint &) = default;
  DspCoreCheckpoint(DspCoreCheckpoint &&) noexcept = default;
  DspCoreCheckpoint &operator=(DspCoreCheckpoint &&) noexcept = default;
  ~DspCoreCheckpoint();

private:
  friend class DspCore;
  struct Storage;
  explicit DspCoreCheckpoint(std::shared_ptr<const Storage> storage);
  std::shared_ptr<const Storage> storage_;
};

// Native interpreter reference path. Each DO/REP body instruction can yield
// independently. Interrupt entry executes at most two vector words atomically;
// peripheral time remains an explicit board-model input.
class DspCore {
public:
  explicit DspCore(DspBus &bus);
  ~DspCore();
  DspCore(const DspCore &) = delete;
  DspCore &operator=(const DspCore &) = delete;
  void reset_spi_boot();
  [[nodiscard]] DspState state() const;
  [[nodiscard]] DspDebugState debug_state() const;
  void step();
  void step(DspNativeAccessObserver observer);

private:
  friend class DspMachine;

  struct RunHooks {
    std::function<void(DspState, bool)> begin;
    std::function<void(unsigned)> complete;
    std::function<void()> failed;
    std::function<void()> reset;
  };

  void set_run_hooks(RunHooks hooks);
  [[nodiscard]] bool faulted() const;
  [[nodiscard]] std::uint64_t native_cycles() const;
  [[nodiscard]] unsigned next_step_cycles() const;
  [[nodiscard]] DspCoreCheckpoint checkpoint() const;
  void restore_checkpoint(const DspCoreCheckpoint &checkpoint);
  [[nodiscard]] bool executing() const;
  [[nodiscard]] DspReferenceProgress begin_reference_instruction(DspReferenceExecution contract);
  [[nodiscard]] DspReferenceProgress begin_reference_interrupt(DspReferenceExecution contract);
  [[nodiscard]] DspReferenceProgress begin_reference_step(DspReferenceExecution contract,
                                                          bool interrupt_entry);
  [[nodiscard]] DspReferenceProgress advance_reference_access(const DspReferenceTicket &ticket);
  [[nodiscard]] std::optional<DspReferencePending> reference_pending() const;
  [[nodiscard]] std::optional<DspReferencePending> reference_pending_unchecked() const;
  [[nodiscard]] DspReferenceProgress
  reference_progress(std::optional<DspReferenceEffect> completed = std::nullopt) const;
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
