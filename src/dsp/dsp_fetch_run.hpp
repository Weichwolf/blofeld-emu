// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_cycle.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_run_clock.hpp"

#include <cstdint>
#include <memory>
#include <optional>

namespace blofeld {
enum class DspFetchTiming : std::uint8_t { unqualified, reference_cycle_order };

enum class DspFetchResetTiming : std::uint8_t { unqualified, reference_cycle_boundary };

struct DspFetchRunInputs {
  std::uint32_t extal_hz = 0;
  std::uint64_t timebase_hz = 0;
  DspFetchTiming timing = DspFetchTiming::unqualified;
  DspReferenceExecution execution = DspReferenceExecution::access_boundaries;
  DspFetchPipeline pipeline = DspFetchPipeline::explicit_effects;
  DspFetchResetTiming resets = DspFetchResetTiming::unqualified;
  bool operator==(const DspFetchRunInputs &) const = default;
};

struct DspFetchCycleInputs {
  std::uint32_t known_requests = 0;
  std::uint32_t requests = 0;
  DspDmaGrant grant = DspDmaGrant::unknown;
  // Backend phase placement is still explicit; a missing fetched word stalls it.
  bool admit_native = false;
  bool retire_previous = false;
};

struct DspFetchCycleResult {
  std::uint64_t completed_cycles = 0;
  std::uint64_t start_ticks = 0;
  std::uint64_t completion_ticks = 0;
  std::uint64_t period_ticks = 0;
  DspReferenceCycleResult bus;
  std::optional<std::uint32_t> issued_address = std::nullopt;
  DspProgramFetchView fetch;
  bool native_starved = false;
  std::optional<DspPipelineCycle> pipeline = std::nullopt;
};

class DspFetchCheckpoint {
public:
  DspFetchCheckpoint(const DspFetchCheckpoint &) = default;
  DspFetchCheckpoint &operator=(const DspFetchCheckpoint &) = default;
  ~DspFetchCheckpoint();

private:
  friend class DspFetchRun;
  struct Storage;
  explicit DspFetchCheckpoint(std::shared_ptr<const Storage> storage);
  std::shared_ptr<const Storage> storage_;
};

// Single-threaded F1/F2 reference clock, not the complete seven-stage backend.
// Each cycle orders DMA, old F2, selected native effect, new F1, peripheral clocks.
class DspFetchRun {
public:
  DspFetchRun(DspFetchRunInputs inputs, DspMachine &machine);
  // Initial asserted reset needs no pre-reset opcode or ROM fetch.
  DspFetchRun(DspFetchRunInputs inputs,
              DspMachine &machine,
              std::optional<DspHardwareResetInputs> initial_reset);
  ~DspFetchRun();
  DspFetchRun(const DspFetchRun &) = delete;
  DspFetchRun &operator=(const DspFetchRun &) = delete;
  [[nodiscard]] DspReferenceProgress begin_instruction(DspMachine &machine);
  [[nodiscard]] DspReferenceProgress overlap_instruction(DspMachine &machine);
  [[nodiscard]] DspFetchCycleResult advance_cycle(DspMachine &machine, DspFetchCycleInputs inputs);
  // Explicit nominal reset at a completed cycle; caller owns stable observed pins.
  void begin_hardware_reset(DspMachine &machine, DspHardwareResetInputs inputs);
  std::uint64_t advance_hardware_reset_ticks(DspMachine &machine, std::uint64_t maximum_ticks);
  void release_hardware_reset(DspMachine &machine);
  [[nodiscard]] bool hardware_reset_active() const;
  [[nodiscard]] std::optional<DspHardwareResetState> hardware_reset_state() const;
  [[nodiscard]] std::optional<std::uint64_t> hardware_reset_remaining_ticks() const;
  [[nodiscard]] std::uint64_t hardware_reset_ticks() const;
  [[nodiscard]] std::uint64_t native_epoch() const;
  [[nodiscard]] DspFetchCheckpoint checkpoint(const DspMachine &machine) const;
  void restore(const DspFetchCheckpoint &checkpoint, DspMachine &machine);
  // Pure boundary query: validates the bound owner and consumes no native work.
  [[nodiscard]] std::uint64_t next_cycle_ticks(const DspMachine &machine) const;
  // Pure ownership/native-state validation, including held reset boundaries.
  void validate_boundary(const DspMachine &machine) const;
  [[nodiscard]] std::uint64_t elapsed_ticks() const;
  [[nodiscard]] std::uint64_t completed_cycles() const;
  [[nodiscard]] std::uint64_t timer_clk2_edges() const;
  [[nodiscard]] bool faulted() const;
  [[nodiscard]] std::optional<DspPipelineView> pipeline() const;

private:
  void check_owner(const DspMachine &machine) const;
  void check_running(const DspMachine &machine) const;
  void fail(DspMachine &machine) noexcept;
  void complete_cycle(DspMachine &machine, std::uint64_t period, bool switch_was_pending);
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
