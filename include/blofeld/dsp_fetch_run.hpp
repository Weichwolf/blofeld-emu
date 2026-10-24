// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_dma.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_program_fetch.hpp"
#include "blofeld/dsp_reference_cycle.hpp"
#include "blofeld/dsp_reference_execution.hpp"

#include <cstdint>
#include <memory>
#include <optional>

namespace blofeld {
enum class DspFetchTiming : std::uint8_t { unqualified, reference_cycle_order };

struct DspFetchRunInputs {
  std::uint32_t extal_hz = 0;
  std::uint64_t timebase_hz = 0;
  DspFetchTiming timing = DspFetchTiming::unqualified;
  bool operator==(const DspFetchRunInputs &) const = default;
};

struct DspFetchCycleInputs {
  std::uint32_t known_requests = 0;
  std::uint32_t requests = 0;
  DspDmaGrant grant = DspDmaGrant::unknown;
  // Backend phase placement is still explicit; a missing fetched word stalls it.
  bool admit_native = false;
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
  ~DspFetchRun();
  DspFetchRun(const DspFetchRun &) = delete;
  DspFetchRun &operator=(const DspFetchRun &) = delete;
  [[nodiscard]] DspReferenceProgress begin_instruction(DspMachine &machine);
  [[nodiscard]] DspFetchCycleResult advance_cycle(DspMachine &machine, DspFetchCycleInputs inputs);
  [[nodiscard]] DspFetchCheckpoint checkpoint(const DspMachine &machine) const;
  void restore(const DspFetchCheckpoint &checkpoint, DspMachine &machine);
  [[nodiscard]] std::uint64_t elapsed_ticks() const;
  [[nodiscard]] std::uint64_t completed_cycles() const;
  [[nodiscard]] std::uint64_t timer_clk2_edges() const;
  [[nodiscard]] bool faulted() const;

private:
  void check_owner(const DspMachine &machine) const;
  void fail(DspMachine &machine) noexcept;
  void complete_cycle(DspMachine &machine, std::uint64_t period, bool switch_was_pending);
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
