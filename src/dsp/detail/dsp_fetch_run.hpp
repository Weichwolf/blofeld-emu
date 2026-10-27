// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/detail/dsp_pipeline.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_cycle.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_run_clock.hpp"

#include <cstdint>
#include <memory>
#include <optional>

namespace blofeld {
struct DspFetchState {
  DspFetchRunInputs inputs;
  std::optional<DspInstructionPipeline> pipeline;
  std::uint64_t cycles = 0;
  std::uint64_t ticks = 0;
  std::uint64_t clk2_edges = 0;
  std::uint64_t next_dma_cycle = 0;
  std::uint64_t epoch = 1;
  std::uint64_t epoch_cycles = 0;
  std::uint64_t reset_ticks = 0;
  std::optional<DspRunClock> reset_clock;
  DspDebugState expected_cpu;
  std::optional<DspReferenceTicket> expected_ticket;
  std::optional<DspReferenceTicket> expected_retirement;
  std::optional<std::uint32_t> expected_latch;
  DspProgramFetchView expected_fetch;
  std::optional<std::uint64_t> expected_dma;
  std::shared_ptr<const bool> expected_owner;

  void bind(const DspMachine &machine);
  void verify(const DspMachine &machine) const;
  [[nodiscard]] std::uint64_t period(const DspMachine &machine) const;
  [[nodiscard]] DspReferenceCycleInputs
  plan(const DspMachine &machine, DspFetchCycleInputs inputs, bool &starved) const;
};

struct DspFetchRun::Impl {
  const std::shared_ptr<const bool> lifetime = std::make_shared<const bool>(true);
  DspFetchState state;
  const DspMachine *owner = nullptr;
  bool active = false;
  bool faulted = false;
};

struct DspFetchCheckpoint::Storage {
  DspCheckpoint machine;
  DspFetchState clock;
  const DspMachine *owner;
  std::weak_ptr<const bool> lifetime;
};
}
