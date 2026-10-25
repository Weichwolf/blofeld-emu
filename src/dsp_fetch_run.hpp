// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_fetch_run.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_program_fetch.hpp"
#include "blofeld/dsp_reference_cycle.hpp"
#include "blofeld/dsp_reference_execution.hpp"

#include <cstdint>
#include <memory>
#include <optional>

namespace blofeld {
struct DspFetchState {
  DspFetchRunInputs inputs;
  std::uint64_t cycles = 0;
  std::uint64_t ticks = 0;
  std::uint64_t clk2_edges = 0;
  std::uint64_t next_dma_cycle = 0;
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
  DspFetchState state;
  const DspMachine *owner = nullptr;
  bool active = false;
  bool faulted = false;
};

struct DspFetchCheckpoint::Storage {
  DspCheckpoint machine;
  DspFetchState clock;
  const DspMachine *owner;
};
}
