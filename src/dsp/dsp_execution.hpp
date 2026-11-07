// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_io.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp56300/core.hpp"

#include <cstdint>
#include <memory>
#include <optional>

namespace blofeld {
enum class DspBoundary : std::uint8_t { before, through };

struct DspExecutionResult {
  std::uint64_t start_ticks;
  std::uint64_t completion_ticks;
  std::optional<dsp56300::RunResult> execution;
};

// Functional instruction execution on the shared device timeline. Instructions
// are atomic; an event inside the final instruction delays its next dispatch.
// No fetch pipeline, copied CPU state, effect tickets or synthetic bus grants.
class DspExecution {
public:
  DspExecution(DspMachine &machine,
               std::uint64_t epoch_ticks,
               std::optional<DspResetInputs> initial_reset = std::nullopt);
  DspExecution(const DspExecution &) = delete;
  DspExecution &operator=(const DspExecution &) = delete;
  void begin_hardware_reset(DspResetInputs inputs);
  std::uint64_t advance_reset_to(std::uint64_t horizon);
  void release_hardware_reset();
  [[nodiscard]] std::uint64_t next_deadline() const;
  [[nodiscard]] std::optional<DspExecutionResult>
  advance_native_region(std::uint64_t horizon,
                        DspBoundary boundary,
                        DspDmaInputs inputs,
                        DspPeripheralObserver observer = {});

  [[nodiscard]] bool faulted() const noexcept { return faulted_; }

  void validate_boundary() const;

private:
  void start_reset(DspResetInputs inputs);
  [[nodiscard]] std::uint64_t now() const;
  [[nodiscard]] unsigned capacity(std::uint64_t horizon, DspBoundary boundary) const;
  void native_time(unsigned cycles);
  void synchronize_prefix(unsigned pc);
  void peripheral_completed(unsigned pc);
  DspMachine &machine_;
  std::shared_ptr<const bool> identity_;
  std::uint64_t epoch_ticks_;
  std::uint64_t accounted_base_ = 0;
  std::uint64_t remaining_cycles_ = 0;
  unsigned available_cycles_ = 0;
  DspDmaInputs inputs_;
  DspPeripheralObserver observer_;
  bool faulted_ = false;
};
}
