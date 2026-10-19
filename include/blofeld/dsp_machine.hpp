// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_run_clock.hpp"

#include <cstdint>
#include <memory>
#include <optional>

namespace blofeld {

enum class DspRunTiming : std::uint8_t { unqualified, native_internal_estimates };

struct DspRunContract {
  DspRunClockInputs clock;
  DspRunTiming timing = DspRunTiming::unqualified;
  bool operator==(const DspRunContract &) const = default;
};

// Immutable trusted in-process state; not a portable file or whole-board format.
class DspCheckpoint {
public:
  DspCheckpoint(const DspCheckpoint &) = default;
  DspCheckpoint &operator=(const DspCheckpoint &) = default;
  DspCheckpoint(DspCheckpoint &&) noexcept = default;
  DspCheckpoint &operator=(DspCheckpoint &&) noexcept = default;
  ~DspCheckpoint();

private:
  friend class DspMachine;
  struct Storage;
  explicit DspCheckpoint(std::shared_ptr<const Storage> storage);
  std::shared_ptr<const Storage> storage_;
};

// Single-threaded DSP owner. Restore replaces bus/CPU objects and invalidates
// references to them; external wiring and schedules belong to their callers.
class DspMachine {
public:
  DspMachine();
  explicit DspMachine(std::optional<DspRunContract> run);
  ~DspMachine();
  DspMachine(const DspMachine &) = delete;
  DspMachine &operator=(const DspMachine &) = delete;
  [[nodiscard]] DspBus &bus();
  [[nodiscard]] DspCore &cpu();
  [[nodiscard]] const DspBus &bus() const;
  [[nodiscard]] const DspCore &cpu() const;
  [[nodiscard]] bool clocked() const;
  [[nodiscard]] const DspRunClock &run_clock() const;
  [[nodiscard]] bool timing_faulted() const;
  [[nodiscard]] DspCheckpoint checkpoint() const;
  // Strong exception guarantee; a prior valid checkpoint can recover a fault.
  void restore(const DspCheckpoint &checkpoint);

private:
  struct State;
  std::unique_ptr<State> state_;
};

}
