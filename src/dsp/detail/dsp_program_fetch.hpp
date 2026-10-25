// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_program_fetch.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
// Owner-driven F1/F2 transitions; elapsed clocks belong to the future scheduler.
class DspProgramFetch {
public:
  [[nodiscard]] DspProgramFetchView view() const { return state_; }

  [[nodiscard]] bool active() const { return state_.active; }

  void start(std::uint32_t pc, std::uint32_t omr);
  void check_stage(DspProgramFetchStage stage) const;
  void issue();
  void capture(DspBus &bus);
  [[nodiscard]] std::optional<std::uint32_t> peek(std::uint32_t address) const;
  void check_consume(std::uint32_t address) const;
  [[nodiscard]] std::uint32_t consume(std::uint32_t address);
  void redirect(std::uint32_t pc, std::uint32_t omr);

private:
  DspProgramFetchView state_;
};
}
