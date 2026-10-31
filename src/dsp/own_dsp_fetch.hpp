// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp56300/core.hpp"

#include <cstdint>
#include <memory>
#include <optional>

namespace blofeld {
class OwnDspMachine;

// Untimed F1/F2 owner. The machine must outlive this object. Reset and restore
// invalidate its identity; a new owner must establish a fresh fetch context.
class OwnDspFetch {
public:
  explicit OwnDspFetch(OwnDspMachine &machine);
  OwnDspFetch(const OwnDspFetch &) = delete;
  OwnDspFetch &operator=(const OwnDspFetch &) = delete;
  [[nodiscard]] DspProgramFetchView view() const;
  void issue();
  void capture();
  [[nodiscard]] dsp56300::InstructionProgress begin();
  // Call only after the admitted instruction's native retirement.
  void synchronize();

private:
  void check_owner() const;
  void check_eligible() const;
  void check_sample_context() const;
  [[nodiscard]] dsp56300::InstructionProgress admit(const DspFetchedInstruction &captured);
  OwnDspMachine &machine_;
  std::shared_ptr<const bool> identity_;
  DspProgramFetch fetch_;
  std::optional<DspFetchedInstruction> admitted_;
  std::optional<DspFetchedInstruction> repeated_;
  std::uint64_t instructions_ = 0;
  bool in_flight_ = false;
};
}
