// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_interrupts.hpp"
#include "dsp56300/core.hpp"

#include <optional>

namespace blofeld {
// Acknowledge a real peripheral request only after both native vector reads.
class DspInterruptAdmission {
public:
  DspInterruptAdmission(DspBus &bus, dsp56300::Core &cpu) noexcept : bus_(bus), cpu_(cpu) {}

  DspInterruptAdmission(const DspInterruptAdmission &) = delete;
  DspInterruptAdmission &operator=(const DspInterruptAdmission &) = delete;

  [[nodiscard]] bool pending() const noexcept { return request_.has_value(); }

  [[nodiscard]] bool admit(dsp56300::InstructionWords foreground);
  [[nodiscard]] bool admit();
  void fetch_completed(unsigned address);

private:
  friend class DspMachine;
  void reset_after_cpu_reset() noexcept;
  DspBus &bus_;
  dsp56300::Core &cpu_;
  std::optional<DspInterruptRequest> request_;
  dsp56300::Word vector_ = 0;
  bool first_ = false;
};
}
