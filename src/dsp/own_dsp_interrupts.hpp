// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_interrupts.hpp"
#include "dsp56300/core.hpp"

#include <optional>

namespace blofeld {
// Untimed interrupt ownership. The board scheduler still owns clocks and reset.
// Advance every staged effect through this owner after admitting a request.
class OwnDspInterrupts {
public:
  OwnDspInterrupts(DspBus &bus, dsp56300::Core &cpu) noexcept : bus_(bus), cpu_(cpu) {}

  OwnDspInterrupts(const OwnDspInterrupts &) = delete;
  OwnDspInterrupts &operator=(const OwnDspInterrupts &) = delete;

  [[nodiscard]] bool pending() const noexcept { return request_.has_value(); }

  [[nodiscard]] bool admit(dsp56300::InstructionWords foreground);
  [[nodiscard]] dsp56300::InstructionProgress advance(const dsp56300::EffectTicket &ticket);

private:
  DspBus &bus_;
  dsp56300::Core &cpu_;
  std::optional<DspInterruptRequest> request_;
  dsp56300::Word vector_ = 0;
  bool first_ = false;
};
}
