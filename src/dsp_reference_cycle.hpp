// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_program_fetch.hpp"
#include "blofeld/dsp_reference_cycle.hpp"
#include "blofeld/dsp_reference_execution.hpp"

#include <optional>

namespace blofeld {
[[nodiscard]] DspReferenceCycleResult
reference_cycle_footprint(const DspBus &bus,
                          const DspReferenceCycleInputs &inputs,
                          const std::optional<DspReferencePending> &pending,
                          const DspProgramFetchView &fetch,
                          std::optional<DspAddressStageView> memory = std::nullopt);
}
