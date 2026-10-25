// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_cycle.hpp"
#include "dsp/dsp_reference_execution.hpp"

#include <optional>

namespace blofeld {
[[nodiscard]] DspReferenceCycleResult
reference_cycle_footprint(const DspBus &bus,
                          const DspReferenceCycleInputs &inputs,
                          const std::optional<DspReferencePending> &pending,
                          const DspProgramFetchView &fetch,
                          std::optional<DspAddressStageView> memory = std::nullopt);
}
