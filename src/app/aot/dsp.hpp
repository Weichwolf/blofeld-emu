// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/program.hpp"

#include <dsp56300/core.hpp>
#include <span>

namespace blofeld::app::aot {
// Prepare the complete supplied code catalogue against the concrete DSP adapter.
// No Core or device state is executed or captured during compilation.
[[nodiscard]] dsp56300::NativeImage
prepare_dsp(dsp56300::CodeImage image,
            std::span<const dsp56300::VariantLayout> memory = {},
            std::span<const dsp56300::InterruptRequest> interrupts = {},
            std::span<const dsp56300::CodeTransition> transitions = {});
[[nodiscard]] vi::dsp::Program
prepare_dsp_program(dsp56300::CodeImage image,
                    std::span<const vi::dsp::ProgramChoice> choices,
                    std::span<const dsp56300::VariantLayout> memory = {},
                    std::span<const dsp56300::InterruptRequest> interrupts = {},
                    std::span<const dsp56300::CodeTransition> transitions = {});
}
