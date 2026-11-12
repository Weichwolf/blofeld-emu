// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/boot_rom.hpp"
#include "dsp/program.hpp"
#include "profile.hpp"
#include "storage/firmware.hpp"

#include <cstdint>
#include <dsp56300/core.hpp>
#include <span>
#include <string_view>

namespace blofeld::app::aot {
struct DspSource {
  std::string_view name;
  std::span<const std::uint8_t> bytes;
};

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
// Bind a local source catalogue to the exact decoded firmware and ROM inputs.
// Source construction and compilation execute no device or guest state.
[[nodiscard]] vi::dsp::Program prepare_dsp_catalogue(std::string_view document,
                                                     const vi::FirmwareUpdate &firmware,
                                                     vi::dsp::BootRom boot,
                                                     std::span<const DspSource> sources = {});
[[nodiscard]] vi::dsp::Program load_dsp_program(const Profile &profile, vi::dsp::BootRom boot);
}
