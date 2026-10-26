// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_peripheral_read_stage.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace blofeld {
enum class DspReferenceBusOrder : std::uint8_t { unqualified, dma_then_fetch_then_native };

// Selected reference-cycle actions, not physical instruction-stage timestamps.
// No caller supplies a core footprint or program word.
struct DspReferenceCycleInputs {
  DspDmaInputs dma;
  std::optional<DspProgramFetchStage> fetch = std::nullopt;
  std::optional<DspReferenceTicket> access = std::nullopt;
  std::optional<DspReferenceTicket> retirement = std::nullopt;
  std::optional<DspAddressStageTicket> address_update = std::nullopt;
  std::optional<DspAddressStageTicket> memory = std::nullopt;
  // P address of an actual queued AG1 instruction; no supplied opcode or data.
  std::optional<std::uint32_t> peripheral_capture = std::nullopt;
  DspReferenceBusOrder order = DspReferenceBusOrder::unqualified;
};

inline constexpr std::size_t dsp_reference_cycle_accesses = 3;

struct DspReferenceCycleResult {
  DspDmaBusResult dma;
  std::optional<DspReferenceProgress> progress = std::nullopt;
  std::optional<DspInstructionRetire> retirement = std::nullopt;
  std::optional<DspAddressStageView> address_update = std::nullopt;
  std::optional<DspMemoryStageResult> memory = std::nullopt;
  std::optional<DspProgramFetchView> fetch = std::nullopt;
  std::optional<DspPeripheralReadTicket> peripheral_read = std::nullopt;
  std::array<DspMemoryAccess, dsp_reference_cycle_accesses> core_accesses{};
  std::size_t core_access_count = 0;
};
}
