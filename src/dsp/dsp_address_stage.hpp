// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>

namespace blofeld {
class DspAddressStageTicket {
public:
  DspAddressStageTicket() = default;
  bool operator==(const DspAddressStageTicket &) const = default;

private:
  friend class DspCore;
  std::shared_ptr<const bool> owner_;
  std::size_t slot_ = 0;
  std::uint64_t serial_ = 0;
};

struct DspAddressStageLane {
  DspAddressUpdate update;
  bool pointer_update = false;
};

enum class DspAddressStageKind : std::uint8_t {
  register_agu,
  peripheral_immediate,
  peripheral_register,
  absolute_short,
  absolute_long,
  peripheral_bit,
  address_only
};

struct DspAddressStageView {
  DspProgramWord word;
  DspAddressUpdate update;
  bool pointer_update = false;
  bool published = false;
  std::optional<DspAddressStageLane> secondary = std::nullopt;
  std::array<DspMemoryAccess, 2> data_accesses{};
  std::size_t data_access_count = 0;
  DspAddressStageKind kind = DspAddressStageKind::register_agu;
  std::optional<DspProgramWord> extension = std::nullopt;
};

struct DspAddressOnlyStageResult {
  DspAddressStageView address;
  DspState inputs;
  DspReferenceProgress progress;
};

struct DspMemoryStageResult {
  DspAddressStageView address;
  DspState inputs;
  DspNativeAccess data;
  bool bit_read = false; // Genuine E1 read;native bit callback remains pending for E2.
  std::optional<DspNativeAccess> additional_data = std::nullopt;
  std::optional<DspMoveWriteback> move = std::nullopt;
  DspReferenceProgress progress;
};
}
