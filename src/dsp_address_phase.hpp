// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_native_access.hpp"
#include "blofeld/dsp_reference_execution.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace blofeld {
struct DspAddressPhase {
  std::optional<DspAddressUpdate> pending;
  std::array<std::optional<std::uint32_t>, 8> published{};

  void capture(std::uint32_t opcode,
               const DspNativeAccess &access,
               const std::array<std::uint32_t, 8> &before,
               const std::array<std::uint32_t, 8> &native);
};
}
