// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <optional>

namespace blofeld {
struct DspAddressOnlyForm {
  unsigned mode = 0;
  unsigned reg = 0;
};

// FM13-117 U parallel move: address-register update, without a data operand.
[[nodiscard]] std::optional<DspAddressOnlyForm> decode_address_only(std::uint32_t opcode);
}
