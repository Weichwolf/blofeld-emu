// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_address_only.hpp"

#include "dsp56300/transfers.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace blofeld {
std::optional<DspAddressOnlyForm> decode_address_only(const std::uint32_t opcode) {
  const auto transfers = dsp56300::inspect_transfers(opcode);
  if (!transfers || !transfers->address_update) { return std::nullopt; }
  const auto &address = *transfers->address_update;
  using enum dsp56300::AddressKind;
  constexpr std::array modes{
      post_decrement_offset, post_increment_offset, post_decrement, post_increment};
  for (unsigned mode = 0; mode < modes.size(); ++mode) {
    if (modes[mode] == address.kind) {
      return DspAddressOnlyForm{.mode = mode, .reg = address.index};
    }
  }
  return std::nullopt;
}
}
