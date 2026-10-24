// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/hcs08.hpp"

#include <cstdint>
#include <optional>

namespace blofeld::detail::hcs08 {
[[nodiscard]] std::optional<std::uint16_t> pending_interrupt(const Bus &bus,
                                                             const Hcs08State &state);
}
