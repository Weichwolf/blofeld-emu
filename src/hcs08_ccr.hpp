// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "blofeld/bus.hpp"
#include "blofeld/hcs08.hpp"

#include <cstdint>

namespace blofeld::detail::hcs08 {
// Conservative instruction-form qualification for partial reset CCR knownness.
// Fully known debug/native paths continue through the complete SLEIGH engine.
[[nodiscard]] std::uint8_t ccr_after(const Bus &bus, const Hcs08State &before);
}
