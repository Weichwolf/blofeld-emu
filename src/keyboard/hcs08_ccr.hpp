// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "common/bus.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_known_memory.hpp"

#include <cstdint>

namespace blofeld::detail::hcs08 {
// Conservative instruction-form qualification for partial reset CCR knownness.
// Fully known debug/native paths continue through the complete SLEIGH engine.
[[nodiscard]] std::uint8_t ccr_after(const Bus &bus, const Hcs08State &before);
[[nodiscard]] std::uint8_t byte_vnz_after(const Hcs08State &before, Hcs08KnownByte source);
}
