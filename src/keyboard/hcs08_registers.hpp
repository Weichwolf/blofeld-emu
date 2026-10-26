// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "keyboard/hcs08.hpp"

#include <cstdint>

namespace blofeld::detail::hcs08 {
struct RegisterKnowledge {
  std::uint8_t a;
  std::uint16_t hx;
};

// Qualify dependencies before native execution, then commit only on retirement.
// Unknown representatives must never reach an unqualified native consumer.
[[nodiscard]] RegisterKnowledge
registers_after(const Bus &code, const Hcs08State &before, const Bus &data);
}
