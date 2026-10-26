// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"

#include <optional>

namespace blofeld {
struct Hcs08State;

// Optional source query for owners that retain unknown pending-event history.
// Only a known set CPU I bit can exclude an uncertain maskable source. Otherwise
// return a known request/no request, or fail with the actual execution context.
// Querying never acknowledges a source. Board reset handling remains separate
// and must run even during the CPU's next-instruction maskable-IRQ guarantee.
class Hcs08KnownInterrupts {
public:
  virtual ~Hcs08KnownInterrupts() = default;
  [[nodiscard]] virtual std::optional<InterruptRequest>
  interrupt_request(const Hcs08State &state) const = 0;
};
} // namespace blofeld
