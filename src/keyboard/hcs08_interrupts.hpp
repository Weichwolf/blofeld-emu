// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "keyboard/hcs08.hpp"

#include <cstdint>
#include <optional>

namespace blofeld::detail::hcs08 {
[[nodiscard]] std::optional<std::uint16_t> pending_interrupt(
    const Bus &bus, const Hcs08State &state, Hcs08IrqDelay delay = Hcs08IrqDelay::none);

[[nodiscard]] Hcs08IrqDelay irq_delay_after(const Hcs08State &before,
                                            const Hcs08State &after,
                                            unsigned opcode,
                                            Hcs08RtiIrqContract rti_contract);
}
