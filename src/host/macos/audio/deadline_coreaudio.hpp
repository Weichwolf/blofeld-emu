// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <chrono>
#include <cstdint>
#include <mach/kern_return.h>

namespace blofeld::host::audio {
[[nodiscard]] std::uint32_t device_period_ticks(std::chrono::nanoseconds duration);
[[nodiscard]] kern_return_t set_current_deadline(std::uint32_t period) noexcept;
}
