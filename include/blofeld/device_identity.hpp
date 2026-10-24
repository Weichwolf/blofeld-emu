// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/flash_image.hpp"

#include <cstddef>
#include <string_view>

namespace blofeld {
namespace device_identity {
inline constexpr unsigned serial_address = 0x81e0;
inline constexpr std::size_t serial_bytes = 32;
}

// Factory data for a new virtual unit, before CPU execution. The original
// update/reference bytes stay immutable. Existing or unknown unit data cannot
// be overwritten; restore an original NVM image directly instead.
[[nodiscard]] McuFlashImage provision_virtual_identity(const McuFlashImage &unprovisioned,
                                                       std::string_view serial);
}
