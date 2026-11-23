// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstdint>

namespace blofeld::host::audio {
using Frame = std::array<std::int32_t, 2>; // Signed 24-bit PCM in native 32-bit containers.
}
