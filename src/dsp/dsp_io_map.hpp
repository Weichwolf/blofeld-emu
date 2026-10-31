// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>

namespace blofeld {
struct DspAddress;

enum class DspIoSelection : std::uint8_t { outside, assigned, reserved };

// DSP56371UM tables3-7/3-8. Assignment does not imply implemented semantics;
// reserved slots do not imply any known read value or accepted write behavior.
[[nodiscard]] DspIoSelection dsp_io_selection(DspAddress access);
}
