// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "storage/nand_image.hpp"

#include <cstdint>

namespace blofeld {
struct Firmware;

enum class NandUnwrittenState : std::uint8_t { unknown, erased_virtual_device };

// Build new cold memory from both original update payloads. This never changes
// a file/live owner, executes an updater or establishes physical program history.
// Unknown leaves other cells unknown. Erased explicitly declares a new virtual
// device with known FF cells and good unprotected blocks; it is not a factory dump.
[[nodiscard]] NandImage initial_firmware_nand_image(const Firmware &firmware,
                                                    NandUnwrittenState unwritten);
}
