// SPDX-License-Identifier: GPL-3.0-only
#pragma once

namespace blofeld {
struct DspMemoryAccess;

namespace detail {
// Classifies the shared native/DMA resource; performs no register access.
[[nodiscard]] bool independent_dma_peripheral(DspMemoryAccess access);
}
}
