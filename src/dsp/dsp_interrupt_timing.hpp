// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_core.hpp"

#include <cstdint>

namespace dsp56k {
class Opcodes;
}

namespace blofeld {
// Instruction costs only, with no claimed recognition/pipeline/entry latency.
[[nodiscard]] unsigned nominal_vector_cycles(const DspInterruptVector &vector,
                                             const dsp56k::Opcodes &opcodes,
                                             std::uint32_t external_memory);
}
