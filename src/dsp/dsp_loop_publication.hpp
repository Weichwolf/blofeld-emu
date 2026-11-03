// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/core.hpp"

namespace blofeld {
class DspProgramFetch;
struct DspFetchedInstruction;

[[nodiscard]] bool native_loop_publication(const DspFetchedInstruction *admitted,
                                           const dsp56300::PendingInstruction &pending);
void observe_loop_publication(DspProgramFetch &fetch,
                              const DspFetchedInstruction &admitted,
                              const dsp56300::Registers &before,
                              const dsp56300::Registers &after);
[[nodiscard]] bool confirm_loop_publication(DspProgramFetch &fetch,
                                            const DspFetchedInstruction &admitted,
                                            const dsp56300::Registers &registers,
                                            const dsp56300::InstructionFetch &context,
                                            const dsp56300::InstructionOutcome &outcome);
struct DspFetchCommit;
void redirect_loop_control(DspProgramFetch &fetch,
                           const DspFetchedInstruction &admitted,
                           DspFetchCommit actual);
} // namespace blofeld
