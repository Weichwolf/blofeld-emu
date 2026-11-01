// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"

#include <iosfwd>

namespace blofeld::test {
void require_original_native(const OwnDspMachine &machine,
                             const dsp56300::InstructionProgress &progress);
void verify_original_reference_access(const OwnDspCheckpoint &last_pair,
                                      const OwnDspMachine &frontier,
                                      std::ostream &trace);
}
