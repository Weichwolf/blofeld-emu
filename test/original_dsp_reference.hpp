// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_machine.hpp"

#include <iosfwd>

namespace blofeld::test {
void verify_original_reference_access(const DspCheckpoint &last_pair,
                                      const DspMachine &frontier,
                                      std::ostream &trace);
}
