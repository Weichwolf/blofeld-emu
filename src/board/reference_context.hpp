// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_machine.hpp"

#include <string>

namespace blofeld::reference {
inline std::string mcu_pc_context(const McuMachine &machine) {
  // An active instruction PC is unavailable during the reset sequence.
  if (machine.system_reset_active()) { return "unknown-reset"; }
  return hex(machine.cpu().state().pc);
}
}
