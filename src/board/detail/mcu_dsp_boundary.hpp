// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "mcu/hc12.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_run_clock.hpp"

#include <array>
#include <cstdint>

namespace blofeld::detail {
inline std::array<std::uint64_t, 15> mcu_boundary(const McuMachine &mcu) {
  const auto cpu = mcu.cpu().state();
  const auto &clock = mcu.run_clock();
  return {cpu.pc,
          cpu.d,
          cpu.x,
          cpu.y,
          cpu.sp,
          cpu.ccr,
          static_cast<std::uint64_t>(mcu.cpu().waiting()),
          clock.elapsed_ticks(),
          clock.bus_cycles(),
          clock.oscillator_cycles(),
          clock.native_epoch(),
          clock.reset_sysclk_edges(),
          mcu.idle_bus_cycles(),
          static_cast<std::uint64_t>(clock.faulted()),
          static_cast<std::uint64_t>(mcu.timing_faulted())};
}
}
