// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "mcu_dsp_runtime_fixture.hpp"

#include <cstdint>
#include <span>

namespace blofeld::test::mcu_reset {
using mcu_dsp::Fixture;
using mcu_dsp::rejects;
using mcu_dsp::require;
inline constexpr unsigned mcu_hz = 18000000, mcu_timebase = 180000000;
inline constexpr unsigned initial_ticks = 300, mcu_period = 60, sysclk_period = 30;
inline constexpr unsigned u_data = 0x0278, u_direction = 0x027a;
inline constexpr unsigned ram = 0x1200, dsp_ram = 0x400, marker = 0x5a;
inline constexpr unsigned native_release = 480, recovery = 90;

mcu_dsp::Options options(std::span<const std::uint8_t> program = {});
void configure(Fixture &fixture);
void held(Fixture &fixture,
          std::uint64_t until,
          std::uint64_t completed = 0,
          std::uint64_t clk2 = 0);
void observe_nops(Fixture &fixture, std::uint64_t recovery_end, std::uint64_t previous = 0);

struct Running {
  std::uint64_t until;
  std::uint64_t recovery_end;
  std::uint64_t reset_ticks;
  std::uint64_t previous_cycles = 0;
  std::uint64_t previous_clk2 = 0;
};

void running(Fixture &fixture, Running expected);
void running_reset();
void guards();
}
