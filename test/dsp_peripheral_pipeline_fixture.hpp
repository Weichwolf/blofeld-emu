// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp_memory_pipeline_fixture.hpp"

#include <filesystem>

namespace blofeld::test::peripheral_pipeline {
inline constexpr unsigned once = 0xfffffc;
inline constexpr unsigned clock_control = 0xffff90;
inline constexpr unsigned control = 0xffff91;
inline constexpr unsigned transmit = 0xffff93;
inline constexpr unsigned literal = 0x234567;
using memory_pipeline::clocks;
using memory_pipeline::contract;
using memory_pipeline::prepare;
using memory_pipeline::stages;
inline constexpr unsigned body = memory_pipeline::body;
void writes();
void state_checks();

struct BootstrapFiles {
  std::filesystem::path rom;
  std::filesystem::path trace;
};

void bootstrap(const BootstrapFiles &files);
}
