// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp_memory_pipeline_fixture.hpp"

#include <cstdint>

namespace blofeld::test::branch_pipeline {
using memory_pipeline::body;
using memory_pipeline::clocks;
using memory_pipeline::contract;
using memory_pipeline::prepare;
inline constexpr unsigned once = 0xfffffc;
inline constexpr unsigned literal = 0x654321;
inline constexpr unsigned spi_omr = 0x305;
inline constexpr std::uint64_t word_mask = 0xffffff;
void branches();
void state_checks();
}
