// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "samples/sample_wave.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace blofeld::samples::wave_detail {
using Bytes = std::span<const std::uint8_t>;
[[nodiscard]] std::uint64_t little(Bytes bytes, std::size_t offset, unsigned width);
void format(SampleWave &wave, Bytes bytes);
void metadata(SampleWave &wave, Bytes sampler, Bytes instrument);
void audio(SampleWave &wave, Bytes bytes);
}
