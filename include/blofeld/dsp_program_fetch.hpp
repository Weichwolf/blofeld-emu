// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace blofeld {
enum class DspProgramFetchStage : std::uint8_t { address, read };

struct DspProgramWord {
  std::uint32_t address = 0;
  std::uint32_t value = 0;
  bool operator==(const DspProgramWord &) const = default;
};

// Fixed host continuation bound, not a claim about the silicon queue depth.
inline constexpr std::size_t dsp_program_fetch_capacity = 8;

struct DspProgramFetchView {
  std::array<DspProgramWord, dsp_program_fetch_capacity> words{};
  std::size_t used = 0;
  std::optional<std::uint32_t> pending_address = std::nullopt;
  std::uint32_t next_address = 0;
  std::uint32_t consume_address = 0;
  std::uint32_t omr = 0;
  bool active = false;
  bool operator==(const DspProgramFetchView &) const = default;
};
}
