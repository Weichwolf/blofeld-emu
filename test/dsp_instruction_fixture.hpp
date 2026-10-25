// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_core.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace blofeld::test {
inline constexpr unsigned instruction_seed_words = 14;
inline constexpr unsigned instruction_program_words = 64;
inline constexpr unsigned instruction_step_limit = 128;
inline constexpr unsigned instruction_data_begin = 0x100;
inline constexpr unsigned instruction_data_words = 64;
inline constexpr std::size_t instruction_input_bytes =
    static_cast<std::size_t>(instruction_seed_words + instruction_program_words) * 3;

struct InstructionInput {
  // X0/X1/Y0/Y1, A0/A1/A2, B0/B1/B2, R0/N0/M0, CCR/scaling/compatibility.
  std::array<std::uint32_t, instruction_seed_words> registers{};
  std::vector<std::uint32_t> program;
};

enum class InstructionStop : std::uint8_t { budget, rejected };

struct InstructionResult {
  InstructionStop stop = InstructionStop::budget;
  unsigned steps = 0;
  DspDebugState state;
  std::string fault;
  std::array<std::uint32_t, instruction_data_words> x{};
  std::array<std::uint32_t, instruction_data_words> y{};
};

[[nodiscard]] std::optional<InstructionInput>
decode_instruction_input(std::span<const std::uint8_t> bytes);
[[nodiscard]] InstructionResult
execute_instruction_input(const InstructionInput &input,
                          unsigned step_budget = instruction_step_limit);
}
