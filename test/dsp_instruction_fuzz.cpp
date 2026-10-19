// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_instruction_fixture.hpp"

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <span>

namespace {
struct Statistics {
  std::uint64_t inputs = 0;
  std::uint64_t decoded = 0;
  std::uint64_t instructions = 0;
  std::uint64_t rejected = 0;
  std::uint64_t budgets = 0;

  ~Statistics() {
    std::cerr << "NATIVE inputs=" << inputs << " decoded=" << decoded
              << " instructions=" << instructions << " rejected=" << rejected
              << " budgets=" << budgets << '\n';
  }
};
}

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t *data, const std::size_t size) {
  static Statistics statistics;
  ++statistics.inputs;
  const auto input = blofeld::test::decode_instruction_input(std::span(data, size));
  if (input) {
    ++statistics.decoded;
    const auto result = blofeld::test::execute_instruction_input(*input);
    statistics.instructions += result.steps;
    if (result.stop == blofeld::test::InstructionStop::rejected) {
      ++statistics.rejected;
    } else {
      ++statistics.budgets;
    }
  }
  return 0;
}
