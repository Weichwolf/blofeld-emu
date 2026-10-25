// SPDX-License-Identifier: GPL-3.0-only
#include "hc12_minmax_fixture.hpp"

#include "mcu/hc12.hpp"

namespace blofeld::test {
void check_minmax_result(const MinmaxMemory &memory,
                         const blofeld::Hc12 &cpu,
                         const MinmaxOperation operation,
                         const MinmaxValues values,
                         const unsigned leading_reads) {
  const auto state = cpu.state();
  const auto result = minmax_value(operation, values);
  const auto expected_d = minmax_accumulator(operation, operation.memory ? values.left : result);
  require_minmax(state.d == expected_d && state.ccr == minmax_flags(values, operation.word),
                 "Min/max must retain independent result, subtraction flags and other registers");
  const unsigned width = operation.word ? 2U : 1U;
  require_minmax(memory.access_count == (operation.memory ? 2U : 1U) + leading_reads &&
                     memory.accesses[leading_reads] == MinmaxAccess{.address = memory.source,
                                                                    .size = width,
                                                                    .value = values.right},
                 "Min/max must read its original operand exactly once at its native width");
  if (operation.memory) {
    require_minmax(
        memory.accesses[leading_reads + 1] ==
            MinmaxAccess{.write = true, .address = memory.source, .size = width, .value = result},
        "Memory min/max always writes the selected value, including unchanged data");
  }
  const unsigned high = memory.bytes[memory.source];
  const unsigned observed = operation.word ? (high << 8U) | memory.bytes[memory.source + 1] : high;
  require_minmax(observed == (operation.memory ? result : values.right),
                 "Min/max destination and source preservation must match independent values");
}

}
