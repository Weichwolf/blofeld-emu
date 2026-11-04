// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_interlocks.hpp"
#include "dsp/dsp_native_dependencies.hpp"
#include "dsp_native_dependency_operands.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {
unsigned assertions = 0;

void require(const bool condition, const std::string_view message) {
  ++assertions;
  if (!condition) { throw std::runtime_error(std::string(message)); }
}

struct Step {
  std::uint32_t opcode;
  unsigned base;
};

// Own program, independently assembled with NXP 6.3.28. Quiet-anchor windows
// are fixed normal-model observations, not instruction-retirement timestamps.
constexpr std::array program{
    Step{.opcode = 0x05f43a, .base = 2}, Step{.opcode = 0x05f439, .base = 2},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x56f400, .base = 2}, Step{.opcode = 0x57f400, .base = 2},
    Step{.opcode = 0x44f400, .base = 2}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x0c1940, .base = 2},
    Step{.opcode = 0x04cc90, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x56f400, .base = 2},
    Step{.opcode = 0x57f400, .base = 2}, Step{.opcode = 0x44f400, .base = 2},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x0c1940, .base = 2}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x04cc90, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x56f400, .base = 2},
    Step{.opcode = 0x57f400, .base = 2}, Step{.opcode = 0x44f400, .base = 2},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x0c1940, .base = 2}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x04cc90, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x56f400, .base = 2}, Step{.opcode = 0x57f400, .base = 2},
    Step{.opcode = 0x44f400, .base = 2}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x0c1941, .base = 2},
    Step{.opcode = 0x04cc90, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1}, Step{.opcode = 0x000000, .base = 1},
    Step{.opcode = 0x000000, .base = 1},
};

struct Window {
  unsigned first;
  unsigned last;
  unsigned expected;
};

constexpr std::array windows{Window{.first = 20, .last = 28, .expected = 10},
                             Window{.first = 41, .last = 50, .expected = 10},
                             Window{.first = 63, .last = 73, .expected = 11},
                             Window{.first = 86, .last = 94, .expected = 9}};

void reference_windows() {
  blofeld::DspInterlocks history;
  std::array<unsigned, program.size()> clocks{};
  unsigned elapsed = 0;
  for (unsigned index = 0; index < program.size(); ++index) {
    const auto step = program.at(index);
    const auto decoded = blofeld::inspect_native_dependencies(step.opcode, step.base);
    if (!decoded) { throw std::runtime_error("Missing authored dependency description"); }
    ++assertions;
    const auto before = history;
    const auto stalls = history.preview(*decoded);
    require(stalls == history.preview(*decoded), "Dependency preview is repeatable");
    auto restored = before;
    restored.prepare(*decoded);
    restored.observe_opcode(step.opcode);
    require(restored.complete(step.base) == stalls, "A copied history preserves its next delay");
    history.prepare(*decoded);
    history.observe_opcode(step.opcode);
    require(history.complete(step.base) == stalls,
            "Completion retains the prepared dependency clock");
    elapsed += step.base + stalls;
    clocks.at(index) = elapsed;
  }
  for (const auto &window : windows) {
    require(clocks.at(window.last) - clocks.at(window.first) == window.expected,
            "Independent normal-mode NXP dependency window");
  }
}

void guards() {
  constexpr unsigned excessive_units = 1048577;
  constexpr unsigned full_accumulator_add = 0x200010;
  require(!blofeld::inspect_native_dependencies(0x1000000, 1), "Non-word opcode refuses");
  require(!blofeld::inspect_native_dependencies(0, 0), "Unknown base cost refuses");
  require(!blofeld::inspect_native_dependencies(0, excessive_units), "Unbounded base cost refuses");
  const auto add = blofeld::inspect_native_dependencies(full_accumulator_add, 1);
  require(add && add->arithmetic_writes == 1 && add->status_write,
          "Full accumulator ADD occupies the result and status paths");
  const blofeld::DspInterlocks history;
  auto invalid = blofeld::DspInterlockInstruction{.native_units = 1, .agu_reads = 256};
  bool refused = false;
  try {
    static_cast<void>(history.preview(invalid));
  } catch (const blofeld::Fault &) { refused = true; }
  require(refused, "Out-of-range dependency mask refuses without changing history");
  invalid.agu_reads = 0;
  require(history.preview(invalid) == 0, "Rejected descriptor preserves history");
}
}

int main() {
  try {
    reference_windows();
    guards();
    assertions += blofeld::test::native_dependency_operand_checks();
    assertions += blofeld::test::native_dependency_alu_checks();
    assertions += blofeld::test::native_dependency_scalar_checks();
    assertions += blofeld::test::native_dependency_parallel_checks();
    assertions += blofeld::test::native_dependency_address_checks();
    std::cout << "PASS native dependencies: " << assertions << " independent checks\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
