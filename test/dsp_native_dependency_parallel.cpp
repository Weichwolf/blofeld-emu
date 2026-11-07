// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_interlocks.hpp"
#include "dsp/dsp_native_dependencies.hpp"
#include "dsp56300/core.hpp"
#include "dsp_native_dependency_operands.hpp"
#include "dsp_native_parallel_cases.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace blofeld::test {
namespace {
unsigned checks = 0;
constexpr unsigned normal_mode = 0x305, masked_status = 0x300;
constexpr std::uint64_t input_x = 0x020304010203, input_y = 0x040506030405;
constexpr unsigned move_only = 0x200000, read_status = 0x0445b9;
constexpr std::array readers{0x218400U, 0x21a400U};

void require(const bool condition) {
  ++checks;
  if (!condition) { throw std::runtime_error("Independent parallel ALU dependency expectation"); }
}

DspInterlockInstruction describe(const unsigned opcode) {
  const auto decoded = inspect_native_dependencies(opcode, 1);
  if (!decoded) {
    throw std::runtime_error("Missing authored parallel ALU dependency description");
  }
  ++checks;
  return *decoded;
}

void retire(DspInterlocks &history, const DspInterlockInstruction &instruction) {
  const auto delay = history.preview(instruction);
  history.prepare(instruction);
  history.observe_opcode(instruction.opcode);
  require(history.complete(instruction.native_units) == delay);
}

struct Resources {
  unsigned accumulator;
  bool status;
};

void dependencies(const unsigned opcode, const Resources resources) {
  const auto decoded = describe(opcode);
  require(decoded.arithmetic_writes == resources.accumulator &&
          decoded.status_write == resources.status && decoded.move_reads == 0 &&
          decoded.move_writes == 0 && decoded.agu_reads == 0 && decoded.agu_writes == 0 &&
          !decoded.status_read && !decoded.partial_status_source);
  const auto nop = describe(0);
  const auto status = describe(read_status);
  for (unsigned gap = 0; gap < 4; ++gap) {
    DspInterlocks history;
    retire(history, decoded);
    for (unsigned count = 0; count < gap; ++count) { retire(history, nop); }
    for (unsigned accumulator = 0; accumulator < readers.size(); ++accumulator) {
      const auto delay = (resources.accumulator & (1U << accumulator)) != 0 && gap == 0 ? 1U : 0U;
      require(history.preview(describe(readers.at(accumulator))) == delay);
    }
    require(history.preview(status) == (resources.status && gap < 2 ? 2U - gap : 0U));
    auto restored = history;
    retire(restored, nop);
    require(restored.preview(status) == (resources.status && gap == 0 ? 1U : 0U));
  }
}

void actual_retirement(const parallel_runtime::Sample &sample) {
  std::array<dsp56300::Word, 1> program{sample.opcode};
  std::array<dsp56300::Word, 1> data{};
  for (const auto backend :
       {dsp56300::Backend::reference, dsp56300::Backend::cached, dsp56300::Backend::jit}) {
    dsp56300::Core cpu({.program = program, .x = data, .y = data}, nullptr, {.backend = backend});
    auto before = cpu.registers();
    before.omr = normal_mode;
    before.sr = masked_status;
    before.x = input_x;
    before.y = input_y;
    before.a = sample.before_a;
    before.b = sample.before_b;
    cpu.set_registers(before);
    auto expected = before;
    expected.pc = 1;
    expected.a = sample.after_a;
    expected.b = sample.after_b;
    expected.sr = sample.status;
    const auto result = cpu.run(1);
    ++checks;
    if (result.dispatches != 1 || result.fault.kind != dsp56300::FaultKind::none ||
        cpu.registers() != expected ||
        cpu.progress() != dsp56300::Progress{.instructions = 1, .estimated_cycles = 1}) {
      throw std::runtime_error("Independent full parallel ALU state: " + std::string(sample.name));
    }
  }
}

void complete_selector_table() {
  // FM parallel DALU table: TST, CMP and CMPM leave both accumulators unchanged.
  constexpr std::array flags_only{0x03U, 0x0bU, 0x05U, 0x0dU, 0x45U, 0x4dU, 0x55U, 0x5dU,
                                  0x65U, 0x6dU, 0x75U, 0x7dU, 0x07U, 0x0fU, 0x47U, 0x4fU,
                                  0x57U, 0x5fU, 0x67U, 0x6fU, 0x77U, 0x7fU};
  constexpr std::array reserved{4U, 8U, 12U};
  constexpr unsigned selector_count = 256;
  constexpr unsigned maximum = 0x1d;
  constexpr unsigned maximum_magnitude = 0x15;
  for (unsigned alu = 0; alu < selector_count; ++alu) {
    if (std::ranges::find(reserved, alu) != reserved.end()) {
      require(!inspect_native_dependencies(move_only | alu, 1));
      continue;
    }
    const bool no_result = alu == 0 || std::ranges::find(flags_only, alu) != flags_only.end();
    const auto destination = (alu & 8U) != 0 ? 2U : 1U;
    const auto result = no_result ? 0U : destination;
    const auto resources = alu == maximum || alu == maximum_magnitude ? 2U : result;
    dependencies(move_only | alu, {.accumulator = resources, .status = alu != 0});
  }
}
}

unsigned native_dependency_parallel_checks() {
  checks = 0;
  complete_selector_table();
  for (const auto &sample : parallel_runtime::samples) {
    dependencies(sample.opcode,
                 {.accumulator = sample.resources, .status = sample.status_resource});
    actual_retirement(sample);
  }
  return checks;
}
}
