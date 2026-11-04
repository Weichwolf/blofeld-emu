// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_interlocks.hpp"
#include "dsp/dsp_native_dependencies.hpp"
#include "dsp56300/core.hpp"
#include "dsp_native_dependency_operands.hpp"

#include <array>
#include <cstdint>
#include <stdexcept>

namespace blofeld::test {
namespace {
unsigned checks = 0;
constexpr std::array operations{0U, 2U, 3U, 4U, 6U};
constexpr std::array sources{4U, 5U, 6U, 7U};
constexpr std::array<unsigned, 2> readers{0x218400, 0x21a400}; // MOVE A1/B1,X0.
constexpr unsigned status_read = 0x0445b9;                     // MOVEC SR,X1.

constexpr std::array data_words{0x010203U, 0x020304U, 0x030405U, 0x040506U};

struct Sample {
  unsigned opcode, operation, accumulator, source;
};

void require(const bool value) {
  ++checks;
  if (!value) { throw std::runtime_error("Independent binary ALU dependency expectation"); }
}

DspInterlockInstruction describe(const unsigned opcode, const unsigned base = 1) {
  const auto result = inspect_native_dependencies(opcode, base);
  if (!result) { throw std::runtime_error("Missing authored ALU dependency description"); }
  ++checks;
  return *result;
}

void retire(DspInterlocks &history, const DspInterlockInstruction &instruction) {
  const auto before = history.preview(instruction);
  history.prepare(instruction);
  history.observe_opcode(instruction.opcode);
  require(history.complete(instruction.native_units) == before);
}

void dependencies(const Sample sample) {
  const auto opcode = sample.opcode;
  const auto accumulator = sample.accumulator;
  const auto writer = describe(opcode);
  const auto mask = 1U << accumulator;
  require(writer.native_units == 1 && writer.arithmetic_writes == mask && writer.status_write &&
          writer.move_reads == 0 && writer.move_writes == 0 && writer.agu_reads == 0 &&
          writer.agu_writes == 0 && !writer.status_read && !writer.partial_status_source);
  const auto consumer = describe(readers.at(accumulator));
  const auto other = describe(readers.at(1U - accumulator));
  const auto status = describe(status_read);
  for (unsigned gap = 0; gap < 4; ++gap) {
    DspInterlocks history;
    retire(history, writer);
    for (unsigned count = 0; count < gap; ++count) { retire(history, describe(0)); }
    require(history.preview(consumer) == (gap == 0 ? 1U : 0U));
    require(history.preview(other) == 0);
    require(history.preview(status) == (gap < 2 ? 2U - gap : 0U));
    auto restored = history;
    retire(restored, consumer);
    require(restored.preview(consumer) == 0);
  }
}

std::uint64_t value(const Sample sample) {
  const auto operation = sample.operation;
  const auto input = data_words.at(sample.source - 4);
  // Literal nonsaturating operands, no rounding; four distinct source words.
  constexpr std::uint64_t initial = 0x123456;
  constexpr unsigned shift = 24;
  switch (operation) {
    case 0: return (initial + input) << shift;
    case 2: return (initial | input) << shift;
    case 3: return (initial ^ input) << shift;
    case 4: return (initial - input) << shift;
    case 6: return (initial & input) << shift;
    default: throw std::runtime_error("Unexpected authored binary ALU function");
  }
}

void actual_retirement(const Sample sample) {
  const auto opcode = sample.opcode;
  const auto operation = sample.operation;
  const auto accumulator = sample.accumulator;
  constexpr unsigned word_bits = 24;
  std::array<dsp56300::Word, 1> program{opcode};
  std::array<dsp56300::Word, 1> data{};
  constexpr std::uint64_t initial = 0x123456000000;
  for (const auto backend :
       {dsp56300::Backend::reference, dsp56300::Backend::cached, dsp56300::Backend::jit}) {
    dsp56300::Core cpu({.program = program, .x = data, .y = data}, nullptr, {.backend = backend});
    auto before = cpu.registers();
    before.x = (std::uint64_t{data_words[2]} << word_bits) | data_words[0];
    before.y = (std::uint64_t{data_words[3]} << word_bits) | data_words[1];
    before.a = before.b = initial;
    cpu.set_registers(before);
    auto expected = before;
    expected.pc = 1;
    (accumulator == 0 ? expected.a : expected.b) = value(sample);
    // Logical functions update N/Z/V only; ADD/SUB also report U for these small values.
    expected.sr = operation == 0 || operation == 4 ? 0x10U : 0U;
    const auto result = cpu.run(1);
    require(result.dispatches == 1 && result.fault.kind == dsp56300::FaultKind::none &&
            cpu.registers() == expected &&
            cpu.progress() == dsp56300::Progress{.instructions = 1, .estimated_cycles = 1});
  }
}
}

unsigned native_dependency_alu_checks() {
  checks = 0;
  for (const auto source : sources) {
    for (const auto operation : operations) {
      for (unsigned accumulator = 0; accumulator < 2; ++accumulator) {
        const auto opcode = 0x200000U | (source << 4) | (accumulator << 3) | operation;
        const Sample sample{
            .opcode = opcode, .operation = operation, .accumulator = accumulator, .source = source};
        dependencies(sample);
        actual_retirement(sample);
      }
    }
  }
  // Retain all former guard stimuli; independently qualified carry/TFR/CMP now describe.
  for (const unsigned opcode : {0x200004U, 0x200008U, 0x20000cU}) {
    require(!inspect_native_dependencies(opcode, 1));
  }
  constexpr unsigned compare_x0_a = 0x200045;
  for (const unsigned opcode : {0x200021U, 0x200041U, compare_x0_a}) {
    const auto decoded = describe(opcode);
    require(decoded.arithmetic_writes == (opcode == compare_x0_a ? 0U : 1U) &&
            decoded.status_write);
  }
  return checks;
}
}
