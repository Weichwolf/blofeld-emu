// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_interlocks.hpp"
#include "dsp/dsp_native_dependencies.hpp"
#include "dsp56300/core.hpp"
#include "dsp_native_address_cases.hpp"
#include "dsp_native_dependency_operands.hpp"

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace blofeld::test {
namespace {
unsigned checks = 0;
constexpr unsigned normal_mode = 0x305, masked_status = 0x300;
constexpr std::uint64_t initial_a = 0x123456400000, initial_b = 0x345678800000;
constexpr std::uint64_t input_y = 0x040506030405;
constexpr unsigned initial_r = 0x20, initial_n = 1, linear = 0xffffff;
constexpr unsigned memory_words = 128, x_pattern = 0x1000, y_pattern = 0x2000;

void require(const bool condition) {
  ++checks;
  if (!condition) { throw std::runtime_error("Independent AGU dependency expectation"); }
}

DspInterlockInstruction describe(const address_runtime::Instruction instruction) {
  const auto result = inspect_native_dependencies(instruction.words.front(), instruction.cost);
  if (!result) { throw std::runtime_error("Missing authored address dependency description"); }
  ++checks;
  return *result;
}

void retire(DspInterlocks &history, const DspInterlockInstruction &instruction) {
  const auto delay = history.preview(instruction);
  history.prepare(instruction);
  history.observe_opcode(instruction.opcode);
  require(history.complete(instruction.native_units) == delay);
}

void dependencies(const address_runtime::Sample &sample, const unsigned gap) {
  const auto producer = describe(sample.producer);
  const auto consumer = describe(sample.consumer);
  const auto nop = describe({.words = {}, .count = 1, .cost = 1});
  DspInterlocks history;
  retire(history, producer);
  for (unsigned index = 0; index < gap; ++index) { retire(history, nop); }
  require(history.preview(consumer) == sample.extra.at(gap));
  auto restored = history;
  retire(restored, consumer);
  require(restored.preview(consumer) == 0);
}

void actual_retirement(const address_runtime::Sample &sample, const unsigned gap) {
  std::array<dsp56300::Word, 8> program{};
  for (unsigned index = 0; index < sample.producer.count; ++index) {
    program.at(index) = sample.producer.words.at(index);
  }
  for (unsigned index = 0; index < sample.consumer.count; ++index) {
    program.at(sample.producer.count + gap + index) = sample.consumer.words.at(index);
  }
  for (const auto backend :
       {dsp56300::Backend::reference, dsp56300::Backend::cached, dsp56300::Backend::jit}) {
    std::array<dsp56300::Word, memory_words> x{};
    std::array<dsp56300::Word, memory_words> y{};
    for (unsigned index = 0; index < memory_words; ++index) {
      x.at(index) = x_pattern + index;
      y.at(index) = y_pattern + index;
    }
    auto expected_x = x;
    auto expected_y = y;
    if (sample.memory_write) {
      (sample.memory_y ? expected_y : expected_x).at(sample.memory_address) = sample.memory_value;
    }
    dsp56300::Core cpu({.program = program, .x = x, .y = y}, nullptr, {.backend = backend});
    auto before = cpu.registers();
    before.a = initial_a;
    before.b = initial_b;
    before.x = sample.before_x;
    before.y = input_y;
    before.omr = normal_mode;
    before.sr = masked_status;
    before.r.front() = initial_r;
    before.n.front() = initial_n;
    before.m.front() = linear;
    cpu.set_registers(before);
    auto expected = before;
    expected.pc = sample.producer.count + gap + sample.consumer.count;
    expected.a = sample.after_a;
    expected.b = sample.after_b;
    expected.x = sample.after_x;
    expected.y = sample.after_y;
    expected.sr = sample.status;
    expected.r.front() = sample.r0;
    expected.n.front() = sample.n0;
    expected.m.front() = sample.m0;
    const auto result = cpu.run(gap + 2);
    ++checks;
    if (result.dispatches != gap + 2 || result.fault.kind != dsp56300::FaultKind::none ||
        cpu.registers() != expected || x != expected_x || y != expected_y ||
        cpu.progress() != dsp56300::Progress{.instructions = gap + 2,
                                             .estimated_cycles = sample.producer.cost + gap +
                                                                 sample.consumer.cost}) {
      throw std::runtime_error("Independent full AGU state/RAM: " + std::string(sample.name));
    }
  }
}

void register_groups() {
  // Independently assembled encodings cover all eight AGU register groups.
  for (const auto sample : address_runtime::operands) {
    const auto decoded = describe({.words = {sample.opcode}, .count = 1, .cost = sample.cost});
    require(decoded.agu_reads == (1U << sample.group) && decoded.agu_writes == 0 &&
            decoded.move_reads == sample.move_reads && decoded.move_writes == 0 &&
            decoded.arithmetic_writes == 0 && decoded.status_write == sample.status &&
            !decoded.status_read && decoded.partial_status_source == sample.partial &&
            decoded.move_read_advance == sample.advance);
  }
  for (const auto opcode : address_runtime::guards) {
    require(!inspect_native_dependencies(opcode, 1));
  }
}
}

unsigned native_dependency_address_checks() {
  checks = 0;
  register_groups();
  for (const auto &sample : address_runtime::samples) {
    for (unsigned gap = 0; gap < sample.gaps; ++gap) {
      dependencies(sample, gap);
      actual_retirement(sample, gap);
    }
  }
  return checks;
}
}
