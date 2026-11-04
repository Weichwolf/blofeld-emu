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
constexpr unsigned word_bits = 24, arithmetic_counts = 64, logical_counts = 32;
constexpr unsigned accumulator_bits = 56;
constexpr std::uint64_t initial_word = 0x123456, initial = initial_word << word_bits;
constexpr unsigned short_value = 3, long_value = 0x010203;
constexpr unsigned read_status = 0x0445b9;
constexpr unsigned immediate_long_base = 0x0140c0, immediate_short_base = 0x014380;
constexpr std::array functions{0U, 2U, 3U, 4U, 5U, 6U};
constexpr std::array count_registers{2U, 3U, 4U, 5U, 6U, 7U};
constexpr std::array readers{0x218400U, 0x21a400U};

struct Sample {
  unsigned opcode = 0, extension = 0, words = 1;
  unsigned destination = 0, writers = 0, status = 0;
  std::uint64_t value = initial;
};

void require(const bool value) {
  ++checks;
  if (!value) { throw std::runtime_error("Independent scalar ALU dependency expectation"); }
}

DspInterlockInstruction describe(const unsigned opcode, const unsigned base = 1) {
  const auto decoded = inspect_native_dependencies(opcode, base);
  if (!decoded) { throw std::runtime_error("Missing authored scalar dependency description"); }
  ++checks;
  return *decoded;
}

void retire(DspInterlocks &history, const DspInterlockInstruction &instruction) {
  const auto delay = history.preview(instruction);
  history.prepare(instruction);
  history.observe_opcode(instruction.opcode);
  require(history.complete(instruction.native_units) == delay);
}

void dependencies(const Sample sample) {
  const auto decoded = describe(sample.opcode, sample.words);
  require(decoded.arithmetic_writes == sample.writers && decoded.status_write &&
          decoded.move_reads == 0 && decoded.move_writes == 0 && decoded.agu_reads == 0 &&
          decoded.agu_writes == 0 && !decoded.status_read && !decoded.partial_status_source);
  for (unsigned gap = 0; gap < 4; ++gap) {
    DspInterlocks history;
    retire(history, decoded);
    for (unsigned count = 0; count < gap; ++count) { retire(history, describe(0)); }
    for (unsigned accumulator = 0; accumulator < readers.size(); ++accumulator) {
      const auto stalls = (sample.writers & (1U << accumulator)) != 0 && gap == 0 ? 1U : 0U;
      require(history.preview(describe(readers.at(accumulator))) == stalls);
    }
    require(history.preview(describe(read_status)) == (gap < 2 ? 2U - gap : 0U));
    auto copy = history;
    retire(copy, describe(0));
    require(copy.preview(describe(read_status)) == (gap == 0 ? 1U : 0U));
  }
}

void actual_retirement(const Sample sample) {
  std::array<dsp56300::Word, 2> program{sample.opcode, sample.extension};
  std::array<dsp56300::Word, 1> data{};
  for (const auto backend :
       {dsp56300::Backend::reference, dsp56300::Backend::cached, dsp56300::Backend::jit}) {
    dsp56300::Core cpu({.program = program, .x = data, .y = data}, nullptr, {.backend = backend});
    auto before = cpu.registers();
    before.a = before.b = initial;
    before.x = 1; // Register-count cases use X0=1, independently assembled by NXP.
    cpu.set_registers(before);
    auto expected = before;
    expected.pc = sample.words;
    expected.sr = sample.status;
    (sample.destination == 0 ? expected.a : expected.b) = sample.value;
    const auto result = cpu.run(1);
    require(result.dispatches == 1 && result.fault.kind == dsp56300::FaultKind::none &&
            cpu.registers() == expected &&
            cpu.progress() ==
                dsp56300::Progress{.instructions = 1, .estimated_cycles = sample.words});
  }
}

struct ImmediateForm {
  unsigned function, destination;
  bool long_form;
};

Sample immediate(const ImmediateForm form) {
  const auto function = form.function;
  const auto long_form = form.long_form;
  const auto destination = form.destination;
  const std::uint64_t input = long_form ? long_value : short_value;
  auto value = initial_word;
  switch (function) {
    case 0: value += input; break;
    case 2: value |= input; break;
    case 3: value ^= input; break;
    case 4: value -= input; break;
    case 5: break; // CMP updates flags; the actual accumulator is unchanged.
    case 6: value &= input; break;
    default: throw std::runtime_error("Unexpected own immediate ALU function");
  }
  return {.opcode = (long_form ? immediate_long_base : immediate_short_base) | (destination << 3) |
                    function,
          .extension = long_form ? long_value : 0U,
          .words = long_form ? 2U : 1U,
          .destination = destination,
          .writers = function == 5 ? 0U : 1U << destination,
          .status = function == 0 || function == 4 || function == 5 ? 0x10U : 0U,
          .value = value << word_bits};
}

void immediate_cases() {
  for (const auto function : functions) {
    for (const bool long_form : {false, true}) {
      for (unsigned destination = 0; destination < 2; ++destination) {
        const auto sample =
            immediate({.function = function, .destination = destination, .long_form = long_form});
        dependencies(sample);
        actual_retirement(sample);
      }
    }
  }
}

struct Shift {
  unsigned immediate, registers;
  bool arithmetic, left;
};

constexpr std::array shifts{
    Shift{.immediate = 0x0c1d00, .registers = 0x0c1e40, .arithmetic = true, .left = true},
    Shift{.immediate = 0x0c1c00, .registers = 0x0c1e60, .arithmetic = true, .left = false},
    Shift{.immediate = 0x0c1e80, .registers = 0x0c1e10, .arithmetic = false, .left = true},
    Shift{.immediate = 0x0c1ec0, .registers = 0x0c1e30, .arithmetic = false, .left = false}};

void immediate_shifts(const Shift shift, const unsigned destination) {
  const auto source = shift.arithmetic ? destination << 7 : 0U;
  const auto limit = shift.arithmetic ? arithmetic_counts : logical_counts;
  for (unsigned count = 0; count < limit; ++count) {
    auto sample = Sample{.opcode = shift.immediate | source | (count << 1) | destination,
                         .destination = destination,
                         .writers = 1U << destination,
                         .status = shift.arithmetic ? 0x10U : 0U};
    if (shift.arithmetic && count >= accumulator_bits) {
      require(!inspect_native_dependencies(sample.opcode, 1));
      continue;
    }
    dependencies(sample);
    if (count < 2) {
      sample.value = shift.left ? initial << count : initial >> count;
      actual_retirement(sample);
    }
  }
}

void register_shifts(const Shift shift, const unsigned destination) {
  const auto source = shift.arithmetic ? destination << 4 : 0U;
  for (const auto reg : count_registers) {
    constexpr unsigned x0_selector = 4;
    auto sample = Sample{.opcode = shift.registers | source | (reg << 1) | destination,
                         .destination = destination,
                         .writers = 1U << destination,
                         .status = shift.arithmetic ? 0x10U : 0U,
                         .value = shift.left ? initial << 1 : initial >> 1};
    dependencies(sample);
    if (reg == x0_selector) { actual_retirement(sample); }
  }
}

void shift_cases() {
  for (const auto shift : shifts) {
    for (unsigned destination = 0; destination < 2; ++destination) {
      immediate_shifts(shift, destination);
      register_shifts(shift, destination);
    }
  }
}

struct ForwardCase {
  Shift shift;
  unsigned counter, gap;
  bool arithmetic_producer;
};

void changed_count(const ForwardCase sample) {
  constexpr unsigned subtract_long = 0x0140c4;
  constexpr unsigned load_count = 0x208c00;
  const auto counter = sample.counter;
  const auto destination = 1U - counter;
  const auto producer =
      sample.arithmetic_producer ? subtract_long | (counter << 3) : load_count | (counter << 8);
  const auto base = sample.arithmetic_producer ? 2U : 1U;
  const auto shift_source = sample.shift.arithmetic ? destination << 4 : 0U;
  const auto consumer = sample.shift.registers | shift_source | ((counter + 2) << 1) | destination;
  DspInterlocks history;
  retire(history, describe(producer, base));
  for (unsigned gap = 0; gap < sample.gap; ++gap) { retire(history, describe(0)); }
  require(history.preview(describe(consumer)) == 0);
  std::array<dsp56300::Word, 4> program{producer};
  if (sample.arithmetic_producer) { program[1] = 6; }
  program.at(base + sample.gap) = consumer;
  std::array<dsp56300::Word, 1> data{};
  for (const auto backend :
       {dsp56300::Backend::reference, dsp56300::Backend::cached, dsp56300::Backend::jit}) {
    dsp56300::Core cpu({.program = program, .x = data, .y = data}, nullptr, {.backend = backend});
    auto before = cpu.registers();
    before.x = 1;
    before.a = before.b = initial;
    (counter == 0 ? before.a : before.b) = std::uint64_t{7} << word_bits;
    cpu.set_registers(before);
    auto expected = before;
    expected.pc = base + sample.gap + 1;
    (counter == 0 ? expected.a : expected.b) = std::uint64_t{1} << word_bits;
    (destination == 0 ? expected.a : expected.b) = sample.shift.left ? initial << 1 : initial >> 1;
    expected.sr = sample.shift.arithmetic || sample.arithmetic_producer ? 0x10U : 0U;
    const auto result = cpu.run(2 + sample.gap);
    require(result.dispatches == 2 + sample.gap && result.fault.kind == dsp56300::FaultKind::none &&
            cpu.registers() == expected &&
            cpu.progress() == dsp56300::Progress{.instructions = 2 + sample.gap,
                                                 .estimated_cycles = base + sample.gap + 1});
  }
}

void forwarding_cases() {
  for (const auto shift : shifts) {
    for (unsigned counter = 0; counter < 2; ++counter) {
      for (const bool arithmetic : {false, true}) {
        for (const unsigned gap : {0U, 1U}) {
          changed_count(
              {.shift = shift, .counter = counter, .gap = gap, .arithmetic_producer = arithmetic});
        }
      }
    }
  }
}
}

unsigned native_dependency_scalar_checks() {
  checks = 0;
  immediate_cases();
  shift_cases();
  forwarding_cases();
  for (const unsigned opcode :
       {0x014381U, 0x014387U, 0x0140c1U, 0x0140c7U, 0x0c1e40U, 0x0c1e42U, 0x0c1e10U, 0x0c1e12U}) {
    require(!inspect_native_dependencies(opcode, 1));
  }
  return checks;
}
}
