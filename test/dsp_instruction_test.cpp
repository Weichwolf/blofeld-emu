// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp56kEmu/assembler.h"
#include "dsp_instruction_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_operand_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace blofeld::test {
namespace {
constexpr unsigned cookie = 0x123456;
constexpr unsigned second_cookie = 0x654321;
constexpr unsigned missing_extension = 0x44f400;
constexpr unsigned jump_absolute = 0x0af080;
constexpr unsigned status_index = 13;
constexpr unsigned minimum_input_bytes = 45;
constexpr std::uint64_t expected_sum = 0x9000000;
constexpr unsigned next_data_offset = 0x010101;

InstructionInput own_case(const std::vector<const char *> &source) {
  InstructionInput result;
  result.registers.at(0) = cookie;
  result.registers.at(1) = second_cookie;
  result.registers.at(2) = cookie ^ second_cookie;
  result.registers.at(5) = 2;
  result.registers.at(10) = instruction_data_begin;
  result.registers.at(11) = 1;
  result.registers.at(12) = word_mask;
  const dsp56k::Assembler assembler;
  for (const auto *line : source) {
    const auto assembled = assembler.assemble(line);
    require(assembled.success(),
            "Cannot assemble own native instruction seed: " + std::string(line));
    for (unsigned index = 0; index < assembled.wordCount; ++index) {
      result.program.push_back(assembled.word[index]);
    }
  }
  return result;
}

std::vector<std::uint8_t> encoded(const InstructionInput &input) {
  std::vector<std::uint8_t> bytes;
  const auto append = [&](const std::uint32_t value) {
    constexpr unsigned byte_mask = 0xff;
    for (unsigned index = 0; index < 3; ++index) {
      bytes.push_back(static_cast<std::uint8_t>((value >> ((2 - index) * 8U)) & byte_mask));
    }
  };
  for (const auto value : input.registers) { append(value); }
  for (const auto value : input.program) { append(value); }
  return bytes;
}

void test_input_bounds_and_literal_values() {
  auto bytes = encoded(own_case({"nop"}));
  constexpr std::array<std::uint8_t, 3> first{0x12, 0x34, 0x56};
  require(bytes.size() == minimum_input_bytes &&
              std::equal(first.begin(), first.end(), bytes.begin()),
          "Own input format uses a literal big-endian 42-byte header and one instruction");
  const auto input = decode_instruction_input(bytes);
  require(input && input->registers.at(0) == cookie && input->program == std::vector<unsigned>{0},
          "Literal header and NOP bytes must decode independently of execution");
  for (std::size_t size = 0; size < bytes.size(); ++size) {
    require(!decode_instruction_input(std::span(bytes).first(size)),
            "Every truncated minimum input must reject before native allocation");
  }
  bytes.push_back(0);
  require(!decode_instruction_input(bytes), "Incomplete raw instruction words must reject");
  bytes.resize(instruction_input_bytes + 3);
  require(!decode_instruction_input(bytes),
          "Oversize native programs must reject before execution");
}

void test_native_arithmetic_and_memory() {
  auto input = own_case({"move #>$7,x0", "add x0,a"});
  const auto arithmetic = execute_instruction_input(input, 2);
  require(arithmetic.stop == InstructionStop::budget && arithmetic.steps == 2 &&
              arithmetic.state.cpu.pc == 3 && arithmetic.state.cpu.a == expected_sum &&
              (arithmetic.state.cpu.x & word_mask) == 7,
          "Native ADD must add X0 at the A1 position after both executed input instructions");
  const auto memory = execute_instruction_input(own_case({"move x1,x:(r0)+"}), 1);
  require(memory.stop == InstructionStop::budget && memory.x.at(0) == second_cookie &&
              memory.x.at(1) == (cookie ^ next_data_offset) &&
              memory.state.cpu.r.at(0) == instruction_data_begin + 1,
          "Native store/postincrement changes exactly the expected RAM cell and address");
  const auto read = execute_instruction_input(own_case({"move x:(r0)+,y0"}), 1);
  require((read.state.cpu.y & word_mask) == cookie &&
              read.state.cpu.r.at(0) == instruction_data_begin + 1,
          "Native load reads the explicit own RAM seed and advances its EA once");
}

void test_bounded_branches_and_repeat() {
  auto branch = own_case({"nop"});
  branch.program = {jump_absolute, 0};
  const auto loop = execute_instruction_input(branch);
  require(loop.stop == InstructionStop::budget && loop.steps == instruction_step_limit &&
              loop.state.cpu.pc == 0,
          "A native self-branch must return at the fixture budget without skipping instructions");
  const auto repeated = own_case({"rep #3", "nop"});
  const auto partial = execute_instruction_input(repeated, 2);
  require(partial.stop == InstructionStop::budget && partial.state.cpu.pc == 1 &&
              partial.state.cpu.lc == 2,
          "One REP and one yielded body step leave two actual repetitions pending");
  const auto complete = execute_instruction_input(repeated, 4);
  require(complete.stop == InstructionStop::budget && complete.state.cpu.pc == 2 &&
              complete.state.cpu.lc == 0,
          "Three yielded REP body steps finish at the next word and restore LC");
}

void test_strict_extension_and_reserved_memory() {
  auto truncated = own_case({"nop"});
  truncated.program = {missing_extension};
  const auto missing = execute_instruction_input(truncated);
  require(missing.stop == InstructionStop::rejected && missing.steps == 0 &&
              missing.fault.contains("pc=0x000000") && missing.fault.contains("omr=0x000005") &&
              missing.fault.contains("Unknown DSP memory read at P:0x000001"),
          "Missing used extension must retain native PC/OMR/address and fail-stop");
  const auto reserved = execute_instruction_input(own_case({"move x0,y:>$2000"}));
  require(reserved.stop == InstructionStop::rejected && reserved.steps == 0 &&
              reserved.fault.contains("Unmapped DSP memory write at Y:0x002000") &&
              reserved.x.at(0) == cookie,
          "Arbitrary native inputs must not invent reserved RAM or alter unrelated own data");
}

void test_unqualified_execution_control() {
  struct Control {
    const char *name;
    std::uint32_t opcode;
    const char *operation;
  };

  // Independent literal formats: DSP56300FM 13-49/76/170/179/183.
  constexpr std::array<Control, 5> controls{
      {{.name = "trap", .opcode = 0x000006, .operation = "TRAP"},
       {.name = "illegal", .opcode = 0x000005, .operation = "ILLEGAL"},
       {.name = "debug", .opcode = 0x000200, .operation = "DEBUG"},
       {.name = "stop", .opcode = 0x000087, .operation = "STOP"},
       {.name = "wait", .opcode = 0x000086, .operation = "WAIT"}}};
  for (const auto control : controls) {
    const auto input = own_case({control.name});
    require(input.program == std::vector<std::uint32_t>{control.opcode},
            "Assembler must match the independent execution-control literal");
    const auto result = execute_instruction_input(input);
    require(result.stop == InstructionStop::rejected && result.steps == 0 &&
                result.fault.contains("Unqualified DSP reference") &&
                result.fault.contains(control.operation) && result.fault.contains("pc=0x000000"),
            "Unqualified native exception/halt must fail at its own instruction: " +
                std::string(control.name));
  }
}

void test_conditional_execution_control(const std::string_view prefix) {
  require(prefix == "trap" || prefix == "debug", "Unknown conditional control test group");

  struct Condition {
    const char *suffix;
    unsigned true_status;
    unsigned false_status;
  };

  // Independent CCCC order and flag truth vectors: FM tables 12-17/18.
  constexpr std::array<Condition, 16> conditions{{
      {.suffix = "cc", .true_status = 0, .false_status = carry},
      {.suffix = "ge", .true_status = negative | overflow, .false_status = overflow},
      {.suffix = "ne", .true_status = carry, .false_status = zero},
      {.suffix = "pl", .true_status = overflow, .false_status = negative},
      {.suffix = "nn", .true_status = unnormalized, .false_status = 0},
      {.suffix = "ec", .true_status = unnormalized, .false_status = extension},
      {.suffix = "lc", .true_status = sticky, .false_status = limit},
      {.suffix = "gt", .true_status = negative | overflow, .false_status = zero},
      {.suffix = "cs", .true_status = carry, .false_status = 0},
      {.suffix = "lt", .true_status = overflow, .false_status = negative | overflow},
      {.suffix = "eq", .true_status = zero, .false_status = carry},
      {.suffix = "mi", .true_status = negative, .false_status = overflow},
      {.suffix = "nr", .true_status = zero | unnormalized | extension, .false_status = extension},
      {.suffix = "es", .true_status = extension, .false_status = unnormalized},
      {.suffix = "ls", .true_status = limit, .false_status = sticky},
      {.suffix = "le", .true_status = zero | negative, .false_status = negative | overflow},
  }};

  struct Operation {
    const char *prefix;
    const char *diagnostic;
    unsigned opcode;
  };

  // Independent fixed opcode fields: FM 13-50 and 13-180.
  constexpr std::array<Operation, 2> operations{{
      {.prefix = "trap", .diagnostic = "TRAP", .opcode = 0x000010},
      {.prefix = "debug", .diagnostic = "DEBUG", .opcode = 0x000300},
  }};
  for (const auto operation : operations) {
    if (prefix != operation.prefix) { continue; }
    for (std::size_t index = 0; index < conditions.size(); ++index) {
      const auto condition = conditions.at(index);
      const auto source = std::string(operation.prefix) + condition.suffix;
      auto input = own_case({source.c_str()});
      const auto opcode = operation.opcode + static_cast<std::uint32_t>(index);
      require(input.program == std::vector<std::uint32_t>{opcode},
              "Conditional control encoding disagrees with the manual: " + source);
      for (const bool taken : {false, true}) {
        const auto status = taken ? condition.true_status : condition.false_status;
        input.registers.at(status_index) = status;
        const auto result = execute_instruction_input(input, 1);
        require(result.state.cpu.sr == (masked_status | status) && result.state.cpu.sp == 0 &&
                    result.x.at(0) == cookie && result.y.at(0) == (cookie ^ second_cookie),
                "Conditional control must preserve CCR, stack and seeded memory: " + source);
        if (taken) {
          require(result.stop == InstructionStop::rejected && result.steps == 0 &&
                      result.fault.contains(operation.diagnostic) &&
                      result.fault.contains("pc=0x000000"),
                  "Taken conditional control must report its unsupported state: " + source);
        } else {
          require(result.stop == InstructionStop::budget && result.steps == 1 &&
                      result.state.cpu.pc == 1 && result.fault.empty(),
                  "False conditional control must advance exactly one instruction: " + source);
        }
      }
    }
  }
}

void test_shift_count_boundaries() {
  // Own saved input from the earlier UBSan report: ASL #63,A,A; ASR #63,B,B.
  constexpr std::array<std::uint8_t, 48> regression{
      0x12, 0x34, 0x56, 0x65, 0x43, 0x21, 0x77, 0x77, 0x77, 0, 0,    0,    0,    0,    0,    0,
      0,    2,    0,    0,    0,    0,    0,    0,    0,    0, 0,    0,    0,    0,    0,    1,
      0,    0,    0,    1,    0xff, 0xff, 0xff, 0,    0,    0, 0x0c, 0x1d, 0x7e, 0x0c, 0x1c, 0xff};
  const auto input = own_case({"asr #$3f,b,b"});
  const auto right = execute_instruction_input(input, 1);
  require(right.stop == InstructionStop::rejected && right.steps == 0 &&
              right.fault.contains("ASR") && right.fault.contains("63"),
          "Undefined ASR count must reject before the native carry-bit shift");
  const auto saved = decode_instruction_input(regression);
  if (!saved) { throw Fault("Preserved own sanitizer input failed to decode"); }
  require(encoded(*saved) == std::vector(regression.begin(), regression.end()),
          "Preserved own sanitizer input must retain its exact header and instruction words");
  const auto left = execute_instruction_input(saved.value(), 1);
  require(left.stop == InstructionStop::rejected && left.steps == 0 && left.fault.contains("ASL") &&
              left.fault.contains("63"),
          "An unqualified ASL count must reject before native arithmetic");
  for (const auto *instruction : {"asr #$38,a,a", "asl #$38,a,a"}) {
    const auto boundary = execute_instruction_input(own_case({instruction}), 1);
    require(boundary.stop == InstructionStop::rejected && boundary.steps == 0 &&
                boundary.fault.contains("56"),
            "First unqualified normal shift count must reject at its own PC");
  }
  for (const auto *instruction : {"asr #$29,a,a", "asr x0,a,a"}) {
    constexpr unsigned sixteen_bit_arithmetic = 0x20000;
    constexpr unsigned first_undefined_count = 41;
    constexpr unsigned shift_count_mask = 0x3f;
    auto narrow = own_case({"move #>$c20300,sr", instruction});
    narrow.registers.at(0) = (cookie & ~shift_count_mask) | first_undefined_count;
    const auto boundary = execute_instruction_input(narrow, 2);
    require(boundary.stop == InstructionStop::rejected && boundary.steps == 1 &&
                boundary.state.cpu.sr == (masked_status | sixteen_bit_arithmetic) &&
                boundary.fault.contains("41") && boundary.fault.contains("pc=0x000002"),
            "SA-mode ASR immediate and low-six-bit register counts above 40 are undefined");
  }
}

void test_defined_shift_results() {
  struct Shift {
    const char *source;
    std::uint64_t value;
    std::uint64_t result;
    unsigned flags;
  };

  // Independent 56-bit shifts: last shifted bit supplies C; any sign change supplies V.
  constexpr std::array<Shift, 8> shifts{{
      {.source = "asr #0,a,a", .value = 0x01020304050607, .result = 0x01020304050607, .flags = 0},
      {.source = "asr #1,a,a", .value = 1, .result = 0, .flags = carry},
      {.source = "asr #55,a,a", .value = 0x40000000000000, .result = 0, .flags = carry},
      {.source = "asr #55,a,a",
       .value = accumulator_mask,
       .result = accumulator_mask,
       .flags = carry},
      {.source = "asl #0,a,a", .value = 0x123456, .result = 0x123456, .flags = 0},
      {.source = "asl #40,a,a", .value = 1, .result = 0x10000000000, .flags = 0},
      {.source = "asl #55,a,a", .value = 1, .result = sign_bit, .flags = overflow | limit},
      {.source = "asl #55,a,a", .value = 2, .result = 0, .flags = carry | overflow | limit},
  }};
  for (const auto shift : shifts) {
    auto input = own_case({shift.source});
    input.registers.at(4) = static_cast<std::uint32_t>(shift.value & word_mask);
    input.registers.at(5) = static_cast<std::uint32_t>((shift.value >> word_bits) & word_mask);
    input.registers.at(6) = static_cast<std::uint32_t>(shift.value >> data_bits);
    input.registers.at(status_index) =
        sticky | carry | overflow | zero | negative | extension | unnormalized;
    const auto result = execute_instruction_input(input, 1);
    const auto flags = sticky | shift.flags | value_flags({.value = shift.result});
    require(result.stop == InstructionStop::budget && result.steps == 1 &&
                result.state.cpu.pc == 1 && result.state.cpu.a == shift.result &&
                result.state.cpu.b == 0 && result.state.cpu.sr == (masked_status | flags) &&
                result.state.cpu.r.at(0) == instruction_data_begin,
            "Defined shift must retain its independent value, C/V/L and preserved state: " +
                std::string(shift.source));
  }
}

void write_seed(const std::filesystem::path &path, const InstructionInput &input) {
  std::ofstream output(path, std::ios::binary);
  output.exceptions(std::ios::failbit | std::ios::badbit);
  for (const auto value : encoded(input)) { output.put(static_cast<char>(value)); }
}

unsigned write_seeds(const std::filesystem::path &directory) {
  std::filesystem::create_directories(directory);
  using Source = std::vector<const char *>;
  const std::vector<std::pair<const char *, Source>> programs{
      {"nop", {"nop", "nop"}},
      {"arithmetic", {"add x0,a", "sub y0,b", "add b,a"}},
      {"agu", {"move x:(r0)+,x0", "move x0,y:(r1)+", "move (r0)+n0"}},
      {"repeat", {"rep #3", "nop", "nop"}},
      {"zero-repeat", {"rep #0", "nop", "nop"}},
      {"do-loop", {"do #3,$3", "nop", "nop"}},
      {"zero-do-loop", {"do #0,$3", "nop", "nop"}},
      {"program-store", {"move x0,p:(r0)", "nop"}},
      {"shift", {"asl #$3f,a,a", "asr #$3f,b,b"}},
      {"bits", {"bset #23,x:(r0)", "bchg #0,x:(r0)+", "bclr #1,x:(r0)"}},
      {"stack", {"jsr $4", "nop", "nop", "rts"}},
      {"normalization", {"norm r0,a", "norm r1,b"}},
      {"division", {"div x0,a", "div y1,b"}},
      {"reserved-write", {"move x0,y:>$2000"}},
      {"reset", {"reset", "nop"}},
      {"trap", {"trap"}},
      {"illegal", {"illegal"}},
      {"debug", {"debug"}},
      {"stop", {"stop"}},
      {"wait", {"wait"}},
      {"false-trap", {"trapeq"}},
      {"true-trap", {"trapne"}},
      {"false-debug", {"debugeq"}},
      {"true-debug", {"debugne"}}};
  for (const auto &[name, source] : programs) { write_seed(directory / name, own_case(source)); }
  auto missing = own_case({"nop"});
  missing.program = {missing_extension};
  write_seed(directory / "missing-extension", missing);
  auto branch = own_case({"nop"});
  branch.program = {jump_absolute, 0};
  write_seed(directory / "self-branch", branch);
  auto maximum = own_case({"rep x0", "nop"});
  maximum.registers.at(0) = word_mask;
  write_seed(directory / "long-repeat", maximum);
  auto compatibility_case = own_case({"rep #0", "nop"});
  compatibility_case.registers.at(status_index) = compatibility;
  write_seed(directory / "compatibility-repeat", compatibility_case);
  return static_cast<unsigned>(programs.size()) + 4;
}
}
}

int main(const int count, char **arguments) {
  try {
    if (count == 2 && std::string_view(arguments[1]) == "--operand-bounds") {
      blofeld::test::operands::logical_checks();
      blofeld::test::operands::bitfield_checks();
      blofeld::test::operands::rejection_checks();
      std::cout << "Native operand boundaries, literal results/flags and fault recovery passed.\n";
      return 0;
    }
    if (count == 2 && std::string_view(arguments[1]) == "--shift-counts") {
      blofeld::test::test_shift_count_boundaries();
      blofeld::test::test_defined_shift_results();
      std::cout << "Native shift-count regression and rejection boundaries passed.\n";
      return 0;
    }
    if (count == 3 && std::string_view(arguments[1]) == "--write-seeds") {
      const auto seeds = blofeld::test::write_seeds(arguments[2]);
      std::cout << "Wrote " << seeds << " own native instruction seeds.\n";
      return 0;
    }
    if (count == 3 && std::string_view(arguments[1]) == "--conditions") {
      blofeld::test::test_conditional_execution_control(arguments[2]);
      std::cout << "Conditional control encodings, flags, faults and preservation passed.\n";
      return 0;
    }
    blofeld::test::require(
        count == 1,
        "Usage: dsp-instruction-test [--write-seeds directory | --conditions trap|debug | "
        "--shift-counts | --operand-bounds]");
    blofeld::test::test_input_bounds_and_literal_values();
    blofeld::test::test_native_arithmetic_and_memory();
    blofeld::test::test_bounded_branches_and_repeat();
    blofeld::test::test_strict_extension_and_reserved_memory();
    blofeld::test::test_unqualified_execution_control();
    std::cout << "Native instruction fixture bounds, arithmetic, memory, REP and faults passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
